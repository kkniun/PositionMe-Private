package com.openpositioning.PositionMe.sensors;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.openpositioning.PositionMe.utils.PdrProcessing;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

/**
 * Particle filter that fuses local PDR motion with absolute GNSS/WiFi fixes.
 */
public class ParticleFilterEngine {

    private static final int DEFAULT_PARTICLE_COUNT = 100;
    private static final double DEFAULT_INITIAL_STD_M = 2.0;
    private static final double DEFAULT_PREDICTION_NOISE_STD_M = 0.35;
    private static final double DEFAULT_HEADING_NOISE_STD_RAD = Math.toRadians(4.0);
    private static final double DEFAULT_ABSOLUTE_FIX_STD_M = 4.0;
    private static final double FLOOR_MISMATCH_PENALTY = 0.2;
    private static final double RESAMPLE_THRESHOLD_RATIO = 0.5;
    private static final double MIN_WEIGHT = 1e-12;
    private static final double RECOVERY_DISTANCE_STD_MULTIPLIER = 6.0;
    private static final double MIN_RECOVERY_DISTANCE_M = 8.0;
    private static final double MIN_SUPPORT_WEIGHT_RATIO = 0.2;
    private static final double MAX_CREDIBLE_RECOVERY_ACCURACY_M = 6.0;

    private final ParticleInitializer particleInitializer;
    private final ParticleInitializer.SpawnValidator spawnValidator;
    @Nullable
    private final FloorTransitionGate floorTransitionGate;
    private final Random random;
    private final List<Particle> particles = new ArrayList<>();

    private long lastTimestampMs;
    // 临时调试状态：用于判断 PF 是否被楼层/墙体约束卡住，定位完成后可删除。
    private int lastPredictWallRejectCount;
    private int lastPredictFloorConstraintRejectCount;
    private boolean lastAbsoluteFixReanchored;
    private boolean lastAbsoluteFixRejectedByConstraints;

    static final class StateSnapshot {
        private final List<Particle> particles;
        private final long lastTimestampMs;
        private final int lastPredictWallRejectCount;
        private final int lastPredictFloorConstraintRejectCount;
        private final boolean lastAbsoluteFixReanchored;
        private final boolean lastAbsoluteFixRejectedByConstraints;

        StateSnapshot(
                @NonNull List<Particle> particles,
                long lastTimestampMs,
                int lastPredictWallRejectCount,
                int lastPredictFloorConstraintRejectCount,
                boolean lastAbsoluteFixReanchored,
                boolean lastAbsoluteFixRejectedByConstraints
        ) {
            this.particles = new ArrayList<>(particles.size());
            for (Particle particle : particles) {
                this.particles.add(new Particle(particle));
            }
            this.lastTimestampMs = lastTimestampMs;
            this.lastPredictWallRejectCount = lastPredictWallRejectCount;
            this.lastPredictFloorConstraintRejectCount = lastPredictFloorConstraintRejectCount;
            this.lastAbsoluteFixReanchored = lastAbsoluteFixReanchored;
            this.lastAbsoluteFixRejectedByConstraints = lastAbsoluteFixRejectedByConstraints;
        }
    }

    public ParticleFilterEngine() {
        this(new ParticleInitializer(), ParticleInitializer.allowAll(), null, new Random());
    }

    public ParticleFilterEngine(
            ParticleInitializer particleInitializer,
            ParticleInitializer.SpawnValidator spawnValidator
    ) {
        this(particleInitializer, spawnValidator, null, new Random());
    }

    /**
     * @param floorTransitionGate if non-null, floor changes are only kept when the gate allows
     *                            (e.g. user must be in stairs/lift zone). Null disables gating.
     */
    public ParticleFilterEngine(
            ParticleInitializer particleInitializer,
            ParticleInitializer.SpawnValidator spawnValidator,
            @Nullable FloorTransitionGate floorTransitionGate
    ) {
        this(particleInitializer, spawnValidator, floorTransitionGate, new Random());
    }

    ParticleFilterEngine(
            ParticleInitializer particleInitializer,
            ParticleInitializer.SpawnValidator spawnValidator,
            Random random
    ) {
        this(particleInitializer, spawnValidator, null, random);
    }

    ParticleFilterEngine(
            ParticleInitializer particleInitializer,
            ParticleInitializer.SpawnValidator spawnValidator,
            @Nullable FloorTransitionGate floorTransitionGate,
            Random random
    ) {
        this.particleInitializer = particleInitializer;
        this.spawnValidator = spawnValidator == null
                ? ParticleInitializer.allowAll()
                : spawnValidator;
        this.floorTransitionGate = floorTransitionGate;
        this.random = random;
    }

    public void initialize(double fixX, double fixY, int floor, long timestampMs) {
        initialize(fixX, fixY, floor, timestampMs, 0.0, DEFAULT_PARTICLE_COUNT, DEFAULT_INITIAL_STD_M);
    }

    public void initialize(double fixX, double fixY, int floor, long timestampMs, double headingRad) {
        initialize(fixX, fixY, floor, timestampMs, headingRad, DEFAULT_PARTICLE_COUNT, DEFAULT_INITIAL_STD_M);
    }

    public void initialize(
            double fixX,
            double fixY,
            int floor,
            long timestampMs,
            double headingRad,
            double positionStdMeters
    ) {
        initialize(fixX, fixY, floor, timestampMs, headingRad, DEFAULT_PARTICLE_COUNT, positionStdMeters);
    }

    public void initialize(double fixX, double fixY, int floor, long timestampMs, int particleCount) {
        initialize(fixX, fixY, floor, timestampMs, 0.0, particleCount, DEFAULT_INITIAL_STD_M);
    }

    public void initialize(
            double fixX,
            double fixY,
            int floor,
            long timestampMs,
            double headingRad,
            int particleCount,
            double positionStdMeters
    ) {
        resetLastAbsoluteFixOutcomeFlags();
        particles.clear();
        particles.addAll(particleInitializer.initialize(
                fixX,
                fixY,
                floor,
                particleCount,
                positionStdMeters,
                headingRad,
                spawnValidator
        ));
        lastTimestampMs = timestampMs;
        normalizeWeights();
    }

    public void predict(PdrDelta delta, int floor, long timestampMs) {
        if (particles.isEmpty() || delta == null) {
            return;
        }

        lastPredictWallRejectCount = 0;
        lastPredictFloorConstraintRejectCount = 0;
        double stepLengthMeters = delta.getStepLengthMeters();
        double deltaHeadingRad = delta.getDeltaHeadingRad();
        for (Particle particle : particles) {
            double previousX = particle.getX();
            double previousY = particle.getY();
            int previousFloor = particle.getFloor();
            double previousHeadingRad = particle.getHeadingRad();

            double predictedHeadingRad = normalizeHeading(
                    previousHeadingRad
                            + deltaHeadingRad
                            + random.nextGaussian() * DEFAULT_HEADING_NOISE_STD_RAD
            );
            double[] localStep = PdrProcessing.projectStepToLocalFrame(stepLengthMeters, predictedHeadingRad);
            double predictedX = previousX
                    + localStep[0]
                    + random.nextGaussian() * DEFAULT_PREDICTION_NOISE_STD_M;
            double predictedY = previousY
                    + localStep[1]
                    + random.nextGaussian() * DEFAULT_PREDICTION_NOISE_STD_M;
            int predictedFloor = resolvePredictedFloor(previousFloor, floor);

            if (predictedFloor != previousFloor
                    && floorTransitionGate != null
                    && !floorTransitionGate.allowsFloorChange(
                            previousX,
                            previousY,
                            predictedX,
                            predictedY,
                            previousFloor,
                            predictedFloor
                    )) {
                lastPredictFloorConstraintRejectCount++;
                predictedFloor = previousFloor;
            }

            // Map matching / walls: reject illegal segments, not only illegal endpoints.
            if (!spawnValidator.isValidMotion(
                    previousX,
                    previousY,
                    predictedX,
                    predictedY,
                    previousFloor,
                    predictedFloor
            )) {
                lastPredictWallRejectCount++;
                resetParticleAfterInvalidPrediction(
                        particle,
                        previousX,
                        previousY,
                        previousFloor,
                        previousHeadingRad
                );
                predictedX = particle.getX();
                predictedY = particle.getY();
                predictedFloor = particle.getFloor();
                predictedHeadingRad = particle.getHeadingRad();
            }

            particle.setX(predictedX);
            particle.setY(predictedY);
            particle.setFloor(predictedFloor);
            particle.setHeadingRad(predictedHeadingRad);
        }

        lastTimestampMs = timestampMs;
        normalizeWeights();
        resampleIfNeeded();
    }

    public boolean forceFloor(int floor, long timestampMs) {
        if (particles.isEmpty()) {
            lastTimestampMs = timestampMs;
            return false;
        }
        if (!canForceFloor(floor)) {
            return false;
        }
        for (Particle particle : particles) {
            particle.setFloor(floor);
        }
        lastTimestampMs = timestampMs;
        return true;
    }

    public void updateWithAbsoluteFix(double fixX, double fixY, int floor, long timestampMs) {
        updateWithAbsoluteFix(fixX, fixY, floor, timestampMs, DEFAULT_ABSOLUTE_FIX_STD_M);
    }

    public void updateWithAbsoluteFix(
            double fixX,
            double fixY,
            int floor,
            long timestampMs,
            double accuracyMeters
    ) {
        updateWithAbsoluteFix(fixX, fixY, Integer.valueOf(floor), timestampMs, accuracyMeters);
    }

    void updateWithAbsoluteFix(
            double fixX,
            double fixY,
            @Nullable Integer floorPrior,
            long timestampMs,
            double accuracyMeters
    ) {
        double measurementStdMeters = sanitizeAccuracyMeters(accuracyMeters);
        boolean reanchorRequired = !particles.isEmpty()
                && shouldReanchorToAbsoluteFix(fixX, fixY, measurementStdMeters);
        Integer acceptedFloorPrior = sanitizeAbsoluteFloorPrior(fixX, fixY, floorPrior, reanchorRequired);
        int validationFloor = resolveAbsoluteFixValidationFloor(fixX, fixY, acceptedFloorPrior, floorPrior);
        lastAbsoluteFixReanchored = false;
        lastAbsoluteFixRejectedByConstraints = false;
        if (isAbsoluteFixInvalidUnderConstraints(fixX, fixY, validationFloor)) {
            lastAbsoluteFixRejectedByConstraints = true;
            return;
        }
        if (particles.isEmpty()) {
            List<Particle> initializedParticles = particleInitializer.initialize(
                    fixX,
                    fixY,
                    resolveInitializationFloor(acceptedFloorPrior),
                    DEFAULT_PARTICLE_COUNT,
                    sanitizeAccuracyMeters(accuracyMeters),
                    0.0,
                    spawnValidator
            );
            if (initializedParticles.isEmpty()) {
                lastAbsoluteFixRejectedByConstraints = true;
                return;
            }
            particles.clear();
            particles.addAll(initializedParticles);
            lastTimestampMs = timestampMs;
            normalizeWeights();
            return;
        }

        if (reanchorRequired) {
            boolean hasMotionSupport = hasMotionCompatibleSupportForAbsoluteFix(
                    fixX,
                    fixY,
                    validationFloor,
                    MIN_SUPPORT_WEIGHT_RATIO
            );
            boolean useCredibleRecoveryReanchor = !hasMotionSupport
                    && isCredibleRecoveryReanchor(fixX, fixY, validationFloor, measurementStdMeters);
            if (!hasMotionSupport && !useCredibleRecoveryReanchor) {
                lastAbsoluteFixRejectedByConstraints = true;
                return;
            }
            lastAbsoluteFixReanchored = true;
            reanchorToAbsoluteFix(
                    fixX,
                    fixY,
                    acceptedFloorPrior,
                    timestampMs,
                    measurementStdMeters,
                    useCredibleRecoveryReanchor
            );
            return;
        }

        if (!hasMotionCompatibleSupportForAbsoluteFix(fixX, fixY, validationFloor, Double.MIN_VALUE)) {
            lastAbsoluteFixRejectedByConstraints = true;
            return;
        }

        double variance = measurementStdMeters * measurementStdMeters;
        for (Particle particle : particles) {
            if (!isParticleMotionCompatibleWithAbsoluteFix(particle, fixX, fixY, validationFloor)) {
                particle.setWeight(MIN_WEIGHT);
                continue;
            }
            double dx = particle.getX() - fixX;
            double dy = particle.getY() - fixY;
            double distanceSq = dx * dx + dy * dy;
            double spatialLikelihood = Math.exp(-0.5 * distanceSq / variance);
            double floorLikelihood = getFloorLikelihood(particle.getFloor(), acceptedFloorPrior);
            double updatedWeight = sanitizeWeight(particle.getWeight()) * spatialLikelihood * floorLikelihood;
            particle.setWeight(Math.max(updatedWeight, MIN_WEIGHT));
        }

        lastTimestampMs = timestampMs;
        normalizeWeights();
        resampleIfNeeded();
    }

    public void normalizeWeights() {
        if (particles.isEmpty()) {
            return;
        }

        double weightSum = 0.0;
        for (Particle particle : particles) {
            double sanitizedWeight = sanitizeWeight(particle.getWeight());
            particle.setWeight(sanitizedWeight);
            weightSum += sanitizedWeight;
        }

        if (weightSum <= 0.0 || Double.isNaN(weightSum) || Double.isInfinite(weightSum)) {
            double equalWeight = 1.0 / particles.size();
            for (Particle particle : particles) {
                particle.setWeight(equalWeight);
            }
            return;
        }

        for (Particle particle : particles) {
            particle.setWeight(particle.getWeight() / weightSum);
        }
    }

    public void resampleIfNeeded() {
        int particleCount = particles.size();
        if (particleCount < 2) {
            return;
        }

        double squaredWeightSum = 0.0;
        for (Particle particle : particles) {
            double weight = particle.getWeight();
            squaredWeightSum += weight * weight;
        }

        if (squaredWeightSum <= 0.0) {
            return;
        }

        double effectiveSampleSize = 1.0 / squaredWeightSum;
        if (effectiveSampleSize >= particleCount * RESAMPLE_THRESHOLD_RATIO) {
            return;
        }

        List<Particle> resampledParticles = new ArrayList<>(particleCount);
        double step = 1.0 / particleCount;
        double target = random.nextDouble() * step;
        double cumulativeWeight = particles.get(0).getWeight();
        int sourceIndex = 0;

        for (int i = 0; i < particleCount; i++) {
            while (target > cumulativeWeight && sourceIndex < particleCount - 1) {
                sourceIndex++;
                cumulativeWeight += particles.get(sourceIndex).getWeight();
            }

            Particle source = particles.get(sourceIndex);
            resampledParticles.add(new Particle(
                    source.getX(),
                    source.getY(),
                    source.getFloor(),
                    step,
                    source.getHeadingRad()
            ));
            target += step;
        }

        particles.clear();
        particles.addAll(resampledParticles);
        normalizeWeights();
    }

    public FusedPose estimatePose() {
        if (particles.isEmpty()) {
            return null;
        }

        double totalWeight = 0.0;
        for (Particle particle : particles) {
            totalWeight += sanitizeWeight(particle.getWeight());
        }
        if (totalWeight <= 0.0 || Double.isNaN(totalWeight) || Double.isInfinite(totalWeight)) {
            return null;
        }

        Map<Integer, Double> floorWeights = new HashMap<>();
        for (Particle particle : particles) {
            double normalizedWeight = sanitizeWeight(particle.getWeight()) / totalWeight;
            floorWeights.merge(particle.getFloor(), normalizedWeight, Double::sum);
        }

        int estimatedFloor = 0;
        double bestFloorWeight = -1.0;
        for (Map.Entry<Integer, Double> entry : floorWeights.entrySet()) {
            if (entry.getValue() > bestFloorWeight) {
                bestFloorWeight = entry.getValue();
                estimatedFloor = entry.getKey();
            }
        }

        double selectedFloorWeight = floorWeights.getOrDefault(estimatedFloor, 0.0);
        boolean useAllParticles = selectedFloorWeight <= 0.0;
        double normalizationBase = useAllParticles ? totalWeight : selectedFloorWeight * totalWeight;

        double meanX = 0.0;
        double meanY = 0.0;
        for (Particle particle : particles) {
            if (!useAllParticles && particle.getFloor() != estimatedFloor) {
                continue;
            }
            double normalizedWeight = sanitizeWeight(particle.getWeight()) / normalizationBase;
            meanX += particle.getX() * normalizedWeight;
            meanY += particle.getY() * normalizedWeight;
        }

        double variance = 0.0;
        for (Particle particle : particles) {
            if (!useAllParticles && particle.getFloor() != estimatedFloor) {
                continue;
            }
            double normalizedWeight = sanitizeWeight(particle.getWeight()) / normalizationBase;
            double dx = particle.getX() - meanX;
            double dy = particle.getY() - meanY;
            variance += normalizedWeight * (dx * dx + dy * dy);
        }

        double spread = Math.sqrt(Math.max(variance, 0.0));
        double confidence = 1.0 / (1.0 + spread);
        long timestamp = lastTimestampMs > 0 ? lastTimestampMs : System.currentTimeMillis();

        Particle legalRepresentative = selectLegalRepresentative(meanX, meanY, estimatedFloor);
        if (legalRepresentative != null) {
            meanX = legalRepresentative.getX();
            meanY = legalRepresentative.getY();
            estimatedFloor = legalRepresentative.getFloor();
        }

        return new FusedPose(meanX, meanY, estimatedFloor, confidence, timestamp);
    }

    int revalidateParticlesAgainstConstraints() {
        if (particles.isEmpty()) {
            return 0;
        }

        List<Particle> legalParticles = new ArrayList<>(particles.size());
        int rejectedParticleCount = 0;
        for (Particle particle : particles) {
            if (spawnValidator.isValid(particle.getX(), particle.getY(), particle.getFloor())) {
                legalParticles.add(particle);
                continue;
            }
            rejectedParticleCount++;
        }
        if (rejectedParticleCount == 0) {
            return 0;
        }

        particles.clear();
        particles.addAll(legalParticles);
        normalizeWeights();
        return rejectedParticleCount;
    }

    void clearParticles(long timestampMs) {
        particles.clear();
        lastTimestampMs = timestampMs;
        resetLastAbsoluteFixOutcomeFlags();
    }

    private void resetLastAbsoluteFixOutcomeFlags() {
        lastAbsoluteFixReanchored = false;
        lastAbsoluteFixRejectedByConstraints = false;
    }

    private double sanitizeAccuracyMeters(double accuracyMeters) {
        if (Double.isNaN(accuracyMeters) || Double.isInfinite(accuracyMeters) || accuracyMeters <= 0.0) {
            return DEFAULT_ABSOLUTE_FIX_STD_M;
        }
        return Math.max(1.0, accuracyMeters);
    }

    private boolean shouldReanchorToAbsoluteFix(double fixX, double fixY, double measurementStdMeters) {
        double supportRadius = Math.max(
                MIN_RECOVERY_DISTANCE_M,
                measurementStdMeters * RECOVERY_DISTANCE_STD_MULTIPLIER
        );
        double supportRadiusSq = supportRadius * supportRadius;
        double supportWeight = 0.0;
        double totalWeight = 0.0;
        for (Particle particle : particles) {
            double weight = sanitizeWeight(particle.getWeight());
            totalWeight += weight;
            double dx = particle.getX() - fixX;
            double dy = particle.getY() - fixY;
            double distanceSq = dx * dx + dy * dy;
            if (distanceSq <= supportRadiusSq) {
                supportWeight += weight;
            }
        }
        if (totalWeight <= 0.0 || Double.isNaN(totalWeight) || Double.isInfinite(totalWeight)) {
            return true;
        }
        // Re-anchor only when too little of the weighted cloud still supports the fix.
        // This prevents one lucky particle from blocking recovery after the cloud drifts away.
        double supportRatio = supportWeight / totalWeight;
        return supportRatio < MIN_SUPPORT_WEIGHT_RATIO;
    }

    private void reanchorToAbsoluteFix(
            double fixX,
            double fixY,
            @Nullable Integer floorPrior,
            long timestampMs,
            double measurementStdMeters,
            boolean useEndpointLockedRecovery
    ) {
        int particleCount = particles.isEmpty() ? DEFAULT_PARTICLE_COUNT : particles.size();
        double headingRad = estimateCircularMeanHeading();
        int reanchorFloor = resolveInitializationFloor(floorPrior);
        List<Particle> reanchoredParticles = useEndpointLockedRecovery
                ? createEndpointLockedRecoveryParticles(fixX, fixY, reanchorFloor, particleCount, headingRad)
                : particleInitializer.initialize(
                        fixX,
                        fixY,
                        reanchorFloor,
                        particleCount,
                        measurementStdMeters,
                        headingRad,
                        spawnValidator
                );
        if (reanchoredParticles.isEmpty()) {
            lastAbsoluteFixReanchored = false;
            lastAbsoluteFixRejectedByConstraints = true;
            return;
        }
        particles.clear();
        particles.addAll(reanchoredParticles);
        lastTimestampMs = timestampMs;
        normalizeWeights();
    }

    private boolean isAbsoluteFixInvalidUnderConstraints(double fixX, double fixY, int validationFloor) {
        return !spawnValidator.isValid(fixX, fixY, validationFloor);
    }

    private void resetParticleAfterInvalidPrediction(
            @NonNull Particle particle,
            double previousX,
            double previousY,
            int previousFloor,
            double previousHeadingRad
    ) {
        particle.setX(previousX);
        particle.setY(previousY);
        particle.setFloor(previousFloor);
        particle.setHeadingRad(previousHeadingRad);
        // Hard wall constraint: once a predicted segment crosses a blocking wall, keep the
        // particle on the last legal pose and effectively kill it for the next normalization.
        particle.setWeight(MIN_WEIGHT);
    }

    private boolean hasMotionCompatibleSupportForAbsoluteFix(
            double fixX,
            double fixY,
            int validationFloor,
            double minimumSupportRatio
    ) {
        double totalWeight = 0.0;
        double motionCompatibleWeight = 0.0;
        for (Particle particle : particles) {
            double weight = sanitizeWeight(particle.getWeight());
            totalWeight += weight;
            if (weight <= 0.0) {
                continue;
            }
            if (isParticleMotionCompatibleWithAbsoluteFix(particle, fixX, fixY, validationFloor)) {
                motionCompatibleWeight += weight;
            }
        }
        if (totalWeight <= 0.0 || Double.isNaN(totalWeight) || Double.isInfinite(totalWeight)) {
            return false;
        }
        return (motionCompatibleWeight / totalWeight) >= minimumSupportRatio;
    }

    private boolean isParticleMotionCompatibleWithAbsoluteFix(
            @NonNull Particle particle,
            double fixX,
            double fixY,
            int validationFloor
    ) {
        return spawnValidator.isValidMotion(
                particle.getX(),
                particle.getY(),
                fixX,
                fixY,
                particle.getFloor(),
                validationFloor
        );
    }

    @Nullable
    private Particle selectLegalRepresentative(double meanX, double meanY, int estimatedFloor) {
        if (spawnValidator.isValid(meanX, meanY, estimatedFloor)) {
            return null;
        }

        Particle bestParticle = null;
        double bestDistanceSq = Double.POSITIVE_INFINITY;
        double bestWeight = -1.0;
        for (Particle particle : particles) {
            if (particle.getFloor() != estimatedFloor || !spawnValidator.isValid(
                    particle.getX(),
                    particle.getY(),
                    particle.getFloor()
            )) {
                continue;
            }
            double dx = particle.getX() - meanX;
            double dy = particle.getY() - meanY;
            double distanceSq = dx * dx + dy * dy;
            double weight = sanitizeWeight(particle.getWeight());
            if (distanceSq < bestDistanceSq
                    || (Math.abs(distanceSq - bestDistanceSq) <= 1e-9 && weight > bestWeight)) {
                bestParticle = particle;
                bestDistanceSq = distanceSq;
                bestWeight = weight;
            }
        }
        return bestParticle;
    }

    @Nullable
    private Integer sanitizeAbsoluteFloorPrior(
            double fixX,
            double fixY,
            @Nullable Integer floorPrior,
            boolean allowRecoveryOverride
    ) {
        if (floorPrior == null || particles.isEmpty() || floorTransitionGate == null) {
            return floorPrior;
        }
        int currentFloor = resolveDominantFloor();
        if (floorPrior == currentFloor) {
            return floorPrior;
        }
        FusedPose currentPose = estimatePose();
        if (currentPose == null) {
            return null;
        }
        boolean floorChangeAllowed = floorTransitionGate.allowsFloorChange(
                currentPose.getX(),
                currentPose.getY(),
                fixX,
                fixY,
                currentFloor,
                floorPrior
        );
        if (floorChangeAllowed) {
            return floorPrior;
        }
        if (allowRecoveryOverride
                && !spawnValidator.isValid(fixX, fixY, currentFloor)
                && spawnValidator.isValid(fixX, fixY, floorPrior)) {
            return floorPrior;
        }
        return null;
    }

    private int resolveAbsoluteFixValidationFloor(
            double fixX,
            double fixY,
            @Nullable Integer acceptedFloorPrior,
            @Nullable Integer requestedFloorPrior
    ) {
        if (acceptedFloorPrior != null) {
            return acceptedFloorPrior;
        }
        int dominantFloor = resolveDominantFloor();
        if (spawnValidator.isValid(fixX, fixY, dominantFloor)) {
            return dominantFloor;
        }
        if (requestedFloorPrior != null && spawnValidator.isValid(fixX, fixY, requestedFloorPrior)) {
            return requestedFloorPrior;
        }
        return dominantFloor;
    }

    private boolean isCredibleRecoveryReanchor(
            double fixX,
            double fixY,
            int validationFloor,
            double measurementStdMeters
    ) {
        if (measurementStdMeters > MAX_CREDIBLE_RECOVERY_ACCURACY_M) {
            return false;
        }
        if (!spawnValidator.isValid(fixX, fixY, validationFloor)) {
            return false;
        }
        List<Particle> recoveryParticles = createEndpointLockedRecoveryParticles(
                fixX,
                fixY,
                validationFloor,
                Math.min(Math.max(particles.size(), 1), 8),
                estimateCircularMeanHeading()
        );
        return !recoveryParticles.isEmpty();
    }

    private List<Particle> createEndpointLockedRecoveryParticles(
            double fixX,
            double fixY,
            int floor,
            int particleCount,
            double headingRad
    ) {
        if (particleCount <= 0 || !spawnValidator.isValid(fixX, fixY, floor)) {
            return Collections.emptyList();
        }
        double initialWeight = 1.0 / particleCount;
        List<Particle> recoveryParticles = new ArrayList<>(particleCount);
        for (int i = 0; i < particleCount; i++) {
            recoveryParticles.add(new Particle(fixX, fixY, floor, initialWeight, headingRad));
        }
        return recoveryParticles;
    }

    private double estimateCircularMeanHeading() {
        double sinSum = 0.0;
        double cosSum = 0.0;
        double weightSum = 0.0;
        for (Particle particle : particles) {
            double weight = sanitizeWeight(particle.getWeight());
            if (weight <= 0.0) {
                continue;
            }
            sinSum += Math.sin(particle.getHeadingRad()) * weight;
            cosSum += Math.cos(particle.getHeadingRad()) * weight;
            weightSum += weight;
        }
        if (weightSum <= 0.0 || (Math.abs(sinSum) < 1e-9 && Math.abs(cosSum) < 1e-9)) {
            return 0.0;
        }
        return normalizeHeading(Math.atan2(sinSum, cosSum));
    }

    private double sanitizeWeight(double weight) {
        if (Double.isNaN(weight) || Double.isInfinite(weight) || weight < 0.0) {
            return 0.0;
        }
        return weight;
    }

    private int resolvePredictedFloor(int previousFloor, int externalFloor) {
        // Modified: the external floor already comes from smoothed barometer/PDR state, so do not
        // require an impossible single-step vertical jump before allowing a floor transition.
        if (externalFloor == previousFloor) {
            return previousFloor;
        }
        return externalFloor;
    }

    private int resolveInitializationFloor(@Nullable Integer floorPrior) {
        if (floorPrior != null) {
            return floorPrior;
        }
        return resolveDominantFloor();
    }

    private int resolveDominantFloor() {
        if (particles.isEmpty()) {
            return 0;
        }

        Map<Integer, Double> floorWeights = new HashMap<>();
        for (Particle particle : particles) {
            floorWeights.merge(particle.getFloor(), sanitizeWeight(particle.getWeight()), Double::sum);
        }

        int dominantFloor = particles.get(0).getFloor();
        double bestWeight = -1.0;
        for (Map.Entry<Integer, Double> entry : floorWeights.entrySet()) {
            if (entry.getValue() > bestWeight) {
                bestWeight = entry.getValue();
                dominantFloor = entry.getKey();
            }
        }
        return dominantFloor;
    }

    private double getFloorLikelihood(int particleFloor, @Nullable Integer floorPrior) {
        if (floorPrior == null) {
            return 1.0;
        }
        return particleFloor == floorPrior ? 1.0 : FLOOR_MISMATCH_PENALTY;
    }

    private double normalizeHeading(double headingRad) {
        double twoPi = Math.PI * 2.0;
        double normalized = headingRad % twoPi;
        if (normalized < 0.0) {
            normalized += twoPi;
        }
        return normalized;
    }

    void setParticlesForTesting(List<Particle> seededParticles, long timestampMs) {
        particles.clear();
        if (seededParticles != null) {
            for (Particle particle : seededParticles) {
                particles.add(new Particle(particle));
            }
        }
        lastTimestampMs = timestampMs;
        lastAbsoluteFixReanchored = false;
        lastAbsoluteFixRejectedByConstraints = false;
    }

    List<Particle> snapshotParticlesForTesting() {
        if (particles.isEmpty()) {
            return Collections.emptyList();
        }
        List<Particle> snapshot = new ArrayList<>(particles.size());
        for (Particle particle : particles) {
            snapshot.add(new Particle(particle));
        }
        return snapshot;
    }

    @NonNull
    StateSnapshot captureStateSnapshot() {
        return new StateSnapshot(
                particles,
                lastTimestampMs,
                lastPredictWallRejectCount,
                lastPredictFloorConstraintRejectCount,
                lastAbsoluteFixReanchored,
                lastAbsoluteFixRejectedByConstraints
        );
    }

    void restoreStateSnapshot(@NonNull StateSnapshot snapshot) {
        particles.clear();
        particles.addAll(snapshot.particles);
        lastTimestampMs = snapshot.lastTimestampMs;
        lastPredictWallRejectCount = snapshot.lastPredictWallRejectCount;
        lastPredictFloorConstraintRejectCount = snapshot.lastPredictFloorConstraintRejectCount;
        lastAbsoluteFixReanchored = snapshot.lastAbsoluteFixReanchored;
        lastAbsoluteFixRejectedByConstraints = snapshot.lastAbsoluteFixRejectedByConstraints;
    }

    boolean hasParticles() {
        return !particles.isEmpty();
    }

    boolean canForceFloor(int floor) {
        if (particles.isEmpty()) {
            return false;
        }
        for (Particle particle : particles) {
            if (!spawnValidator.isValid(particle.getX(), particle.getY(), floor)) {
                return false;
            }
        }
        return true;
    }

    int getLastPredictWallRejectCount() {
        return lastPredictWallRejectCount;
    }

    int getLastPredictFloorConstraintRejectCount() {
        return lastPredictFloorConstraintRejectCount;
    }

    boolean wasLastAbsoluteFixReanchored() {
        return lastAbsoluteFixReanchored;
    }

    boolean wasLastAbsoluteFixRejectedByConstraints() {
        return lastAbsoluteFixRejectedByConstraints;
    }
}
