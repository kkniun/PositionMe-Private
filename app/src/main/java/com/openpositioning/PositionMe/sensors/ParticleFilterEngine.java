package com.openpositioning.PositionMe.sensors;

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
    private static final double MIN_HEIGHT_DELTA_FOR_FLOOR_CHANGE_M = 1.5;

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
        double heightDeltaMeters = delta.getHeightDeltaMeters();
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
            int predictedFloor = resolvePredictedFloor(previousFloor, floor, heightDeltaMeters);

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
                predictedX = previousX;
                predictedY = previousY;
                predictedFloor = previousFloor;
                predictedHeadingRad = previousHeadingRad;
            }

            particle.setX(predictedX);
            particle.setY(predictedY);
            particle.setFloor(predictedFloor);
            particle.setHeadingRad(predictedHeadingRad);
        }

        lastTimestampMs = timestampMs;
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
        lastAbsoluteFixReanchored = false;
        if (particles.isEmpty()) {
            initialize(
                    fixX,
                    fixY,
                    resolveInitializationFloor(floorPrior),
                    timestampMs,
                    0.0,
                    DEFAULT_PARTICLE_COUNT,
                    sanitizeAccuracyMeters(accuracyMeters)
            );
            return;
        }

        double measurementStdMeters = sanitizeAccuracyMeters(accuracyMeters);
        if (shouldReanchorToAbsoluteFix(fixX, fixY, measurementStdMeters)) {
            lastAbsoluteFixReanchored = true;
            reanchorToAbsoluteFix(fixX, fixY, floorPrior, timestampMs, measurementStdMeters);
            return;
        }

        double variance = measurementStdMeters * measurementStdMeters;
        for (Particle particle : particles) {
            double dx = particle.getX() - fixX;
            double dy = particle.getY() - fixY;
            double distanceSq = dx * dx + dy * dy;
            double spatialLikelihood = Math.exp(-0.5 * distanceSq / variance);
            double floorLikelihood = getFloorLikelihood(particle.getFloor(), floorPrior);
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

        return new FusedPose(meanX, meanY, estimatedFloor, confidence, timestamp);
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
            double measurementStdMeters
    ) {
        int particleCount = particles.isEmpty() ? DEFAULT_PARTICLE_COUNT : particles.size();
        double headingRad = estimateCircularMeanHeading();
        int reanchorFloor = resolveInitializationFloor(floorPrior);
        particles.clear();
        particles.addAll(particleInitializer.initialize(
                fixX,
                fixY,
                reanchorFloor,
                particleCount,
                measurementStdMeters,
                headingRad,
                spawnValidator
        ));
        lastTimestampMs = timestampMs;
        normalizeWeights();
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

    private int resolvePredictedFloor(int previousFloor, int externalFloor, double heightDeltaMeters) {
        // Step 1 keeps barometer/PDR floor as a conservative prior. A tiny vertical delta should
        // not snap the whole cloud onto a new floor during normal planar tracking.
        if (externalFloor == previousFloor) {
            return previousFloor;
        }
        if (Math.abs(heightDeltaMeters) < MIN_HEIGHT_DELTA_FOR_FLOOR_CHANGE_M) {
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

    int getLastPredictWallRejectCount() {
        return lastPredictWallRejectCount;
    }

    int getLastPredictFloorConstraintRejectCount() {
        return lastPredictFloorConstraintRejectCount;
    }

    boolean wasLastAbsoluteFixReanchored() {
        return lastAbsoluteFixReanchored;
    }
}
