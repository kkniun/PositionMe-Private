package com.openpositioning.PositionMe.sensors;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

/**
 * Minimal runnable particle-filter skeleton.
 */
public class ParticleFilterEngine {

    private static final int DEFAULT_PARTICLE_COUNT = 100;
    private static final double DEFAULT_INITIAL_STD_M = 2.0;
    private static final double DEFAULT_PREDICTION_NOISE_STD_M = 0.35;
    private static final double DEFAULT_ABSOLUTE_FIX_STD_M = 4.0;
    private static final double FLOOR_MISMATCH_PENALTY = 0.2;
    private static final double RESAMPLE_THRESHOLD_RATIO = 0.5;
    private static final double MIN_WEIGHT = 1e-12;

    private final ParticleInitializer particleInitializer;
    private final ParticleInitializer.SpawnValidator spawnValidator;
    private final Random random;
    private final List<Particle> particles = new ArrayList<>();

    private long lastTimestampMs;

    public ParticleFilterEngine() {
        this(new ParticleInitializer(), ParticleInitializer.allowAll(), new Random());
    }

    public ParticleFilterEngine(
            ParticleInitializer particleInitializer,
            ParticleInitializer.SpawnValidator spawnValidator
    ) {
        this(particleInitializer, spawnValidator, new Random());
    }

    ParticleFilterEngine(
            ParticleInitializer particleInitializer,
            ParticleInitializer.SpawnValidator spawnValidator,
            Random random
    ) {
        this.particleInitializer = particleInitializer;
        this.spawnValidator = spawnValidator == null
                ? ParticleInitializer.allowAll()
                : spawnValidator;
        this.random = random;
    }

    public void initialize(double fixX, double fixY, int floor, long timestampMs) {
        initialize(fixX, fixY, floor, timestampMs, DEFAULT_PARTICLE_COUNT);
    }

    public void initialize(double fixX, double fixY, int floor, long timestampMs, int particleCount) {
        particles.clear();
        particles.addAll(particleInitializer.initialize(
                fixX,
                fixY,
                floor,
                particleCount,
                DEFAULT_INITIAL_STD_M,
                spawnValidator
        ));
        lastTimestampMs = timestampMs;
        normalizeWeights();
    }

    public void predict(double deltaX, double deltaY, int floor, long timestampMs) {
        if (particles.isEmpty()) {
            return;
        }

        for (Particle particle : particles) {
            double previousX = particle.getX();
            double previousY = particle.getY();
            int previousFloor = particle.getFloor();

            double predictedX = previousX + deltaX + random.nextGaussian() * DEFAULT_PREDICTION_NOISE_STD_M;
            double predictedY = previousY + deltaY + random.nextGaussian() * DEFAULT_PREDICTION_NOISE_STD_M;
            int predictedFloor = floor;

            // 这里先保留最小运动模型。
            // 后续应在此接入墙体约束、楼梯/电梯约束和 map matching。
            if (!spawnValidator.isValid(predictedX, predictedY, predictedFloor)) {
                predictedX = previousX;
                predictedY = previousY;
                predictedFloor = previousFloor;
            }

            particle.setX(predictedX);
            particle.setY(predictedY);
            particle.setFloor(predictedFloor);
        }

        lastTimestampMs = timestampMs;
    }

    public void updateWithAbsoluteFix(double fixX, double fixY, int floor, long timestampMs) {
        if (particles.isEmpty()) {
            initialize(fixX, fixY, floor, timestampMs);
            return;
        }

        double variance = DEFAULT_ABSOLUTE_FIX_STD_M * DEFAULT_ABSOLUTE_FIX_STD_M;
        for (Particle particle : particles) {
            double dx = particle.getX() - fixX;
            double dy = particle.getY() - fixY;
            double distanceSq = dx * dx + dy * dy;
            double spatialLikelihood = Math.exp(-0.5 * distanceSq / variance);
            double floorLikelihood = particle.getFloor() == floor ? 1.0 : FLOOR_MISMATCH_PENALTY;

            // 这里先做最小 absolute fix 更新。
            // 后续可在此叠加 WiFi/GNSS 质量、楼层先验和地图约束权重。
            double updatedWeight = particle.getWeight() * spatialLikelihood * floorLikelihood;
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
            weightSum += particle.getWeight();
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
            resampledParticles.add(new Particle(source.getX(), source.getY(), source.getFloor(), step));
            target += step;
        }

        particles.clear();
        particles.addAll(resampledParticles);
    }

    public FusedPose estimatePose() {
        if (particles.isEmpty()) {
            return null;
        }

        double totalWeight = 0.0;
        for (Particle particle : particles) {
            totalWeight += particle.getWeight();
        }
        if (totalWeight <= 0.0) {
            return null;
        }

        double meanX = 0.0;
        double meanY = 0.0;
        Map<Integer, Double> floorWeights = new HashMap<>();
        for (Particle particle : particles) {
            double normalizedWeight = particle.getWeight() / totalWeight;
            meanX += particle.getX() * normalizedWeight;
            meanY += particle.getY() * normalizedWeight;
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

        double variance = 0.0;
        for (Particle particle : particles) {
            double normalizedWeight = particle.getWeight() / totalWeight;
            double dx = particle.getX() - meanX;
            double dy = particle.getY() - meanY;
            variance += normalizedWeight * (dx * dx + dy * dy);
        }

        double spread = Math.sqrt(Math.max(variance, 0.0));
        double confidence = 1.0 / (1.0 + spread);
        long timestamp = lastTimestampMs > 0 ? lastTimestampMs : System.currentTimeMillis();

        return new FusedPose(meanX, meanY, estimatedFloor, confidence, timestamp);
    }
}
