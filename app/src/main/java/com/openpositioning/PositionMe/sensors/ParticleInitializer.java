package com.openpositioning.PositionMe.sensors;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Creates an initial particle cloud around the first absolute fix.
 */
public class ParticleInitializer {

    private static final double DEFAULT_POSITION_STD_M = 2.0;
    private static final int MAX_ATTEMPTS_PER_PARTICLE = 16;
    private static final SpawnValidator ALLOW_ALL_VALIDATOR = (x, y, floor) -> true;

    private final Random random;

    public interface SpawnValidator {
        boolean isValid(double x, double y, int floor);

        default boolean isValidMotion(
                double previousX,
                double previousY,
                double predictedX,
                double predictedY,
                int previousFloor,
                int predictedFloor
        ) {
            return isValid(predictedX, predictedY, predictedFloor);
        }

        default boolean isValidSpawnFromFix(
                double fixX,
                double fixY,
                double sampleX,
                double sampleY,
                int floor
        ) {
            return isValid(sampleX, sampleY, floor)
                    && isValidMotion(fixX, fixY, sampleX, sampleY, floor, floor);
        }
    }

    public ParticleInitializer() {
        this(new Random());
    }

    ParticleInitializer(Random random) {
        this.random = random;
    }

    public static SpawnValidator allowAll() {
        return ALLOW_ALL_VALIDATOR;
    }

    public List<Particle> initialize(double fixX, double fixY, int floor, int particleCount) {
        return initialize(fixX, fixY, floor, particleCount, DEFAULT_POSITION_STD_M, 0.0, allowAll());
    }

    public List<Particle> initialize(
            double fixX,
            double fixY,
            int floor,
            int particleCount,
            double positionStdMeters,
            SpawnValidator validator
    ) {
        return initialize(fixX, fixY, floor, particleCount, positionStdMeters, 0.0, validator);
    }

    public List<Particle> initialize(
            double fixX,
            double fixY,
            int floor,
            int particleCount,
            double positionStdMeters,
            double headingRad,
            SpawnValidator validator
    ) {
        List<Particle> particles = new ArrayList<>();
        if (particleCount <= 0) {
            return particles;
        }

        SpawnValidator effectiveValidator = validator == null ? allowAll() : validator;
        double safeStd = positionStdMeters > 0 ? positionStdMeters : DEFAULT_POSITION_STD_M;
        double initialWeight = 1.0 / particleCount;
        List<double[]> acceptedSamples = new ArrayList<>(particleCount);

        for (int i = 0; i < particleCount; i++) {
            double[] acceptedSample = findValidSample(
                    fixX,
                    fixY,
                    floor,
                    safeStd,
                    effectiveValidator,
                    acceptedSamples.isEmpty() ? MAX_ATTEMPTS_PER_PARTICLE * 4 : MAX_ATTEMPTS_PER_PARTICLE
            );
            if (acceptedSample == null) {
                if (acceptedSamples.isEmpty()) {
                    // When constraints are active, do not silently seed the cloud into illegal
                    // geometry. Let the caller reject/defer the absolute fix instead.
                    return new ArrayList<>();
                }
                acceptedSample = acceptedSamples.get(random.nextInt(acceptedSamples.size()));
            } else {
                acceptedSamples.add(acceptedSample);
            }

            particles.add(new Particle(
                    acceptedSample[0],
                    acceptedSample[1],
                    floor,
                    initialWeight,
                    headingRad
            ));
        }

        return particles;
    }

    private double[] findValidSample(
            double fixX,
            double fixY,
            int floor,
            double positionStdMeters,
            SpawnValidator validator,
            int maxAttempts
    ) {
        for (int attempt = 0; attempt < maxAttempts; attempt++) {
            double sampleX = fixX + random.nextGaussian() * positionStdMeters;
            double sampleY = fixY + random.nextGaussian() * positionStdMeters;
            if (validator.isValidSpawnFromFix(fixX, fixY, sampleX, sampleY, floor)) {
                return new double[]{sampleX, sampleY};
            }
        }
        return null;
    }
}
