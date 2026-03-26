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
    private static final double SEARCH_RADIUS_STEP_M = 0.6;
    private static final int SEARCH_RADIUS_STEPS = 24;
    private static final int SEARCH_DIRECTIONS = 16;
    private static final SpawnValidator ALLOW_ALL_VALIDATOR = (x, y, floor) -> true;

    private final Random random;

    public interface SpawnValidator {
        boolean isValid(double x, double y, int floor);
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

        for (int i = 0; i < particleCount; i++) {
            double sampleX = fixX;
            double sampleY = fixY;
            boolean accepted = false;

            // 当前仓库还没有轻量 BuildingMap 接口，这里先预留可注入校验回调。
            // 后续接 map matching / 楼层约束时，可在这里替换为更严格的合法性判定。
            for (int attempt = 0; attempt < MAX_ATTEMPTS_PER_PARTICLE; attempt++) {
                sampleX = fixX + random.nextGaussian() * safeStd;
                sampleY = fixY + random.nextGaussian() * safeStd;
                if (effectiveValidator.isValid(sampleX, sampleY, floor)) {
                    accepted = true;
                    break;
                }
            }

            if (!accepted) {
                double[] nearestValid = findNearestValidSample(fixX, fixY, floor, effectiveValidator);
                if (nearestValid != null) {
                    sampleX = nearestValid[0];
                    sampleY = nearestValid[1];
                    accepted = true;
                }
            }

            if (!accepted) {
                if (!particles.isEmpty()) {
                    Particle anchor = particles.get(random.nextInt(particles.size()));
                    sampleX = anchor.getX();
                    sampleY = anchor.getY();
                } else if (effectiveValidator.isValid(fixX, fixY, floor)) {
                    sampleX = fixX;
                    sampleY = fixY;
                } else {
                    double[] nearestValid = findNearestValidSample(fixX, fixY, floor, effectiveValidator);
                    if (nearestValid != null) {
                        sampleX = nearestValid[0];
                        sampleY = nearestValid[1];
                    } else {
                        // Last-resort fallback only when no valid sample could be found.
                        sampleX = fixX;
                        sampleY = fixY;
                    }
                }
            }

            particles.add(new Particle(sampleX, sampleY, floor, initialWeight, headingRad));
        }

        return particles;
    }

    private double[] findNearestValidSample(
            double centerX,
            double centerY,
            int floor,
            SpawnValidator validator
    ) {
        if (validator == null) {
            return null;
        }
        if (validator.isValid(centerX, centerY, floor)) {
            return new double[]{centerX, centerY};
        }
        for (int radiusStep = 1; radiusStep <= SEARCH_RADIUS_STEPS; radiusStep++) {
            double radius = radiusStep * SEARCH_RADIUS_STEP_M;
            for (int direction = 0; direction < SEARCH_DIRECTIONS; direction++) {
                double angle = (Math.PI * 2.0 * direction) / SEARCH_DIRECTIONS;
                double candidateX = centerX + (Math.cos(angle) * radius);
                double candidateY = centerY + (Math.sin(angle) * radius);
                if (validator.isValid(candidateX, candidateY, floor)) {
                    return new double[]{candidateX, candidateY};
                }
            }
        }
        return null;
    }
}
