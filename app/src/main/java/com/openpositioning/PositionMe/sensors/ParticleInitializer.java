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
        return initialize(fixX, fixY, floor, particleCount, DEFAULT_POSITION_STD_M, allowAll());
    }

    public List<Particle> initialize(
            double fixX,
            double fixY,
            int floor,
            int particleCount,
            double positionStdMeters,
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
                sampleX = fixX;
                sampleY = fixY;
            }

            particles.add(new Particle(sampleX, sampleY, floor, initialWeight));
        }

        return particles;
    }
}
