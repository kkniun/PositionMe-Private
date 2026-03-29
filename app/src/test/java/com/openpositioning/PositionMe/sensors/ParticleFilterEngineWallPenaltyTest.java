package com.openpositioning.PositionMe.sensors;

import org.junit.Test;

import java.util.List;
import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class ParticleFilterEngineWallPenaltyTest {

    @Test
    public void illegalWallCrossingMotionGetsHardRejectedAndCannotCrossWall() {
        ParticleInitializer.SpawnValidator validator = new ParticleInitializer.SpawnValidator() {
            @Override
            public boolean isValid(double x, double y, int floor) {
                return true;
            }

            @Override
            public boolean isValidMotion(
                    double previousX,
                    double previousY,
                    double predictedX,
                    double predictedY,
                    int previousFloor,
                    int predictedFloor
            ) {
                return previousX >= 1.5 || predictedX <= 0.5;
            }
        };

        ParticleFilterEngine engine = new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                validator,
                new ZeroRandom()
        );

        engine.setParticlesForTesting(List.of(
                new Particle(0.0, 0.0, 0, 0.5, Math.PI / 2.0),
                new Particle(2.0, 0.0, 0, 0.5, Math.PI / 2.0)
        ), 1_000L);

        engine.predict(new PdrDelta(1.0f, 0.0f, 0.0f), 0, 1_100L);

        List<Particle> particles = engine.snapshotParticlesForTesting();
        assertEquals(2, particles.size());
        assertEquals(0.0, particles.get(0).getX(), 1e-6);
        assertTrue(particles.get(0).getWeight() < 1e-9);
        assertEquals(3.0, particles.get(1).getX(), 1e-6);
        assertTrue(particles.get(1).getWeight() > 0.999999);
        assertEquals(3.0, engine.estimatePose().getX(), 1e-6);
    }

    @Test
    public void restoreStateSnapshotPreservesWallValidatedPredictBehavior() {
        ParticleInitializer.SpawnValidator validator = new ParticleInitializer.SpawnValidator() {
            @Override
            public boolean isValid(double x, double y, int floor) {
                return true;
            }

            @Override
            public boolean isValidMotion(
                    double previousX,
                    double previousY,
                    double predictedX,
                    double predictedY,
                    int previousFloor,
                    int predictedFloor
            ) {
                return previousX >= 1.5 || predictedX <= 0.5;
            }
        };

        ParticleFilterEngine engine = new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                validator,
                new ZeroRandom()
        );

        engine.setParticlesForTesting(List.of(
                new Particle(0.0, 0.0, 0, 0.5, Math.PI / 2.0),
                new Particle(2.0, 0.0, 0, 0.5, Math.PI / 2.0)
        ), 1_000L);
        ParticleFilterEngine.StateSnapshot snapshot = engine.captureStateSnapshot();

        engine.clearParticles(2_000L);
        engine.restoreStateSnapshot(snapshot);
        engine.predict(new PdrDelta(1.0f, 0.0f, 0.0f), 0, 1_100L);

        List<Particle> particles = engine.snapshotParticlesForTesting();
        assertEquals(2, particles.size());
        assertEquals(0.0, particles.get(0).getX(), 1e-6);
        assertTrue(particles.get(0).getWeight() < 1e-9);
        assertEquals(3.0, particles.get(1).getX(), 1e-6);
        assertTrue(particles.get(1).getWeight() > 0.999999);
    }

    private static final class ZeroRandom extends Random {
        @Override
        public double nextGaussian() {
            return 0.0;
        }

        @Override
        public double nextDouble() {
            return 0.5;
        }
    }
}
