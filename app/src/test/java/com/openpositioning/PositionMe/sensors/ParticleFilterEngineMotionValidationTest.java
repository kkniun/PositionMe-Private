package com.openpositioning.PositionMe.sensors;

import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

public class ParticleFilterEngineMotionValidationTest {

    @Test
    public void predictUsesMotionValidatorNotOnlyEndpointValidator() {
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
                return false;
            }
        };

        ParticleFilterEngine engine = new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                validator,
                new ZeroRandom()
        );

        engine.setParticlesForTesting(java.util.List.of(
                new Particle(0.0, 0.0, 0, 1.0, 0.0)
        ), 1000L);
        engine.predict(new PdrDelta(1.0f, 0.0f, 0.0f), 0, 1100L);

        FusedPose pose = engine.estimatePose();
        assertEquals(0.0, pose.getX(), 1e-6);
        assertEquals(0.0, pose.getY(), 1e-6);
        assertEquals(0, pose.getFloor());
    }

    @Test
    public void restoreStateSnapshotRestoresTimestampAndAbsoluteFixFlags() {
        ParticleFilterEngine engine = new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                ParticleInitializer.allowAll(),
                new ZeroRandom()
        );

        engine.setParticlesForTesting(java.util.List.of(
                new Particle(0.0, 0.0, 0, 1.0, 0.0)
        ), 1_000L);
        ParticleFilterEngine.StateSnapshot snapshot = engine.captureStateSnapshot();

        engine.updateWithAbsoluteFix(30.0, 0.0, 0, 1_100L, 4.0);
        assertEquals(1_100L, engine.estimatePose().getTimestampMs());
        assertEquals(true, engine.wasLastAbsoluteFixReanchored());

        engine.restoreStateSnapshot(snapshot);

        FusedPose restoredPose = engine.estimatePose();
        assertEquals(0.0, restoredPose.getX(), 1e-6);
        assertEquals(0.0, restoredPose.getY(), 1e-6);
        assertEquals(1_000L, restoredPose.getTimestampMs());
        assertFalse(engine.wasLastAbsoluteFixReanchored());
        assertFalse(engine.wasLastAbsoluteFixRejectedByConstraints());
    }

    @Test
    public void initializeClearsStaleAbsoluteFixOutcomeFlags() {
        ParticleInitializer.SpawnValidator validator = new ParticleInitializer.SpawnValidator() {
            @Override
            public boolean isValid(double x, double y, int floor) {
                return x < 10.0;
            }
        };
        ParticleFilterEngine engine = new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                validator,
                new ZeroRandom()
        );

        engine.setParticlesForTesting(java.util.List.of(
                new Particle(0.0, 0.0, 0, 1.0, 0.0)
        ), 1_000L);
        engine.updateWithAbsoluteFix(30.0, 0.0, 0, 1_100L, 4.0);
        assertFalse(engine.wasLastAbsoluteFixReanchored());
        assertEquals(true, engine.wasLastAbsoluteFixRejectedByConstraints());

        engine.initialize(5.0, 0.0, 0, 1_200L, 0.0, 16, 0.0);

        FusedPose pose = engine.estimatePose();
        assertEquals(5.0, pose.getX(), 1e-6);
        assertEquals(0.0, pose.getY(), 1e-6);
        assertEquals(1_200L, pose.getTimestampMs());
        assertFalse(engine.wasLastAbsoluteFixReanchored());
        assertFalse(engine.wasLastAbsoluteFixRejectedByConstraints());
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
