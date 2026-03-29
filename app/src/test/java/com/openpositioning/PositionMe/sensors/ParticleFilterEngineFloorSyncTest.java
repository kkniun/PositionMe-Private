package com.openpositioning.PositionMe.sensors;

import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class ParticleFilterEngineFloorSyncTest {

    @Test
    public void forceFloorUpdatesEstimatedPoseFloorWithoutNewStep() {
        ParticleFilterEngine engine = new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                ParticleInitializer.allowAll(),
                new ZeroRandom()
        );

        engine.initialize(10.0, 5.0, 1, 1_000L, 0.0, 32, 0.0);
        engine.forceFloor(4, 2_000L);

        FusedPose pose = engine.estimatePose();
        assertNotNull(pose);
        assertEquals(4, pose.getFloor());
        assertEquals(10.0, pose.getX(), 1e-6);
        assertEquals(5.0, pose.getY(), 1e-6);
    }

    @Test
    public void forceFloorRejectsIllegalTargetFloorAndKeepsExistingState() {
        ParticleInitializer.SpawnValidator validator = new ParticleInitializer.SpawnValidator() {
            @Override
            public boolean isValid(double x, double y, int floor) {
                return floor != 4;
            }
        };
        ParticleFilterEngine engine = new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                validator,
                new ZeroRandom()
        );

        engine.initialize(10.0, 5.0, 1, 1_000L, 0.0, 32, 0.0);

        assertFalse(engine.forceFloor(4, 2_000L));
        assertTrue(engine.canForceFloor(1));

        FusedPose pose = engine.estimatePose();
        assertNotNull(pose);
        assertEquals(1, pose.getFloor());
        assertEquals(10.0, pose.getX(), 1e-6);
        assertEquals(5.0, pose.getY(), 1e-6);
    }

    @Test
    public void restoreStateSnapshotDoesNotBypassIllegalForceFloorChecks() {
        ParticleInitializer.SpawnValidator validator = new ParticleInitializer.SpawnValidator() {
            @Override
            public boolean isValid(double x, double y, int floor) {
                return floor != 4;
            }
        };
        ParticleFilterEngine engine = new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                validator,
                new ZeroRandom()
        );

        engine.initialize(10.0, 5.0, 1, 1_000L, 0.0, 32, 0.0);
        ParticleFilterEngine.StateSnapshot snapshot = engine.captureStateSnapshot();

        engine.clearParticles(2_000L);
        engine.restoreStateSnapshot(snapshot);

        assertFalse(engine.forceFloor(4, 3_000L));
        FusedPose pose = engine.estimatePose();
        assertNotNull(pose);
        assertEquals(1, pose.getFloor());
        assertEquals(10.0, pose.getX(), 1e-6);
        assertEquals(5.0, pose.getY(), 1e-6);
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
