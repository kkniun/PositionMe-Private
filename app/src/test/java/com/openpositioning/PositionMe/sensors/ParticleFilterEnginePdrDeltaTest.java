package com.openpositioning.PositionMe.sensors;

import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;

public class ParticleFilterEnginePdrDeltaTest {

    @Test
    public void predictWithZeroHeadingMovesNorthInLocalFrame() {
        ParticleFilterEngine engine = new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                ParticleInitializer.allowAll(),
                new ZeroRandom()
        );

        engine.initialize(0.0, 0.0, 0, 1000L, 0.0, 1, 0.0);
        engine.predict(new PdrDelta(1.0f, 0.0f, 0.0f), 0, 1100L);

        FusedPose pose = engine.estimatePose();
        assertEquals(0.0, pose.getX(), 1e-6);
        assertEquals(1.0, pose.getY(), 1e-6);
    }

    @Test
    public void predictQuarterTurnFromNorthMovesEastInLocalFrame() {
        ParticleFilterEngine engine = new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                ParticleInitializer.allowAll(),
                new ZeroRandom()
        );

        engine.initialize(0.0, 0.0, 0, 1000L, 0.0, 1, 0.0);
        engine.predict(new PdrDelta(1.0f, (float) (Math.PI / 2.0), 0.0f), 0, 1100L);

        FusedPose pose = engine.estimatePose();
        assertEquals(1.0, pose.getX(), 1e-6);
        assertEquals(0.0, pose.getY(), 1e-6);
    }

    @Test
    public void predictKeepsExistingFloorWhenVerticalEvidenceIsWeak() {
        ParticleFilterEngine engine = new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                ParticleInitializer.allowAll(),
                new ZeroRandom()
        );

        engine.initialize(0.0, 0.0, 0, 1000L, 0.0, 1, 0.0);
        engine.predict(new PdrDelta(1.0f, 0.0f, 0.2f), 1, 1100L);

        FusedPose pose = engine.estimatePose();
        assertEquals(0, pose.getFloor());
        assertEquals(0.0, pose.getX(), 1e-6);
        assertEquals(1.0, pose.getY(), 1e-6);
    }

    @Test
    public void predictAllowsFloorChangeWhenVerticalEvidenceIsStrong() {
        ParticleFilterEngine engine = new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                ParticleInitializer.allowAll(),
                new ZeroRandom()
        );

        engine.initialize(0.0, 0.0, 0, 1000L, 0.0, 1, 0.0);
        engine.predict(new PdrDelta(1.0f, 0.0f, 2.0f), 1, 1100L);

        FusedPose pose = engine.estimatePose();
        assertEquals(1, pose.getFloor());
        assertEquals(0.0, pose.getX(), 1e-6);
        assertEquals(1.0, pose.getY(), 1e-6);
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
