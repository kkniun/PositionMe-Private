package com.openpositioning.PositionMe.sensors;

import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;

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

        engine.initialize(0.0, 0.0, 0, 1000L, 0.0, 1, 0.0);
        engine.predict(new PdrDelta(1.0f, 0.0f, 0.0f), 0, 1100L);

        FusedPose pose = engine.estimatePose();
        assertEquals(0.0, pose.getX(), 1e-6);
        assertEquals(0.0, pose.getY(), 1e-6);
        assertEquals(0, pose.getFloor());
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
