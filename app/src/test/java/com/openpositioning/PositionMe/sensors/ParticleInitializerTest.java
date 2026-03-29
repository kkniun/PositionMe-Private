package com.openpositioning.PositionMe.sensors;

import org.junit.Test;

import java.util.List;
import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class ParticleInitializerTest {

    @Test
    public void initializeReturnsEmptyWhenNoValidSampleCanBeGenerated() {
        ParticleInitializer initializer = new ParticleInitializer(new SequenceRandom());

        List<Particle> particles = initializer.initialize(
                0.0,
                0.0,
                0,
                4,
                1.0,
                0.0,
                (x, y, floor) -> false
        );

        assertTrue(particles.isEmpty());
    }

    @Test
    public void initializeReusesAcceptedValidSamplesInsteadOfFallingBackToInvalidFix() {
        ParticleInitializer initializer = new ParticleInitializer(new SequenceRandom(
                1.0, 0.0,
                -1.0, 0.0,
                -1.0, 0.0,
                -1.0, 0.0,
                -1.0, 0.0,
                -1.0, 0.0,
                -1.0, 0.0,
                -1.0, 0.0,
                -1.0, 0.0,
                -1.0, 0.0,
                -1.0, 0.0,
                -1.0, 0.0,
                -1.0, 0.0,
                -1.0, 0.0,
                -1.0, 0.0,
                -1.0, 0.0,
                -1.0, 0.0
        ));

        List<Particle> particles = initializer.initialize(
                0.0,
                0.0,
                0,
                2,
                1.0,
                0.0,
                (x, y, floor) -> x >= 0.5
        );

        assertEquals(2, particles.size());
        for (Particle particle : particles) {
            assertTrue(particle.getX() >= 0.5);
            assertEquals(0.0, particle.getY(), 1e-6);
        }
        assertEquals(particles.get(0).getX(), particles.get(1).getX(), 1e-6);
    }

    @Test
    public void initializationRejectsSampleWhenFixToSampleSegmentCrossesWall() {
        ParticleInitializer initializer = new ParticleInitializer(new SequenceRandom(
                1.0, 0.0,
                1.0, 0.0,
                1.0, 0.0,
                1.0, 0.0,
                1.0, 0.0,
                1.0, 0.0,
                1.0, 0.0,
                1.0, 0.0
        ));

        List<Particle> particles = initializer.initialize(
                0.0,
                0.0,
                0,
                1,
                1.0,
                0.0,
                new ParticleInitializer.SpawnValidator() {
                    @Override
                    public boolean isValid(double x, double y, int floor) {
                        return x >= 0.5;
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
                        return predictedX <= 0.0;
                    }
                }
        );

        assertTrue(particles.isEmpty());
    }

    private static final class SequenceRandom extends Random {
        private final double[] sequence;
        private int index;

        SequenceRandom(double... sequence) {
            this.sequence = sequence == null ? new double[0] : sequence;
        }

        @Override
        public double nextGaussian() {
            if (index < sequence.length) {
                return sequence[index++];
            }
            return -1.0;
        }

        @Override
        public int nextInt(int bound) {
            return 0;
        }
    }
}
