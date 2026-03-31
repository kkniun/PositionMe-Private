package com.openpositioning.PositionMe.sensors;

import com.openpositioning.PositionMe.utils.PdrProcessing;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

public class ParticleFilterEngineUpdateTest {

    @Test
    public void projectStepToLocalFrameUsesNorthReferencedClockwiseHeading() {
        double[] northStep = PdrProcessing.projectStepToLocalFrame(1.0, 0.0);
        double[] eastStep = PdrProcessing.projectStepToLocalFrame(1.0, Math.PI / 2.0);

        assertEquals(0.0, northStep[0], 1e-6);
        assertEquals(1.0, northStep[1], 1e-6);
        assertEquals(1.0, eastStep[0], 1e-6);
        assertEquals(0.0, eastStep[1], 1e-6);
    }

    @Test
    public void updateWithNearbyAbsoluteFixMovesEstimateTowardMeasurement() {
        ParticleFilterEngine engine = createDeterministicEngine();
        engine.setParticlesForTesting(Arrays.asList(
                new Particle(0.0, 0.0, 0, 0.5, 0.0),
                new Particle(10.0, 0.0, 0, 0.5, 0.0)
        ), 1000L);

        engine.updateWithAbsoluteFix(10.0, 0.0, 0, 1100L, 1.0);

        FusedPose pose = engine.estimatePose();
        assertEquals(10.0, pose.getX(), 1e-6);
        assertEquals(0.0, pose.getY(), 1e-6);
    }

    @Test
    public void farAbsoluteFixReanchorsDriftedCloud() {
        ParticleFilterEngine engine = createDeterministicEngine();
        engine.setParticlesForTesting(Arrays.asList(
                new Particle(0.0, 0.0, 0, 0.5, 0.0),
                new Particle(0.0, 0.0, 0, 0.5, 0.0)
        ), 1000L);

        engine.updateWithAbsoluteFix(100.0, 20.0, 1, 1100L, 4.0);

        FusedPose pose = engine.estimatePose();
        assertEquals(100.0, pose.getX(), 1e-6);
        assertEquals(20.0, pose.getY(), 1e-6);
        assertEquals(1, pose.getFloor());

        List<Particle> particles = engine.snapshotParticlesForTesting();
        assertEquals(2, particles.size());
        for (Particle particle : particles) {
            assertEquals(100.0, particle.getX(), 1e-6);
            assertEquals(20.0, particle.getY(), 1e-6);
            assertEquals(1, particle.getFloor());
            assertEquals(0.5, particle.getWeight(), 1e-6);
        }
    }

    @Test
    public void constrainedInvalidAbsoluteFixKeepsExistingCloud() {
        ParticleInitializer.SpawnValidator rejectingValidator = (x, y, floor) -> false;
        ParticleFilterEngine engine = new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                rejectingValidator,
                new ZeroRandom()
        );
        engine.setParticlesForTesting(Arrays.asList(
                new Particle(0.0, 0.0, 0, 0.5, 0.0),
                new Particle(0.0, 0.0, 0, 0.5, 0.0)
        ), 1000L);

        engine.updateWithAbsoluteFix(100.0, 20.0, 0, 1100L, 4.0);

        assertFalse(engine.wasLastAbsoluteFixReanchored());
        assertTrue(engine.wasLastAbsoluteFixRejectedByConstraints());
        List<Particle> particles = engine.snapshotParticlesForTesting();
        assertEquals(2, particles.size());
        for (Particle particle : particles) {
            assertEquals(0.0, particle.getX(), 1e-6);
            assertEquals(0.0, particle.getY(), 1e-6);
            assertEquals(0, particle.getFloor());
            assertEquals(0.5, particle.getWeight(), 1e-6);
        }
    }

    @Test
    public void initialIllegalAbsoluteFixIsRejectedInsteadOfSilentlyReanchoring() {
        ParticleInitializer.SpawnValidator validator = (x, y, floor) -> x < 0.5 || x > 1.0;
        ParticleFilterEngine engine = new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                validator,
                new ZeroRandom()
        );

        engine.updateWithAbsoluteFix(0.75, 0.0, 0, 1100L, 1.0);

        assertTrue(engine.wasLastAbsoluteFixRejectedByConstraints());
        assertTrue(engine.snapshotParticlesForTesting().isEmpty());
        assertNull(engine.estimatePose());
    }

    @Test
    public void motionIncompatibleAbsoluteFixDoesNotReanchorCloud() {
        ParticleInitializer.SpawnValidator rejectingMotionValidator = new ParticleInitializer.SpawnValidator() {
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
                rejectingMotionValidator,
                new ZeroRandom()
        );
        engine.setParticlesForTesting(Arrays.asList(
                new Particle(0.0, 0.0, 0, 0.5, 0.0),
                new Particle(0.0, 0.0, 0, 0.5, 0.0)
        ), 1000L);

        engine.updateWithAbsoluteFix(100.0, 20.0, 0, 1100L, 8.0);

        assertFalse(engine.wasLastAbsoluteFixReanchored());
        assertTrue(engine.wasLastAbsoluteFixRejectedByConstraints());
        assertParticlesStayAtX(engine.snapshotParticlesForTesting(), 0.0);
    }

    @Test
    public void credibleMotionIncompatibleAbsoluteFixCanReanchorDriftedCloud() {
        ParticleInitializer.SpawnValidator rejectingMotionValidator = new ParticleInitializer.SpawnValidator() {
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
                rejectingMotionValidator,
                new ZeroRandom()
        );
        engine.setParticlesForTesting(Arrays.asList(
                new Particle(0.0, 0.0, 0, 0.5, 0.0),
                new Particle(0.0, 0.0, 0, 0.5, 0.0)
        ), 1000L);

        engine.updateWithAbsoluteFix(100.0, 20.0, 0, 1100L, 4.0);

        assertTrue(engine.wasLastAbsoluteFixReanchored());
        FusedPose pose = engine.estimatePose();
        assertEquals(100.0, pose.getX(), 1e-6);
        assertEquals(20.0, pose.getY(), 1e-6);
        assertEquals(0, pose.getFloor());
    }

    @Test
    public void oneNearParticleDoesNotBlockCloudLevelReanchor() {
        ParticleFilterEngine engine = createDeterministicEngine();
        List<Particle> particles = Arrays.asList(
                new Particle(0.0, 0.0, 0, 0.1, 0.0),
                new Particle(100.0, 0.0, 0, 0.1, 0.0),
                new Particle(100.0, 0.0, 0, 0.1, 0.0),
                new Particle(100.0, 0.0, 0, 0.1, 0.0),
                new Particle(100.0, 0.0, 0, 0.1, 0.0),
                new Particle(100.0, 0.0, 0, 0.1, 0.0),
                new Particle(100.0, 0.0, 0, 0.1, 0.0),
                new Particle(100.0, 0.0, 0, 0.1, 0.0),
                new Particle(100.0, 0.0, 0, 0.1, 0.0),
                new Particle(100.0, 0.0, 0, 0.1, 0.0)
        );
        engine.setParticlesForTesting(particles, 1000L);

        engine.updateWithAbsoluteFix(0.0, 0.0, 0, 1100L, 1.0);

        List<Particle> updated = engine.snapshotParticlesForTesting();
        assertEquals(10, updated.size());
        for (Particle particle : updated) {
            assertEquals(0.0, particle.getX(), 1e-6);
            assertEquals(0.0, particle.getY(), 1e-6);
            assertEquals(0.1, particle.getWeight(), 1e-6);
        }
    }

    @Test
    public void reanchorThresholdUsesSupportRadiusBoundary() {
        ParticleFilterEngine insideEightMeterRadius = createDeterministicEngine();
        insideEightMeterRadius.setParticlesForTesting(Arrays.asList(
                new Particle(8.0, 0.0, 0, 0.5, 0.0),
                new Particle(8.0, 0.0, 0, 0.5, 0.0)
        ), 1000L);
        insideEightMeterRadius.updateWithAbsoluteFix(0.0, 0.0, 0, 1100L, 1.0);
        assertParticlesStayAtX(insideEightMeterRadius.snapshotParticlesForTesting(), 8.0);

        ParticleFilterEngine outsideEightMeterRadius = createDeterministicEngine();
        outsideEightMeterRadius.setParticlesForTesting(Arrays.asList(
                new Particle(8.01, 0.0, 0, 0.5, 0.0),
                new Particle(8.01, 0.0, 0, 0.5, 0.0)
        ), 1000L);
        outsideEightMeterRadius.updateWithAbsoluteFix(0.0, 0.0, 0, 1100L, 1.0);
        assertParticlesStayAtX(outsideEightMeterRadius.snapshotParticlesForTesting(), 0.0);

        ParticleFilterEngine insideSixSigmaRadius = createDeterministicEngine();
        insideSixSigmaRadius.setParticlesForTesting(Arrays.asList(
                new Particle(12.0, 0.0, 0, 0.5, 0.0),
                new Particle(12.0, 0.0, 0, 0.5, 0.0)
        ), 1000L);
        insideSixSigmaRadius.updateWithAbsoluteFix(0.0, 0.0, 0, 1100L, 2.0);
        assertParticlesStayAtX(insideSixSigmaRadius.snapshotParticlesForTesting(), 12.0);

        ParticleFilterEngine outsideSixSigmaRadius = createDeterministicEngine();
        outsideSixSigmaRadius.setParticlesForTesting(Arrays.asList(
                new Particle(12.01, 0.0, 0, 0.5, 0.0),
                new Particle(12.01, 0.0, 0, 0.5, 0.0)
        ), 1000L);
        outsideSixSigmaRadius.updateWithAbsoluteFix(0.0, 0.0, 0, 1100L, 2.0);
        assertParticlesStayAtX(outsideSixSigmaRadius.snapshotParticlesForTesting(), 0.0);
    }

    @Test
    public void normalizeWeightsRecoversFromInvalidState() {
        ParticleFilterEngine engine = createDeterministicEngine();
        engine.setParticlesForTesting(Arrays.asList(
                new Particle(0.0, 0.0, 0, Double.NaN, 0.0),
                new Particle(1.0, 0.0, 0, Double.POSITIVE_INFINITY, 0.0),
                new Particle(2.0, 0.0, 0, -1.0, 0.0)
        ), 1000L);

        engine.normalizeWeights();

        List<Particle> particles = engine.snapshotParticlesForTesting();
        assertEquals(3, particles.size());
        for (Particle particle : particles) {
            assertTrue(Double.isFinite(particle.getWeight()));
            assertEquals(1.0 / 3.0, particle.getWeight(), 1e-6);
        }
    }

    @Test
    public void updateResamplesAndPreservesParticleCount() {
        ParticleFilterEngine engine = createDeterministicEngine();
        engine.setParticlesForTesting(Arrays.asList(
                new Particle(0.0, 0.0, 0, 0.25, 0.0),
                new Particle(30.0, 0.0, 0, 0.25, 0.0),
                new Particle(60.0, 0.0, 0, 0.25, 0.0),
                new Particle(90.0, 0.0, 0, 0.25, 0.0)
        ), 1000L);

        engine.updateWithAbsoluteFix(90.0, 0.0, 0, 1100L, 1.0);

        List<Particle> particles = engine.snapshotParticlesForTesting();
        assertEquals(4, particles.size());
        for (Particle particle : particles) {
            assertEquals(90.0, particle.getX(), 1e-6);
            assertEquals(0.0, particle.getY(), 1e-6);
            assertEquals(0.25, particle.getWeight(), 1e-6);
        }
    }

    @Test
    public void estimatePoseUsesDominantFloorSubset() {
        ParticleFilterEngine engine = createDeterministicEngine();
        engine.setParticlesForTesting(Arrays.asList(
                new Particle(0.0, 0.0, 0, 0.4, 0.0),
                new Particle(0.0, 0.0, 0, 0.4, 0.0),
                new Particle(100.0, 100.0, 1, 0.2, 0.0)
        ), 1000L);

        FusedPose pose = engine.estimatePose();

        assertEquals(0, pose.getFloor());
        assertEquals(0.0, pose.getX(), 1e-6);
        assertEquals(0.0, pose.getY(), 1e-6);
    }

    @Test
    public void estimatePoseFallsBackToLegalParticleWhenWeightedMeanIsIllegal() {
        ParticleInitializer.SpawnValidator validator = (x, y, floor) -> x < 0.5 || x > 1.0;
        ParticleFilterEngine engine = new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                validator,
                new ZeroRandom()
        );
        engine.setParticlesForTesting(Arrays.asList(
                new Particle(0.0, 0.0, 0, 0.75, 0.0),
                new Particle(3.0, 0.0, 0, 0.25, 0.0)
        ), 1000L);

        FusedPose pose = engine.estimatePose();

        assertEquals(0.0, pose.getX(), 1e-6);
        assertEquals(0.0, pose.getY(), 1e-6);
        assertEquals(0, pose.getFloor());
    }

    @Test
    public void endpointLockedRecoveryUsesExactLegalFixWithoutSeedingIllegalParticles() {
        ParticleInitializer.SpawnValidator validator = new ParticleInitializer.SpawnValidator() {
            @Override
            public boolean isValid(double x, double y, int floor) {
                return x <= 0.0 || x >= 0.5;
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
                return predictedX <= 0.0 || previousX >= 0.5;
            }
        };
        ParticleFilterEngine engine = new ParticleFilterEngine(
                new ParticleInitializer(new SequenceRandom(
                        1.0, 0.0,
                        1.0, 0.0,
                        1.0, 0.0,
                        1.0, 0.0,
                        1.0, 0.0,
                        1.0, 0.0,
                        1.0, 0.0,
                        1.0, 0.0,
                        1.0, 0.0,
                        1.0, 0.0
                )),
                validator,
                new ZeroRandom()
        );
        engine.setParticlesForTesting(Arrays.asList(
                new Particle(-20.0, 0.0, 0, 0.5, 0.0),
                new Particle(-20.0, 0.0, 0, 0.5, 0.0)
        ), 1000L);

        engine.updateWithAbsoluteFix(0.0, 0.0, 0, 1100L, 1.0);

        assertTrue(engine.wasLastAbsoluteFixReanchored());
        assertFalse(engine.wasLastAbsoluteFixRejectedByConstraints());
        assertParticlesStayAtX(engine.snapshotParticlesForTesting(), 0.0);
    }

    @Test
    public void repeatedWallRejectsMarkCloudTrappedAndRecoveryHintInjectsRescueParticles() {
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
                double distanceMeters = Math.hypot(predictedX - previousX, predictedY - previousY);
                if (previousX < 10.0) {
                    return distanceMeters < 0.1;
                }
                if (previousX >= 40.0) {
                    return distanceMeters <= 5.0;
                }
                return false;
            }
        };
        ParticleFilterEngine engine = new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                validator,
                new ZeroRandom()
        );
        List<Particle> trappedParticles = new ArrayList<>();
        for (int i = 0; i < 10; i++) {
            trappedParticles.add(new Particle(0.0, 0.0, 0, 0.1, 0.0));
        }
        engine.setParticlesForTesting(trappedParticles, 1_000L);

        for (int i = 0; i < 5; i++) {
            engine.predict(new PdrDelta(1.0f, 0.0f, 0.0f), 0, 1_100L + i);
        }

        assertTrue(engine.isCloudTrapped());
        assertTrue(engine.getLastPredictWallRejectRatio() > 0.85);

        engine.updateWithAbsoluteFix(50.0, 0.0, Integer.valueOf(0), 2_000L, 4.5, true);

        assertTrue(engine.wasLastAbsoluteFixCloudRecovered());
        assertFalse(engine.wasLastAbsoluteFixRejectedByConstraints());
        FusedPose pose = engine.estimatePose();
        assertTrue(pose.getX() >= 49.0);
        assertEquals(0, pose.getFloor());
    }

    @Test
    public void trappedCloudWithoutAbsoluteFixesUsesBlindBreakoutToResumePdr() {
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
                return previousX >= 2.5 || predictedX <= previousX;
            }
        };
        ParticleFilterEngine engine = new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                validator,
                new ZeroRandom()
        );
        engine.setParticlesForTesting(Arrays.asList(
                new Particle(0.0, 0.0, 0, 0.5, Math.PI / 2.0),
                new Particle(0.0, 0.0, 0, 0.5, Math.PI / 2.0)
        ), 1_000L);

        for (int i = 0; i < 16; i++) {
            engine.predict(new PdrDelta(1.0f, 0.0f, 0.0f), 0, 1_100L + i);
        }

        FusedPose pose = engine.estimatePose();
        assertTrue(pose.getX() >= 4.0);
        assertEquals(0.0, pose.getY(), 1e-6);
        assertFalse(engine.isCloudTrapped());
        assertTrue(engine.getLastPredictWallRejectRatio() < 0.85);
    }

    private ParticleFilterEngine createDeterministicEngine() {
        return new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                ParticleInitializer.allowAll(),
                new ZeroRandom()
        );
    }

    private void assertParticlesStayAtX(List<Particle> particles, double expectedX) {
        assertEquals(2, particles.size());
        for (Particle particle : particles) {
            assertEquals(expectedX, particle.getX(), 1e-6);
            assertEquals(0.0, particle.getY(), 1e-6);
            assertEquals(0.5, particle.getWeight(), 1e-6);
        }
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
            return 1.0;
        }

        @Override
        public double nextDouble() {
            return 0.5;
        }
    }
}
