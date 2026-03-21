package com.openpositioning.PositionMe.sensors;

import com.openpositioning.PositionMe.utils.PdrProcessing;

import org.junit.Test;

import java.util.Arrays;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.assertEquals;
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
}
