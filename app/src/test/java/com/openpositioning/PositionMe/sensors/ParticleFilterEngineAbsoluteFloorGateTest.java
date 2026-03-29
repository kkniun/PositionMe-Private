package com.openpositioning.PositionMe.sensors;

import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class ParticleFilterEngineAbsoluteFloorGateTest {

    @Test
    public void absoluteFixFloorIsDroppedWhenTransitionGateRejectsIt() {
        ParticleFilterEngine engine = new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                ParticleInitializer.allowAll(),
                rejectingGate(),
                new ZeroRandom()
        );

        engine.initialize(0.0, 0.0, 0, 1000L, 0.0, 64, 0.1);
        engine.updateWithAbsoluteFix(20.0, 0.0, Integer.valueOf(2), 1100L, 1.0);

        assertTrue(engine.wasLastAbsoluteFixReanchored());
        assertEquals(0, engine.estimatePose().getFloor());
    }

    @Test
    public void absoluteFixFloorIsKeptWhenTransitionGateAllowsIt() {
        ParticleFilterEngine engine = new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                ParticleInitializer.allowAll(),
                allowingGate(),
                new ZeroRandom()
        );

        engine.initialize(0.0, 0.0, 0, 1000L, 0.0, 64, 0.1);
        engine.updateWithAbsoluteFix(20.0, 0.0, Integer.valueOf(2), 1100L, 1.0);

        assertTrue(engine.wasLastAbsoluteFixReanchored());
        assertEquals(2, engine.estimatePose().getFloor());
    }

    @Test
    public void reanchorRecoveryCanUseRequestedFloorWhenCurrentFloorLockIsWrong() {
        ParticleInitializer.SpawnValidator floorAwareValidator = new ParticleInitializer.SpawnValidator() {
            @Override
            public boolean isValid(double x, double y, int floor) {
                return floor == 2;
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
                floorAwareValidator,
                rejectingGate(),
                new ZeroRandom()
        );

        engine.setParticlesForTesting(java.util.List.of(
                new Particle(0.0, 0.0, 0, 0.5, 0.0),
                new Particle(0.0, 0.0, 0, 0.5, 0.0)
        ), 1000L);
        engine.updateWithAbsoluteFix(30.0, 0.0, Integer.valueOf(2), 1100L, 4.0);

        assertTrue(engine.wasLastAbsoluteFixReanchored());
        assertEquals(2, engine.estimatePose().getFloor());
        assertEquals(30.0, engine.estimatePose().getX(), 1e-6);
    }

    private FloorTransitionGate rejectingGate() {
        return (previousX, previousY, predictedX, predictedY, previousFloor, predictedFloor) ->
                previousFloor == predictedFloor;
    }

    private FloorTransitionGate allowingGate() {
        return (previousX, previousY, predictedX, predictedY, previousFloor, predictedFloor) -> true;
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
