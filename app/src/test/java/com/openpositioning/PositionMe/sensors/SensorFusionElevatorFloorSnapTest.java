package com.openpositioning.PositionMe.sensors;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class SensorFusionElevatorFloorSnapTest {

    @Test
    public void elevatorSnapUsesConservativeRoundingForPositiveOvershoot() {
        assertEquals(2, SensorFusion.resolveElevatorSessionFloorDelta(8.9f, 3.6f));
    }

    @Test
    public void elevatorSnapSupportsDescendingTrips() {
        assertEquals(-1, SensorFusion.resolveElevatorSessionFloorDelta(-3.1f, 3.6f));
    }

    @Test
    public void elevatorSnapRejectsSubFloorHeightNoiseBelowNewThreshold() {
        assertEquals(0, SensorFusion.resolveElevatorSessionFloorDelta(2.6f, 3.6f));
    }

    @Test
    public void elevatorSnapIgnoresTinyPressureDrift() {
        assertEquals(0, SensorFusion.resolveElevatorSessionFloorDelta(0.9f, 3.6f));
    }
}
