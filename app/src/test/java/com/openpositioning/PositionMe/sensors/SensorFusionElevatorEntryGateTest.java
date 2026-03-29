package com.openpositioning.PositionMe.sensors;

import org.junit.Test;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class SensorFusionElevatorEntryGateTest {

    @Test
    public void elevatorEntryRequiresSustainedPositiveSignal() {
        assertFalse(SensorFusion.shouldConfirmElevatorEntry(2, 1_400L));
        assertTrue(SensorFusion.shouldConfirmElevatorEntry(2, 1_500L));
    }

    @Test
    public void elevatorEntryStillRequiresMultiplePositiveDetections() {
        assertFalse(SensorFusion.shouldConfirmElevatorEntry(1, 3_000L));
        assertTrue(SensorFusion.shouldConfirmElevatorEntry(2, 3_000L));
    }

    @Test
    public void recentWalkingCadenceSuppressesBarometerOnlyElevatorEntry() {
        assertTrue(SensorFusion.shouldSuppressElevatorForRecentSteps(3, 200L));
        assertTrue(SensorFusion.shouldSuppressElevatorForRecentSteps(2, 1_000L));
        assertFalse(SensorFusion.shouldSuppressElevatorForRecentSteps(1, 200L));
        assertFalse(SensorFusion.shouldSuppressElevatorForRecentSteps(2, 1_500L));
    }
}
