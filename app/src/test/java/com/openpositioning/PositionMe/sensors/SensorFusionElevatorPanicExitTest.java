package com.openpositioning.PositionMe.sensors;

import org.junit.Test;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class SensorFusionElevatorPanicExitTest {

    @Test
    public void panicExitRequiresMoreThanFiveConfirmedSteps() {
        assertFalse(SensorFusion.shouldForceExitElevatorFromSteps(5));
        assertTrue(SensorFusion.shouldForceExitElevatorFromSteps(6));
    }

    @Test
    public void panicExitRequiresFiveSecondsOfBarometerStillnessBelowThreshold() {
        assertFalse(SensorFusion.shouldForceExitElevatorFromBarometerStillness(0.05f, 5_000L));
        assertFalse(SensorFusion.shouldForceExitElevatorFromBarometerStillness(0.04f, 4_999L));
        assertTrue(SensorFusion.shouldForceExitElevatorFromBarometerStillness(0.04f, 5_000L));
    }

    @Test
    public void panicExitRequiresSixtySecondSessionTimeout() {
        assertFalse(SensorFusion.shouldForceExitElevatorFromTimeout(59_999L));
        assertTrue(SensorFusion.shouldForceExitElevatorFromTimeout(60_000L));
    }
}
