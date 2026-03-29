package com.openpositioning.PositionMe.utils;

import org.junit.Test;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class AutoFloorSwitchGateTest {

    @Test
    public void sameFloorNeverTriggersSwitch() {
        AutoFloorSwitchGate gate = new AutoFloorSwitchGate();

        assertFalse(gate.shouldApply(1, 1, 1_000L, 600L));
        assertFalse(gate.shouldApply(1, 1, 1_800L, 600L));
    }

    @Test
    public void candidateFloorMustRemainStableBeforeSwitching() {
        AutoFloorSwitchGate gate = new AutoFloorSwitchGate();

        assertFalse(gate.shouldApply(2, 1, 1_000L, 600L));
        assertFalse(gate.shouldApply(2, 1, 1_500L, 600L));
        assertTrue(gate.shouldApply(2, 1, 1_601L, 600L));
    }

    @Test
    public void oscillationResetsPendingFloorChange() {
        AutoFloorSwitchGate gate = new AutoFloorSwitchGate();

        assertFalse(gate.shouldApply(2, 1, 1_000L, 600L));
        assertFalse(gate.shouldApply(3, 1, 1_300L, 600L));
        assertFalse(gate.shouldApply(2, 1, 1_601L, 600L));
    }
}
