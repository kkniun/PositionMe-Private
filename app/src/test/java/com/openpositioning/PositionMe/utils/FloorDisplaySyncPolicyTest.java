package com.openpositioning.PositionMe.utils;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class FloorDisplaySyncPolicyTest {

    @Test
    public void resolve_initialWifiMismatch_prefersWifiAfterStability() {
        FloorDisplaySyncPolicy.Decision decision = FloorDisplaySyncPolicy.resolve(
                false,
                true,
                1,
                0,
                false
        );

        assertEquals(1, decision.getLogicalFloor());
        assertTrue(decision.requiresStability());
        assertTrue(decision.shouldCommitToFusion());
        assertTrue(decision.shouldCompleteInitialSync());
    }

    @Test
    public void resolve_initialWifiAgreement_onlyCompletesInitialSync() {
        FloorDisplaySyncPolicy.Decision decision = FloorDisplaySyncPolicy.resolve(
                false,
                true,
                1,
                1,
                false
        );

        assertEquals(1, decision.getLogicalFloor());
        assertTrue(decision.requiresStability());
        assertFalse(decision.shouldCommitToFusion());
        assertTrue(decision.shouldCompleteInitialSync());
    }

    @Test
    public void resolve_afterInitialSync_requiresTransitionFeatureForWifiOverride() {
        FloorDisplaySyncPolicy.Decision rejected = FloorDisplaySyncPolicy.resolve(
                true,
                true,
                2,
                1,
                false
        );
        assertEquals(1, rejected.getLogicalFloor());
        assertFalse(rejected.requiresStability());
        assertFalse(rejected.shouldCommitToFusion());

        FloorDisplaySyncPolicy.Decision accepted = FloorDisplaySyncPolicy.resolve(
                true,
                true,
                2,
                1,
                true
        );
        assertEquals(2, accepted.getLogicalFloor());
        assertTrue(accepted.requiresStability());
        assertTrue(accepted.shouldCommitToFusion());
        assertFalse(accepted.shouldCompleteInitialSync());
    }
}
