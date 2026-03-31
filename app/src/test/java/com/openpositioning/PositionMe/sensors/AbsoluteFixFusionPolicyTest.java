package com.openpositioning.PositionMe.sensors;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class AbsoluteFixFusionPolicyTest {

    @Test
    public void strongLowConfidenceWifiConflictIsRejected() {
        AbsoluteFixFusionPolicy.ConflictAdjustedAbsoluteFix result =
                AbsoluteFixFusionPolicy.resolveConflictAdjustedAbsoluteFix(
                        "WIFI",
                        5.0f,
                        0.28,
                        8.5,
                        16.0,
                        8.5
                );

        assertTrue(result.rejectObservation);
        assertTrue(result.weakAnchorMode);
        assertEquals("rejected_low_conflict_anchor_shift", result.reasonCode);
    }

    @Test
    public void weakWifiConflictIsDownweightedBeforeItCanDriveRecovery() {
        AbsoluteFixFusionPolicy.ConflictAdjustedAbsoluteFix result =
                AbsoluteFixFusionPolicy.resolveConflictAdjustedAbsoluteFix(
                        "WIFI",
                        5.0f,
                        0.34,
                        6.8,
                        7.8,
                        null
                );

        assertFalse(result.rejectObservation);
        assertTrue(result.weakAnchorMode);
        assertTrue(result.adjustedAccuracyMeters > 6.0f);
        assertEquals("downweighted_conflicting_absolute_fix", result.reasonCode);
    }

    @Test
    public void stableCredibleFixKeepsOriginalAccuracy() {
        AbsoluteFixFusionPolicy.ConflictAdjustedAbsoluteFix result =
                AbsoluteFixFusionPolicy.resolveConflictAdjustedAbsoluteFix(
                        "GNSS",
                        3.0f,
                        0.82,
                        1.2,
                        2.0,
                        1.5
                );

        assertFalse(result.rejectObservation);
        assertFalse(result.weakAnchorMode);
        assertEquals(3.0f, result.adjustedAccuracyMeters, 1e-6f);
        assertEquals("stable", result.reasonCode);
    }

    @Test
    public void weakAnchorModeDampsTranslationWithoutFreezingPdr() {
        float scale = AbsoluteFixFusionPolicy.resolveWeakAnchorPdrTranslationScale(
                0.92f,
                0.28,
                11.0,
                true
        );

        assertTrue(scale < 1.0f);
        assertTrue(scale >= 0.55f);
    }

    @Test
    public void consistentFarAbsoluteClusterTriggersRecoveryHint() {
        AbsoluteFixFusionPolicy.ConflictAdjustedAbsoluteFix result =
                AbsoluteFixFusionPolicy.resolveConflictAdjustedAbsoluteFix(
                        "WIFI",
                        7.0f,
                        0.30,
                        12.0,
                        3.0,
                        null,
                        3
                );

        assertFalse(result.rejectObservation);
        assertFalse(result.weakAnchorMode);
        assertTrue(result.recoveryHint);
        assertEquals(4.5f, result.adjustedAccuracyMeters, 1e-6f);
        assertEquals("consistent_absolute_cluster_override", result.reasonCode);
    }
}
