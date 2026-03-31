package com.openpositioning.PositionMe.sensors;

import org.junit.Test;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class SensorFusionStepMotionGateTest {

    @Test
    public void stationaryPseudoStepNeedsIndependentMotionEvidence() {
        assertFalse(SensorFusion.hasIndependentMotionEvidenceForStep(
                true,
                0.05,
                0.08,
                0,
                false
        ));
    }

    @Test
    public void realMotionStillAllowsStepPromotionFromStationaryState() {
        assertTrue(SensorFusion.hasIndependentMotionEvidenceForStep(
                true,
                0.12,
                0.08,
                0,
                false
        ));
        assertTrue(SensorFusion.hasIndependentMotionEvidenceForStep(
                true,
                0.05,
                0.11,
                0,
                false
        ));
        assertTrue(SensorFusion.hasIndependentMotionEvidenceForStep(
                true,
                0.05,
                0.08,
                2,
                false
        ));
    }

    @Test
    public void strongBarometerMotionCanStillPromoteTransitionStep() {
        assertTrue(SensorFusion.hasIndependentMotionEvidenceForStep(
                true,
                0.01,
                0.02,
                0,
                true
        ));
    }

    @Test
    public void alreadyMovingStateDoesNotNeedExtraStepEvidence() {
        assertTrue(SensorFusion.hasIndependentMotionEvidenceForStep(
                false,
                0.01,
                0.02,
                0,
                false
        ));
    }
}
