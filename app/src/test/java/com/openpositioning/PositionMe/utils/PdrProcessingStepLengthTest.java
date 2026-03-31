package com.openpositioning.PositionMe.utils;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class PdrProcessingStepLengthTest {

    @Test
    public void slowCadenceLongStepIsClampedToIndoorConservativeRange() {
        float adjustedStepLength = PdrProcessing.applyCadenceAwareStepLength(1.20f, 1_250L);

        assertTrue(adjustedStepLength < 0.80f);
        assertEquals(0.76f, adjustedStepLength, 0.05f);
    }

    @Test
    public void briskCadenceKeepsNormalWalkingStepMostlyIntact() {
        float adjustedStepLength = PdrProcessing.applyCadenceAwareStepLength(0.82f, 700L);

        assertTrue(adjustedStepLength > 0.75f);
        assertEquals(0.80f, adjustedStepLength, 0.05f);
    }

    @Test
    public void idleGapFallsBackToModerateSlowCadenceInsteadOfFreezingFirstStep() {
        float adjustedStepLength = PdrProcessing.applyCadenceAwareStepLength(1.0f, 2_800L);

        assertTrue(adjustedStepLength > 0.70f);
        assertTrue(adjustedStepLength < 0.82f);
    }
}
