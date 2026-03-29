package com.openpositioning.PositionMe.utils;

import org.junit.Test;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class PdrProcessingElevatorClassifierTest {

    @Test
    public void detectsElevatorLikeMotionWhenVerticalDominatesAndHorizontalIsLow() {
        assertTrue(PdrProcessing.isElevatorLikeMotion(0.22f, 0.14f, 0.30f, 0.12f));
    }

    @Test
    public void rejectsWalkingLikeMotionWhenHorizontalAccelerationIsTooHigh() {
        assertFalse(PdrProcessing.isElevatorLikeMotion(0.45f, 0.20f, 0.30f, 0.12f));
    }
}
