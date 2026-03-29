package com.openpositioning.PositionMe.sensors;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class SensorFusionTransitionPreferenceTest {

    @Test
    public void elevatorSignalPrefersLiftWhenBothTransitionTypesExist() {
        assertEquals(
                SensorFusion.TransitionPreference.LIFT_ONLY,
                SensorFusion.resolveTransitionPreference(true, 0.3, true, true)
        );
    }

    @Test
    public void stairLikeMotionPrefersStairsWhenBothTransitionTypesExist() {
        assertEquals(
                SensorFusion.TransitionPreference.STAIRS_ONLY,
                SensorFusion.resolveTransitionPreference(false, 0.9, true, true)
        );
    }

    @Test
    public void ambiguousMotionAllowsEitherTransitionType() {
        assertEquals(
                SensorFusion.TransitionPreference.ANY_AVAILABLE,
                SensorFusion.resolveTransitionPreference(true, 3.5, true, true)
        );
    }

    @Test
    public void availableGeometryOverridesWeakClassifier() {
        assertEquals(
                SensorFusion.TransitionPreference.LIFT_ONLY,
                SensorFusion.resolveTransitionPreference(false, 1.0, true, false)
        );
        assertEquals(
                SensorFusion.TransitionPreference.STAIRS_ONLY,
                SensorFusion.resolveTransitionPreference(true, 0.0, false, true)
        );
    }
}
