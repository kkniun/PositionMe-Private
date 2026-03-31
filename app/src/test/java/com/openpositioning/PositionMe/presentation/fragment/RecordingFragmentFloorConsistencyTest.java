package com.openpositioning.PositionMe.presentation.fragment;

import com.openpositioning.PositionMe.sensors.FusedPose;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class RecordingFragmentFloorConsistencyTest {

    @Test
    public void mapUpdateFloorPrefersTrustedDisplayFloorWhenAvailable() {
        FusedPose fusedPose = new FusedPose(4.0, 8.0, 2, 0.85, 9_000L);

        assertEquals(Integer.valueOf(1), RecordingFragment.resolveMapUpdateFloor(Integer.valueOf(1), fusedPose));
        assertEquals(2, RecordingFragment.resolveMapUpdateFloor(fusedPose));
    }

    @Test
    public void mapUpdateFloorFallsBackToFusedPoseFloorWhenTrustedFloorUnavailable() {
        FusedPose fusedPose = new FusedPose(4.0, 8.0, 2, 0.85, 9_000L);

        assertEquals(Integer.valueOf(2), RecordingFragment.resolveMapUpdateFloor(null, fusedPose));
    }

    @Test
    public void trackedDistanceRejectsHeadingOnlyOrPassiveDrift() {
        assertFalse(RecordingFragment.shouldAccumulateTrackedDistance(
                false,
                0.90,
                false,
                "PDR",
                10_000L,
                9_900L
        ));

        assertFalse(RecordingFragment.shouldAccumulateTrackedDistance(
                true,
                0.90,
                false,
                "WIFI",
                10_000L,
                Long.MIN_VALUE
        ));

        assertFalse(RecordingFragment.shouldAccumulateTrackedDistance(
                true,
                0.90,
                true,
                "GNSS",
                10_000L,
                Long.MIN_VALUE
        ));

        assertTrue(RecordingFragment.shouldAccumulateTrackedDistance(
                true,
                0.90,
                false,
                "PDR",
                10_000L,
                Long.MIN_VALUE
        ));
    }
}
