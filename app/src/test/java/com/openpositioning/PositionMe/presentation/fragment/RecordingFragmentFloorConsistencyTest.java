package com.openpositioning.PositionMe.presentation.fragment;

import com.openpositioning.PositionMe.sensors.FusedPose;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class RecordingFragmentFloorConsistencyTest {

    @Test
    public void mapUpdateFloorAlwaysUsesCommittedFusedPoseFloor() {
        FusedPose fusedPose = new FusedPose(4.0, 8.0, 2, 0.85, 9_000L);

        assertEquals(2, RecordingFragment.resolveMapUpdateFloor(fusedPose));
    }
}
