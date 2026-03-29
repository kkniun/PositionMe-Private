package com.openpositioning.PositionMe.presentation.fragment;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

public class ReplayFragmentFloorSyncTest {

    @Test
    public void replayUsesExplicitRecordedFloorPathInsteadOfLiveAutoSync() {
        assertEquals(Integer.valueOf(0), FloorDisplayGate.resolveReplayFloorForDisplay(0));
        assertFalse(FloorDisplayGate.shouldAllowAutoFloorSync(true, false, 0));
    }

    @Test
    public void replayRecordedFloorPathIsDistinctFromLiveTrustedGate() {
        assertEquals(Integer.valueOf(3), FloorDisplayGate.resolveReplayFloorForDisplay(3));
        assertFalse(FloorDisplayGate.shouldAllowUserVisibleFloor(false, 3));
    }
}
