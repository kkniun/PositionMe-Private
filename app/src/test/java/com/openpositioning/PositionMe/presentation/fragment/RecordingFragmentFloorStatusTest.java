package com.openpositioning.PositionMe.presentation.fragment;

import org.junit.Test;

import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class RecordingFragmentFloorStatusTest {

    @Test
    public void floorDisplayShowsUnknownUntilFloorBecomesTrusted() {
        assertTrue(FloorDisplayGate.shouldShowUnknownFloorStatus(false));
        assertFalse(FloorDisplayGate.shouldShowUnknownFloorStatus(true));
    }

    @Test
    public void untrustedFloorIsNeverDisplayedToUserAsGroundFloor() {
        assertFalse(FloorDisplayGate.shouldAllowUserVisibleFloor(false, 0));
        assertTrue(FloorDisplayGate.shouldShowUnknownFloorStatus(false));
    }

    @Test
    public void recordingRefreshDoesNotSyncDefaultGroundFloorBeforeCalibration() {
        AtomicInteger syncedFloor = new AtomicInteger(Integer.MIN_VALUE);

        boolean synced = FloorDisplayGate.syncTrustedAutoDisplayedFloor(
                true,
                false,
                0,
                syncedFloor::set
        );

        assertFalse(synced);
        assertEquals(Integer.MIN_VALUE, syncedFloor.get());
    }

    @Test
    public void groundFloorCanStillWorkWhenItIsActuallyTrusted() {
        AtomicInteger syncedFloor = new AtomicInteger(Integer.MIN_VALUE);

        boolean synced = FloorDisplayGate.syncTrustedAutoDisplayedFloor(
                true,
                true,
                0,
                syncedFloor::set
        );

        assertTrue(FloorDisplayGate.shouldAllowUserVisibleFloor(true, 0));
        assertTrue(FloorDisplayGate.shouldAllowBaselineRecalibration(true, true, 0));
        assertTrue(synced);
        assertEquals(0, syncedFloor.get());
    }

    @Test
    public void recordingRefreshSyncsTrustedFloorAfterCalibration() {
        AtomicInteger syncedFloor = new AtomicInteger(Integer.MIN_VALUE);

        boolean synced = FloorDisplayGate.syncTrustedAutoDisplayedFloor(
                true,
                true,
                2,
                syncedFloor::set
        );

        assertTrue(synced);
        assertEquals(2, syncedFloor.get());
    }

    @Test
    public void trustedFloorStillDisplaysAndSyncsNormally() {
        AtomicInteger syncedFloor = new AtomicInteger(Integer.MIN_VALUE);

        assertTrue(FloorDisplayGate.shouldAllowUserVisibleFloor(true, 3));
        assertTrue(FloorDisplayGate.shouldAllowBaselineRecalibration(true, true, 3));
        assertTrue(FloorDisplayGate.syncTrustedAutoDisplayedFloor(
                true,
                true,
                3,
                syncedFloor::set
        ));
        assertEquals(3, syncedFloor.get());
    }

    @Test
    public void floorStatusAndMapSyncStayConsistentAcrossCalibrationBoundary() {
        AtomicInteger syncedFloor = new AtomicInteger(Integer.MIN_VALUE);

        boolean uncalibratedSync = FloorDisplayGate.syncTrustedAutoDisplayedFloor(
                true,
                false,
                0,
                syncedFloor::set
        );
        assertTrue(FloorDisplayGate.shouldShowUnknownFloorStatus(false));
        assertFalse(FloorDisplayGate.shouldAllowBaselineRecalibration(false, true, 0));
        assertFalse(uncalibratedSync);
        assertEquals(Integer.MIN_VALUE, syncedFloor.get());

        boolean calibratedSync = FloorDisplayGate.syncTrustedAutoDisplayedFloor(
                true,
                true,
                3,
                syncedFloor::set
        );
        assertFalse(FloorDisplayGate.shouldShowUnknownFloorStatus(true));
        assertTrue(calibratedSync);
        assertEquals(3, syncedFloor.get());
    }
}
