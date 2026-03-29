package com.openpositioning.PositionMe.presentation.fragment;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

public class TrajectoryMapFragmentFloorDisplayTest {

    @Test
    public void mapAutoFloorDoesNotSwitchToDefaultGroundFloorBeforeCalibration() {
        assertNull(FloorDisplayGate.resolveTrustedFloorForDisplay(false, 0));
    }

    @Test
    public void mapAutoFloorUpdatesAfterTrustedCalibration() {
        assertEquals(Integer.valueOf(2), FloorDisplayGate.resolveTrustedFloorForDisplay(true, 2));
    }

    @Test
    public void untrustedFloorIsNeverAutoSyncedToMap() {
        assertEquals(
                Integer.valueOf(4),
                FloorDisplayGate.resolveDisplayFloor(null, 4, 2)
        );
        assertEquals(
                Integer.valueOf(2),
                FloorDisplayGate.resolveDisplayFloor(null, null, 2)
        );
        assertNull(FloorDisplayGate.resolveDisplayFloor(null, null, null));
    }

    @Test
    public void trustedFloorStillDrivesIncomingMapDisplay() {
        assertEquals(
                Integer.valueOf(6),
                FloorDisplayGate.resolveDisplayFloor(6, 2, 1)
        );
    }

    @Test
    public void untrustedVenueLabelNeverShowsGroundFloorToUser() {
        assertNull(FloorDisplayGate.resolveTrustedFloorForVenueLabel(false, 0));
    }

    @Test
    public void incomingPoseStillHasDisplayFloorWhileFloorTrustIsPending() {
        assertEquals(
                Integer.valueOf(3),
                FloorDisplayGate.resolveDisplayFloor(null, 3, null)
        );
        assertNull(FloorDisplayGate.resolveTrustedFloorForDisplay(false, 0));
    }

    @Test
    public void trustedVenueLabelStillShowsResolvedFloorNormally() {
        assertEquals(
                Integer.valueOf(2),
                FloorDisplayGate.resolveTrustedFloorForVenueLabel(true, 2)
        );
    }
}
