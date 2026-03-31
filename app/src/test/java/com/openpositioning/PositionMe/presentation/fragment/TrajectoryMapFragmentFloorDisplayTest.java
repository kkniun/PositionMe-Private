package com.openpositioning.PositionMe.presentation.fragment;

import org.junit.After;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

public class TrajectoryMapFragmentFloorDisplayTest {

    @After
    public void tearDown() {
        MapUiStateResolver.clearRememberedAutoFloorStateForTesting();
    }

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
    public void reverseTransitionTrustedFloorOverridesStaleMapAndRenderedFloor() {
        assertEquals(
                Integer.valueOf(1),
                FloorDisplayGate.resolveDisplayFloor(1, 2, 2)
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

    @Test
    public void autoFloorStateRestoresAcrossNonUserRefresh() {
        assertEquals(
                true,
                MapUiStateResolver.resolveRestoredAutoFloorState(Boolean.TRUE, false)
        );
        assertEquals(
                false,
                MapUiStateResolver.resolveRestoredAutoFloorState(Boolean.FALSE, true)
        );
        assertEquals(
                true,
                MapUiStateResolver.resolveRestoredAutoFloorState(null, true)
        );
    }

    @Test
    public void freshRecordingEntryDoesNotAutoUnboxNullAutoFloorState() {
        assertNull(
                MapUiStateResolver.resolveSavedOrRememberedAutoFloorState(null, false, false)
        );
    }

    @Test
    public void initializedAutoFloorStateIsReusedWithoutSavedBundle() {
        assertEquals(
                Boolean.TRUE,
                MapUiStateResolver.resolveSavedOrRememberedAutoFloorState(null, true, true)
        );
        assertEquals(
                Boolean.FALSE,
                MapUiStateResolver.resolveSavedOrRememberedAutoFloorState(null, true, false)
        );
    }

    @Test
    public void savedBundleAutoFloorStateOverridesTransientState() {
        assertEquals(
                Boolean.FALSE,
                MapUiStateResolver.resolveSavedOrRememberedAutoFloorState(Boolean.FALSE, true, true)
        );
    }

    @Test
    public void rememberedAutoFloorStateSurvivesViewRecreationWithoutBundle() {
        MapUiStateResolver.rememberAutoFloorState(true);

        assertEquals(
                Boolean.TRUE,
                MapUiStateResolver.resolveSavedOrRememberedAutoFloorState(null, false, false)
        );
    }
}
