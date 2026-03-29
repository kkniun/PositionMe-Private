package com.openpositioning.PositionMe.presentation.fragment;

import com.google.android.gms.maps.model.LatLng;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class TrajectoryMapFragmentDeadbandTest {

    @Test
    public void holdsStationaryDisplayPositionForTinyJitter() {
        LatLng current = new LatLng(55.9444000, -3.1878000);
        LatLng tinyJitter = new LatLng(55.9444008, -3.1878000);

        assertTrue(MapPointerDisplayFilter.shouldHoldStationaryDisplayPosition(current, tinyJitter, true));
    }

    @Test
    public void doesNotHoldDisplayPositionWhenWalking() {
        LatLng current = new LatLng(55.9444000, -3.1878000);
        LatLng moved = new LatLng(55.9444100, -3.1878000);

        assertFalse(MapPointerDisplayFilter.shouldHoldStationaryDisplayPosition(current, moved, true));
        assertFalse(MapPointerDisplayFilter.shouldHoldStationaryDisplayPosition(current, moved, false));
    }

    @Test
    public void markerHoldPrefersElevatorReasonWhenElevatorActive() {
        assertEquals(
                "elevator",
                MapPointerDisplayFilter.resolveMarkerHoldReason(
                        true,
                        true,
                        false,
                        MapMatchingStateResolver.MapMatchingUiState.ACTIVE,
                        500L
                )
        );
    }

    @Test
    public void markerHoldUsesConstraintsLoadingWhenPoseExistsButMapPending() {
        assertEquals(
                "constraints_loading",
                MapPointerDisplayFilter.resolveMarkerHoldReason(
                        true,
                        false,
                        false,
                        MapMatchingStateResolver.MapMatchingUiState.PENDING,
                        500L
                )
        );
    }

    @Test
    public void headingHoldUsesAwaitingMotionForHeadingOnlyUpdates() {
        assertEquals(
                "awaiting_motion",
                MapPointerDisplayFilter.resolveHeadingHoldReason(
                        false,
                        true,
                        false,
                        false,
                        "none",
                        Double.NaN
                )
        );
    }

    @Test
    public void headingHoldKeepsStationaryAsHardStopBeforeHeadingOnly() {
        assertEquals(
                "stationary",
                MapPointerDisplayFilter.resolveHeadingHoldReason(
                        false,
                        true,
                        true,
                        false,
                        "none",
                        Double.NaN
                )
        );
    }

    @Test
    public void headingHoldMarksTinySamePoseAdvanceAsInsufficientMovement() {
        assertEquals(
                "insufficient_movement",
                MapPointerDisplayFilter.resolveHeadingHoldReason(
                        false,
                        false,
                        false,
                        false,
                        "none",
                        0.10
                )
        );
    }

    @Test
    public void headingHoldUsesLowConfidenceBeforeTinyMoveHeuristic() {
        assertEquals(
                "low_confidence",
                MapPointerDisplayFilter.resolveHeadingHoldReason(
                        false,
                        false,
                        false,
                        true,
                        "none",
                        0.10
                )
        );
    }

    @Test
    public void trajectoryRenderingStaysBlockedWithoutAcceptedStep() {
        assertFalse(MapPointerDisplayFilter.shouldArmTrajectoryRendering(
                false,
                1_000L,
                Long.MIN_VALUE
        ));
    }

    @Test
    public void trajectoryRenderingArmsAfterAcceptedStepInCurrentSession() {
        assertTrue(MapPointerDisplayFilter.shouldArmTrajectoryRendering(
                false,
                1_000L,
                1_250L
        ));
    }

    @Test
    public void trajectoryRenderingIgnoresAcceptedStepFromEarlierSession() {
        assertFalse(MapPointerDisplayFilter.shouldArmTrajectoryRendering(
                false,
                2_000L,
                1_250L
        ));
    }
}
