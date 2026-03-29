package com.openpositioning.PositionMe.presentation.fragment;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.google.android.gms.maps.model.LatLng;
import com.openpositioning.PositionMe.utils.UtilFunctions;

final class MapPointerDisplayFilter {

    static final double STATIONARY_VISUAL_POSITION_DEADBAND_M = 0.20;
    static final double HEADING_UPDATE_MIN_MOVEMENT_M = 0.25;
    private static final long MARKER_HOLD_NO_RECENT_POSE_AFTER_MS = 2_500L;

    private MapPointerDisplayFilter() {
    }

    static boolean shouldHoldStationaryDisplayPosition(
            @Nullable LatLng currentRawLocation,
            @NonNull LatLng newLocation,
            boolean isStationary
    ) {
        return isStationary
                && currentRawLocation != null
                && UtilFunctions.distanceBetweenPoints(currentRawLocation, newLocation)
                < STATIONARY_VISUAL_POSITION_DEADBAND_M;
    }

    static boolean shouldArmTrajectoryRendering(
            boolean alreadyArmed,
            long sessionStartTimestampMs,
            long lastAcceptedStepTimestampMs
    ) {
        return alreadyArmed
                || (sessionStartTimestampMs > 0L
                && lastAcceptedStepTimestampMs != Long.MIN_VALUE
                && lastAcceptedStepTimestampMs >= sessionStartTimestampMs);
    }

    @NonNull
    static String resolveMarkerHoldReason(
            boolean hasKnownPose,
            boolean elevatorActive,
            boolean floorSwitchActive,
            @NonNull MapMatchingStateResolver.MapMatchingUiState mapState,
            long poseAgeMs
    ) {
        if (!hasKnownPose) {
            return "none";
        }
        if (elevatorActive) {
            return "elevator";
        }
        if (floorSwitchActive) {
            return "floor_switch";
        }
        if (mapState == MapMatchingStateResolver.MapMatchingUiState.PENDING) {
            return "constraints_loading";
        }
        if (poseAgeMs >= MARKER_HOLD_NO_RECENT_POSE_AFTER_MS) {
            return "no_recent_pose";
        }
        return "none";
    }

    @NonNull
    static String resolveHeadingHoldReason(
            boolean markerCreated,
            boolean headingOnlyUpdate,
            boolean stationary,
            boolean lowConfidence,
            @NonNull String markerHoldReason,
            double displayMovementMeters
    ) {
        if (markerCreated) {
            return "none";
        }
        if (!"none".equals(markerHoldReason)) {
            return markerHoldReason;
        }
        if (lowConfidence) {
            return "low_confidence";
        }
        if (stationary) {
            return "stationary";
        }
        if (headingOnlyUpdate) {
            return "awaiting_motion";
        }
        if (Double.isFinite(displayMovementMeters)
                && displayMovementMeters < HEADING_UPDATE_MIN_MOVEMENT_M) {
            return "insufficient_movement";
        }
        return "none";
    }
}
