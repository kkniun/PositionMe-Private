package com.openpositioning.PositionMe.utils;

import androidx.annotation.Nullable;

import com.google.android.gms.maps.model.LatLng;

/**
 * Indoor floor transition controller driven primarily by cumulative elevation change.
 *
 * <p>The current confirmed logical floor acts as an elevation anchor. Once the user is anchored
 * to a floor, subsequent barometric height deltas are interpreted relative to that floor:
 * approximately one floor-height down means the next lower floor, two floor-heights down means
 * two floors lower, and climbing back up toward the anchor elevation returns to the anchor floor.</p>
 */
public class IndoorFloorController {

    private static final long FLOOR_CHANGE_DEBOUNCE_MS = 1_400L;
    private static final long TRANSITION_ZONE_LATCH_MS = 8_000L;
    private static final double TRANSITION_ZONE_RADIUS_METERS = 6.0;
    private static final float STRONG_ABSOLUTE_SWITCH_METERS = 4.0f;
    private static final float RECENTER_ELEVATION_RATIO = 0.22f;
    private static final float RECENTER_ELEVATION_ALPHA = 0.18f;
    private static final int MAX_FLOOR_OFFSET = 3;

    private final IndoorSpatialConstraintModel spatialModel;

    private float anchorElevation = Float.NaN;
    private LatLng anchorLocation;
    private int anchorLogicalFloor;
    private int pendingCandidateFloor = Integer.MIN_VALUE;
    private long pendingCandidateSinceMs;
    private long lastTransitionZoneSeenMs;

    public IndoorFloorController(IndoorSpatialConstraintModel spatialModel) {
        this.spatialModel = spatialModel;
    }

    public void reset() {
        anchorElevation = Float.NaN;
        anchorLocation = null;
        anchorLogicalFloor = 0;
        pendingCandidateFloor = Integer.MIN_VALUE;
        pendingCandidateSinceMs = 0L;
        lastTransitionZoneSeenMs = 0L;
    }

    @Nullable
    public Integer evaluate(@Nullable LatLng currentPosition,
                            float elevationMeters,
                            @Nullable Integer wifiFloor,
                            boolean elevatorHint,
                            long timestampMillis) {
        if (currentPosition == null) {
            return null;
        }

        spatialModel.updatePosition(currentPosition);
        if (!spatialModel.hasIndoorContext()) {
            return null;
        }

        float floorHeight = spatialModel.getCurrentFloorHeight();
        if (floorHeight <= 0f) {
            return null;
        }

        int currentLogicalFloor = spatialModel.getCurrentLogicalFloor();
        if (Float.isNaN(anchorElevation)) {
            seedAnchor(currentPosition, elevationMeters, currentLogicalFloor);
            return null;
        }

        boolean nearLift = isNearTransitionFeature(currentPosition, "lift");
        boolean nearStairs = isNearTransitionFeature(currentPosition, "stairs");
        if (nearLift || nearStairs || elevatorHint) {
            lastTransitionZoneSeenMs = timestampMillis;
        }
        boolean transitionSupported =
                timestampMillis - lastTransitionZoneSeenMs <= TRANSITION_ZONE_LATCH_MS;

        float elevationDelta = elevationMeters - anchorElevation;
        int estimatedOffset = estimateFloorOffset(
                elevationDelta,
                floorHeight,
                transitionSupported || elevatorHint
        );

        if (estimatedOffset == 0) {
            clearPendingCandidate();
            recenterAnchorIfStable(currentPosition, elevationMeters, floorHeight);
            return null;
        }

        int candidateFloor = spatialModel.clampLogicalFloor(
                spatialModel.getCurrentBuildingId(),
                anchorLogicalFloor + estimatedOffset
        );

        if (candidateFloor == currentLogicalFloor) {
            clearPendingCandidate();
            recenterAnchorIfStable(currentPosition, elevationMeters, floorHeight);
            return null;
        }

        if (wifiFloor != null && wifiFloor == candidateFloor) {
            transitionSupported = true;
        }

        boolean strongAbsoluteCue = Math.abs(elevationDelta) >= STRONG_ABSOLUTE_SWITCH_METERS;
        if (!transitionSupported && !strongAbsoluteCue) {
            clearPendingCandidate();
            return null;
        }

        if (candidateFloor != pendingCandidateFloor) {
            pendingCandidateFloor = candidateFloor;
            pendingCandidateSinceMs = timestampMillis;
            return null;
        }

        long debounceMs = wifiFloor != null && wifiFloor == candidateFloor
                ? FLOOR_CHANGE_DEBOUNCE_MS / 2
                : FLOOR_CHANGE_DEBOUNCE_MS;
        if (timestampMillis - pendingCandidateSinceMs < debounceMs) {
            return null;
        }

        spatialModel.setCurrentLogicalFloor(candidateFloor);
        seedAnchor(currentPosition, elevationMeters, candidateFloor);
        clearPendingCandidate();
        lastTransitionZoneSeenMs = timestampMillis;
        return spatialModel.getCurrentLogicalFloor();
    }

    private int estimateFloorOffset(float elevationDelta,
                                    float floorHeight,
                                    boolean transitionSupported) {
        float absoluteDelta = Math.abs(elevationDelta);
        float floorChangeThreshold = transitionSupported
                ? Math.max(2.6f, floorHeight * 0.64f)
                : Math.max(3.2f, floorHeight * 0.82f);

        if (absoluteDelta < floorChangeThreshold) {
            return 0;
        }

        int roundedOffset = Math.round(elevationDelta / floorHeight);
        if (roundedOffset == 0) {
            roundedOffset = elevationDelta > 0f ? 1 : -1;
        }
        roundedOffset = Math.max(-MAX_FLOOR_OFFSET, Math.min(MAX_FLOOR_OFFSET, roundedOffset));
        return roundedOffset;
    }

    private void recenterAnchorIfStable(LatLng currentPosition,
                                        float elevationMeters,
                                        float floorHeight) {
        if (Float.isNaN(anchorElevation)) {
            return;
        }
        float elevationDelta = elevationMeters - anchorElevation;
        if (Math.abs(elevationDelta) > Math.max(0.9f, floorHeight * RECENTER_ELEVATION_RATIO)) {
            return;
        }
        anchorElevation += elevationDelta * RECENTER_ELEVATION_ALPHA;
        anchorLocation = currentPosition;
        anchorLogicalFloor = spatialModel.getCurrentLogicalFloor();
    }

    private boolean isNearTransitionFeature(LatLng currentPosition, String indoorType) {
        String buildingId = spatialModel.getCurrentBuildingId();
        int currentFloor = spatialModel.getCurrentLogicalFloor();
        for (int floor = currentFloor - 1; floor <= currentFloor + 1; floor++) {
            if (spatialModel.isNearFeature(
                    currentPosition,
                    indoorType,
                    TRANSITION_ZONE_RADIUS_METERS,
                    buildingId,
                    floor
            )) {
                return true;
            }
        }
        return false;
    }

    private void seedAnchor(LatLng currentPosition, float elevationMeters, int logicalFloor) {
        anchorElevation = elevationMeters;
        anchorLocation = currentPosition;
        anchorLogicalFloor = logicalFloor;
    }

    private void clearPendingCandidate() {
        pendingCandidateFloor = Integer.MIN_VALUE;
        pendingCandidateSinceMs = 0L;
    }
}
