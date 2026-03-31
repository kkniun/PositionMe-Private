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

    private static final long FLOOR_CHANGE_DEBOUNCE_MS = 1_200L;
    private static final float STRONG_ABSOLUTE_SWITCH_METERS = 4.0f;
    private static final float FLOOR_HEIGHT_RATIO_THRESHOLD = 0.72f;
    private static final float RECENTER_ELEVATION_RATIO = 0.18f;
    private static final float RECENTER_ELEVATION_ALPHA = 0.12f;
    private static final int MAX_FLOOR_OFFSET = 3;

    private final IndoorSpatialConstraintModel spatialModel;

    private float anchorElevation = Float.NaN;
    private LatLng anchorLocation;
    private int anchorLogicalFloor;
    private int pendingCandidateFloor = Integer.MIN_VALUE;
    private long pendingCandidateSinceMs;

    public IndoorFloorController(IndoorSpatialConstraintModel spatialModel) {
        this.spatialModel = spatialModel;
    }

    public void reset() {
        anchorElevation = Float.NaN;
        anchorLocation = null;
        anchorLogicalFloor = 0;
        pendingCandidateFloor = Integer.MIN_VALUE;
        pendingCandidateSinceMs = 0L;
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

        float elevationDelta = elevationMeters - anchorElevation;
        int estimatedOffset = estimateFloorOffset(elevationDelta, floorHeight);

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

        if (candidateFloor != pendingCandidateFloor) {
            pendingCandidateFloor = candidateFloor;
            pendingCandidateSinceMs = timestampMillis;
            return null;
        }

        if (timestampMillis - pendingCandidateSinceMs < FLOOR_CHANGE_DEBOUNCE_MS) {
            return null;
        }

        spatialModel.setCurrentLogicalFloor(candidateFloor);
        seedAnchor(currentPosition, elevationMeters, candidateFloor);
        clearPendingCandidate();
        return spatialModel.getCurrentLogicalFloor();
    }

    private int estimateFloorOffset(float elevationDelta, float floorHeight) {
        float absoluteDelta = Math.abs(elevationDelta);
        float floorChangeThreshold = Math.max(
                STRONG_ABSOLUTE_SWITCH_METERS,
                floorHeight * FLOOR_HEIGHT_RATIO_THRESHOLD
        );

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
