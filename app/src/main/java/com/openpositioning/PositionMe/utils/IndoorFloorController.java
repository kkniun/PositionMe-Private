package com.openpositioning.PositionMe.utils;

import androidx.annotation.Nullable;

import com.google.android.gms.maps.model.LatLng;

/**
 * Indoor floor transition controller driven by barometric height change and indoor features.
 */
public class IndoorFloorController {

    private static final long FLOOR_CHANGE_DEBOUNCE_MS = 2_500L;
    private static final long TRANSITION_ZONE_LATCH_MS = 6_000L;
    private static final double TRANSITION_ZONE_RADIUS_METERS = 6.0;
    private static final double LIFT_HORIZONTAL_MAX_METERS = 2.0;
    private static final double STAIRS_HORIZONTAL_MIN_METERS = 0.8;
    private static final double WIFI_FLOOR_TOLERANCE = 1.0;
    private static final float FLOOR_CHANGE_TRIGGER_RATIO = 0.38f;
    private static final float MIN_VERTICAL_CHANGE_METERS = 2.4f;
    private static final float ABSOLUTE_ELEVATION_SWITCH_METERS = 4.0f;
    private static final int MAX_EVALUATED_FLOOR_JUMP = 2;

    private final IndoorSpatialConstraintModel spatialModel;

    private float baselineElevation = Float.NaN;
    private LatLng baselineLocation;
    private int baselineLogicalFloor;
    private int lastCandidateFloor = Integer.MIN_VALUE;
    private long lastCandidateSinceMs;
    private long lastTransitionZoneSeenMs;

    public IndoorFloorController(IndoorSpatialConstraintModel spatialModel) {
        this.spatialModel = spatialModel;
    }

    public void reset() {
        baselineElevation = Float.NaN;
        baselineLocation = null;
        baselineLogicalFloor = 0;
        lastCandidateFloor = Integer.MIN_VALUE;
        lastCandidateSinceMs = 0L;
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

        if (Float.isNaN(baselineElevation)) {
            seedBaseline(currentPosition, elevationMeters);
            return null;
        }

        float elevationDelta = elevationMeters - baselineElevation;
        float absoluteElevationDelta = Math.abs(elevationDelta);
        if (absoluteElevationDelta < Math.max(MIN_VERTICAL_CHANGE_METERS * 0.35f, 0.8f)) {
            if (baselineLocation != null
                    && UtilFunctions.distanceBetweenPoints(baselineLocation, currentPosition) > 10.0) {
                seedBaseline(currentPosition, elevationMeters);
            }
            return null;
        }

        boolean absoluteElevationOverride =
                absoluteElevationDelta >= ABSOLUTE_ELEVATION_SWITCH_METERS;
        boolean strongVerticalCue = absoluteElevationOverride
                || absoluteElevationDelta >= Math.max(
                        MIN_VERTICAL_CHANGE_METERS,
                        floorHeight * FLOOR_CHANGE_TRIGGER_RATIO
                );
        if (!strongVerticalCue) {
            return null;
        }

        boolean nearLift = isNearTransitionFeature(currentPosition, "lift");
        boolean nearStairs = isNearTransitionFeature(currentPosition, "stairs");
        if (nearLift || nearStairs) {
            lastTransitionZoneSeenMs = timestampMillis;
        }
        boolean inLatchedTransitionWindow =
                timestampMillis - lastTransitionZoneSeenMs <= TRANSITION_ZONE_LATCH_MS;

        double horizontalMovement = baselineLocation == null
                ? 0d
                : UtilFunctions.distanceBetweenPoints(baselineLocation, currentPosition);

        boolean liftLike = (nearLift || inLatchedTransitionWindow)
                && (elevatorHint || horizontalMovement <= LIFT_HORIZONTAL_MAX_METERS);
        boolean stairsLike = (nearStairs || inLatchedTransitionWindow)
                && horizontalMovement >= STAIRS_HORIZONTAL_MIN_METERS;
        if (!liftLike && !stairsLike && !absoluteElevationOverride) {
            return null;
        }

        int floorStepMagnitude = Math.max(
                1,
                Math.min(
                        MAX_EVALUATED_FLOOR_JUMP,
                        Math.round(absoluteElevationDelta / floorHeight)
                )
        );
        int floorDelta = elevationDelta >= 0f ? floorStepMagnitude : -floorStepMagnitude;
        int candidateFloor = spatialModel.clampLogicalFloor(
                spatialModel.getCurrentBuildingId(),
                baselineLogicalFloor + floorDelta
        );
        if (wifiFloor != null && Math.abs(wifiFloor - candidateFloor) <= WIFI_FLOOR_TOLERANCE) {
            candidateFloor = spatialModel.clampLogicalFloor(
                    spatialModel.getCurrentBuildingId(),
                    wifiFloor
            );
        }
        if (candidateFloor == spatialModel.getCurrentLogicalFloor()) {
            return null;
        }

        if (candidateFloor != lastCandidateFloor) {
            lastCandidateFloor = candidateFloor;
            lastCandidateSinceMs = timestampMillis;
            return null;
        }

        if (timestampMillis - lastCandidateSinceMs < FLOOR_CHANGE_DEBOUNCE_MS) {
            return null;
        }

        spatialModel.setCurrentLogicalFloor(candidateFloor);
        seedBaseline(currentPosition, elevationMeters);
        lastCandidateFloor = Integer.MIN_VALUE;
        lastCandidateSinceMs = 0L;
        lastTransitionZoneSeenMs = timestampMillis;
        return spatialModel.getCurrentLogicalFloor();
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

    private void seedBaseline(LatLng currentPosition, float elevationMeters) {
        baselineElevation = elevationMeters;
        baselineLocation = currentPosition;
        baselineLogicalFloor = spatialModel.getCurrentLogicalFloor();
    }
}
