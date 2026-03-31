package com.openpositioning.PositionMe.utils;

import androidx.annotation.Nullable;

import com.google.android.gms.maps.model.LatLng;

/**
 * Indoor floor transition controller driven by barometric height change and indoor features.
 */
public class IndoorFloorController {

    private static final long FLOOR_CHANGE_DEBOUNCE_MS = 2_500L;
    private static final double TRANSITION_ZONE_RADIUS_METERS = 6.0;
    private static final double LIFT_HORIZONTAL_MAX_METERS = 2.0;
    private static final double STAIRS_HORIZONTAL_MIN_METERS = 1.8;
    private static final double WIFI_FLOOR_TOLERANCE = 1.0;
    private static final float FLOOR_CHANGE_TRIGGER_RATIO = 0.48f;

    private final IndoorSpatialConstraintModel spatialModel;

    private float baselineElevation = Float.NaN;
    private LatLng baselineLocation;
    private int baselineLogicalFloor;
    private int lastCandidateFloor = Integer.MIN_VALUE;
    private long lastCandidateSinceMs;

    public IndoorFloorController(IndoorSpatialConstraintModel spatialModel) {
        this.spatialModel = spatialModel;
    }

    public void reset() {
        baselineElevation = Float.NaN;
        baselineLocation = null;
        baselineLogicalFloor = 0;
        lastCandidateFloor = Integer.MIN_VALUE;
        lastCandidateSinceMs = 0L;
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
        int floorDelta = Math.round(elevationDelta / floorHeight);
        if (floorDelta == 0) {
            if (baselineLocation != null
                    && UtilFunctions.distanceBetweenPoints(baselineLocation, currentPosition) > 10.0) {
                seedBaseline(currentPosition, elevationMeters);
            }
            return null;
        }

        boolean strongVerticalCue = Math.abs(elevationDelta) >= floorHeight * FLOOR_CHANGE_TRIGGER_RATIO;
        if (!strongVerticalCue) {
            return null;
        }

        boolean nearLift = spatialModel.isNearFeature(
                currentPosition,
                "lift",
                TRANSITION_ZONE_RADIUS_METERS
        );
        boolean nearStairs = spatialModel.isNearFeature(
                currentPosition,
                "stairs",
                TRANSITION_ZONE_RADIUS_METERS
        );

        double horizontalMovement = baselineLocation == null
                ? 0d
                : UtilFunctions.distanceBetweenPoints(baselineLocation, currentPosition);

        boolean liftLike = nearLift && (elevatorHint || horizontalMovement <= LIFT_HORIZONTAL_MAX_METERS);
        boolean stairsLike = nearStairs && horizontalMovement >= STAIRS_HORIZONTAL_MIN_METERS;
        if (!liftLike && !stairsLike) {
            return null;
        }

        int candidateFloor = baselineLogicalFloor + floorDelta;
        if (wifiFloor != null && Math.abs(wifiFloor - candidateFloor) <= WIFI_FLOOR_TOLERANCE) {
            candidateFloor = wifiFloor;
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
        return spatialModel.getCurrentLogicalFloor();
    }

    private void seedBaseline(LatLng currentPosition, float elevationMeters) {
        baselineElevation = elevationMeters;
        baselineLocation = currentPosition;
        baselineLogicalFloor = spatialModel.getCurrentLogicalFloor();
    }
}
