package com.openpositioning.PositionMe.utils;

import androidx.annotation.Nullable;
import androidx.annotation.NonNull;

import com.google.android.gms.maps.model.LatLng;

/**
 * Indoor floor transition controller driven by WiFi floor seeding and elevation-led switching.
 *
 * <p>WiFi is used only to confirm the initial floor anchor (or a manual re-anchor). Once the
 * anchor exists, automatic switching is controlled by cumulative elevation change relative to the
 * last confirmed floor. This keeps the start floor accurate without allowing later WiFi jitter to
 * drag the user back to the wrong level.</p>
 */
public class IndoorFloorController {

    private static final long WIFI_FLOOR_CONFIRM_MS = 1_100L;
    private static final long FLOOR_CHANGE_DEBOUNCE_MS = 900L;
    private static final float TRANSITION_WINDOW_START_RATIO = 0.30f;
    private static final float TRANSITION_WINDOW_START_MIN_METERS = 1.1f;
    private static final double STAIRS_PROXIMITY_METERS = 4.0;
    private static final double LIFT_PROXIMITY_METERS = 3.5;
    private final IndoorSpatialConstraintModel spatialModel;

    private float anchorElevation = Float.NaN;
    private boolean anchorConfirmed;
    @Nullable
    private LatLng anchorPosition;
    private long anchorTimestampMs;
    private int pendingWifiFloor = Integer.MIN_VALUE;
    private long pendingWifiSinceMs;
    private int pendingCandidateFloor = Integer.MIN_VALUE;
    private long pendingCandidateSinceMs;
    private FloorTransitionHeuristics.Mode pendingCandidateMode = FloorTransitionHeuristics.Mode.NONE;
    @Nullable
    private LatLng transitionStartPosition;
    private long transitionStartTimestampMs;

    public IndoorFloorController(IndoorSpatialConstraintModel spatialModel) {
        this.spatialModel = spatialModel;
    }

    public void reset() {
        anchorElevation = Float.NaN;
        anchorConfirmed = false;
        anchorPosition = null;
        anchorTimestampMs = 0L;
        pendingWifiFloor = Integer.MIN_VALUE;
        pendingWifiSinceMs = 0L;
        pendingCandidateFloor = Integer.MIN_VALUE;
        pendingCandidateSinceMs = 0L;
        pendingCandidateMode = FloorTransitionHeuristics.Mode.NONE;
        clearTransitionWindow();
    }

    public boolean hasConfirmedAnchor() {
        return anchorConfirmed && !Float.isNaN(anchorElevation);
    }

    public void confirmManualFloor(float elevationMeters, int logicalFloor) {
        spatialModel.setCurrentLogicalFloor(logicalFloor);
        anchorElevation = elevationMeters;
        anchorConfirmed = true;
        clearPendingWifiFloor();
        clearPendingCandidate();
        clearTransitionWindow();
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

        Integer normalizedWifiFloor = wifiFloor == null
                ? null
                : spatialModel.normalizeExternalFloorObservation(wifiFloor);
        Integer seededFloor = maybeSeedFloorFromWifi(
                normalizedWifiFloor,
                currentPosition,
                elevationMeters,
                timestampMillis
        );
        if (seededFloor != null) {
            return seededFloor;
        }

        if (!anchorConfirmed) {
            return null;
        }

        int currentLogicalFloor = spatialModel.getCurrentLogicalFloor();
        float elevationDelta = elevationMeters - anchorElevation;
        updateTransitionWindow(currentPosition, elevationMeters, floorHeight, timestampMillis);
        FloorTransitionHeuristics.Decision decision = resolveNextFloor(
                currentLogicalFloor,
                elevationDelta,
                floorHeight,
                elevatorHint,
                normalizedWifiFloor,
                currentPosition,
                timestampMillis
        );
        int candidateFloor = spatialModel.clampLogicalFloor(
                spatialModel.getCurrentBuildingId(),
                decision.getTargetFloor()
        );
        if (candidateFloor == currentLogicalFloor) {
            clearPendingCandidate();
            return null;
        }

        if (candidateFloor != pendingCandidateFloor || decision.getMode() != pendingCandidateMode) {
            pendingCandidateFloor = candidateFloor;
            pendingCandidateSinceMs = timestampMillis;
            pendingCandidateMode = decision.getMode();
            return null;
        }

        if (timestampMillis - pendingCandidateSinceMs < FLOOR_CHANGE_DEBOUNCE_MS) {
            return null;
        }

        spatialModel.setCurrentLogicalFloor(candidateFloor);
        seedAnchor(currentPosition, elevationMeters, timestampMillis);
        clearPendingCandidate();
        return spatialModel.getCurrentLogicalFloor();
    }

    @Nullable
    private Integer maybeSeedFloorFromWifi(@Nullable Integer normalizedWifiFloor,
                                           @NonNull LatLng currentPosition,
                                           float elevationMeters,
                                           long timestampMillis) {
        if (normalizedWifiFloor == null) {
            clearPendingWifiFloor();
            return null;
        }

        if (anchorConfirmed && spatialModel.getCurrentLogicalFloor() != normalizedWifiFloor) {
            clearPendingWifiFloor();
            return null;
        }

        if (normalizedWifiFloor != pendingWifiFloor) {
            pendingWifiFloor = normalizedWifiFloor;
            pendingWifiSinceMs = timestampMillis;
            return null;
        }

        if (!anchorConfirmed && timestampMillis - pendingWifiSinceMs < WIFI_FLOOR_CONFIRM_MS) {
            return null;
        }

        if (!anchorConfirmed) {
            spatialModel.setCurrentLogicalFloor(normalizedWifiFloor);
            seedAnchor(currentPosition, elevationMeters, timestampMillis);
            anchorConfirmed = true;
            clearPendingWifiFloor();
            clearPendingCandidate();
            return spatialModel.getCurrentLogicalFloor();
        }

        clearPendingWifiFloor();
        return null;
    }

    private FloorTransitionHeuristics.Decision resolveNextFloor(int currentLogicalFloor,
                                                                float elevationDelta,
                                                                float floorHeight,
                                                                boolean elevatorHint,
                                                                @Nullable Integer normalizedWifiFloor,
                                                                @NonNull LatLng currentPosition,
                                                                long timestampMillis) {
        int direction = elevationDelta >= 0f ? 1 : -1;
        int adjacentFloor = clampRelativeFloor(currentLogicalFloor, direction);
        int estimatedLiftTargetFloor = clampRelativeFloor(
                currentLogicalFloor,
                direction * estimateMovedFloors(elevationDelta, floorHeight)
        );
        boolean nearStairs = isNearTransitionFeature(
                currentPosition,
                "stairs",
                STAIRS_PROXIMITY_METERS,
                currentLogicalFloor,
                adjacentFloor
        );
        boolean nearLift = isNearTransitionFeature(
                currentPosition,
                "lift",
                LIFT_PROXIMITY_METERS,
                currentLogicalFloor,
                adjacentFloor,
                estimatedLiftTargetFloor
        );
        double horizontalTravel = getTransitionHorizontalTravelMeters(currentPosition);
        long transitionDurationMs = getTransitionDurationMillis(timestampMillis);
        FloorTransitionHeuristics.Decision decision = FloorTransitionHeuristics.evaluate(
                currentLogicalFloor,
                elevationDelta,
                floorHeight,
                horizontalTravel,
                transitionDurationMs,
                nearStairs,
                nearLift,
                elevatorHint,
                normalizedWifiFloor
        );
        if (!decision.shouldTransition(currentLogicalFloor)) {
            return decision;
        }

        int candidateFloor = spatialModel.clampLogicalFloor(
                spatialModel.getCurrentBuildingId(),
                decision.getTargetFloor()
        );
        if (!hasSemanticSupportForDecision(
                currentPosition,
                decision.getMode(),
                currentLogicalFloor,
                candidateFloor
        )) {
            return new FloorTransitionHeuristics.Decision(
                    currentLogicalFloor,
                    FloorTransitionHeuristics.Mode.NONE
            );
        }
        return new FloorTransitionHeuristics.Decision(candidateFloor, decision.getMode());
    }

    private void seedAnchor(LatLng currentPosition,
                            float elevationMeters,
                            long timestampMillis) {
        anchorElevation = elevationMeters;
        anchorPosition = currentPosition;
        anchorTimestampMs = timestampMillis;
        clearTransitionWindow();
    }

    private void clearPendingWifiFloor() {
        pendingWifiFloor = Integer.MIN_VALUE;
        pendingWifiSinceMs = 0L;
    }

    private void clearPendingCandidate() {
        pendingCandidateFloor = Integer.MIN_VALUE;
        pendingCandidateSinceMs = 0L;
        pendingCandidateMode = FloorTransitionHeuristics.Mode.NONE;
    }

    private void updateTransitionWindow(@NonNull LatLng currentPosition,
                                        float elevationMeters,
                                        float floorHeight,
                                        long timestampMillis) {
        float activationThreshold = Math.max(
                TRANSITION_WINDOW_START_MIN_METERS,
                floorHeight * TRANSITION_WINDOW_START_RATIO
        );
        float absoluteDelta = Math.abs(elevationMeters - anchorElevation);
        if (absoluteDelta < activationThreshold) {
            if (pendingCandidateFloor == Integer.MIN_VALUE) {
                clearTransitionWindow();
            }
            return;
        }

        if (transitionStartPosition == null) {
            transitionStartPosition = currentPosition;
            transitionStartTimestampMs = timestampMillis;
        }
    }

    private void clearTransitionWindow() {
        transitionStartPosition = null;
        transitionStartTimestampMs = 0L;
    }

    private double getTransitionHorizontalTravelMeters(@NonNull LatLng currentPosition) {
        if (transitionStartPosition != null) {
            return UtilFunctions.distanceBetweenPoints(transitionStartPosition, currentPosition);
        }
        if (anchorPosition != null) {
            return UtilFunctions.distanceBetweenPoints(anchorPosition, currentPosition);
        }
        return 0d;
    }

    private long getTransitionDurationMillis(long timestampMillis) {
        if (transitionStartTimestampMs > 0L) {
            return Math.max(0L, timestampMillis - transitionStartTimestampMs);
        }
        if (anchorTimestampMs > 0L) {
            return Math.max(0L, timestampMillis - anchorTimestampMs);
        }
        return 0L;
    }

    private int estimateMovedFloors(float elevationDelta, float floorHeight) {
        float effectiveFloorHeight = Math.max(1.0f, floorHeight);
        return Math.max(1, Math.round(Math.abs(elevationDelta) / effectiveFloorHeight));
    }

    private int clampRelativeFloor(int baseFloor, int deltaFloors) {
        return spatialModel.clampLogicalFloor(
                spatialModel.getCurrentBuildingId(),
                baseFloor + deltaFloors
        );
    }

    private boolean hasSemanticSupportForDecision(@NonNull LatLng point,
                                                  @NonNull FloorTransitionHeuristics.Mode mode,
                                                  int currentFloor,
                                                  int candidateFloor) {
        if (mode == FloorTransitionHeuristics.Mode.NONE || candidateFloor == currentFloor) {
            return false;
        }
        if (mode == FloorTransitionHeuristics.Mode.STAIRS
                && Math.abs(candidateFloor - currentFloor) != 1) {
            return false;
        }

        String indoorType = mode == FloorTransitionHeuristics.Mode.LIFT ? "lift" : "stairs";
        double radiusMeters = mode == FloorTransitionHeuristics.Mode.LIFT
                ? LIFT_PROXIMITY_METERS
                : STAIRS_PROXIMITY_METERS;
        return isNearTransitionFeature(point, indoorType, radiusMeters, currentFloor, candidateFloor);
    }

    private boolean isNearTransitionFeature(@NonNull LatLng point,
                                            @NonNull String indoorType,
                                            double radiusMeters,
                                            int... logicalFloors) {
        String buildingId = spatialModel.getCurrentBuildingId();
        for (int logicalFloor : logicalFloors) {
            int clampedFloor = spatialModel.clampLogicalFloor(buildingId, logicalFloor);
            if (spatialModel.isNearFeature(point, indoorType, radiusMeters, buildingId, clampedFloor)) {
                return true;
            }
        }
        return false;
    }
}
