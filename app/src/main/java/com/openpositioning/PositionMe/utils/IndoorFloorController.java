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
    private static final float STRONG_ABSOLUTE_SWITCH_METERS = 3.7f;
    private static final float FLOOR_HEIGHT_RATIO_THRESHOLD = 0.78f;
    private static final float RECENTER_ELEVATION_RATIO = 0.18f;
    private static final float RECENTER_ELEVATION_ALPHA = 0.12f;

    private final IndoorSpatialConstraintModel spatialModel;

    private float anchorElevation = Float.NaN;
    private LatLng anchorLocation;
    private int anchorLogicalFloor;
    private boolean anchorConfirmed;
    private int pendingWifiFloor = Integer.MIN_VALUE;
    private long pendingWifiSinceMs;
    private int pendingCandidateFloor = Integer.MIN_VALUE;
    private long pendingCandidateSinceMs;

    public IndoorFloorController(IndoorSpatialConstraintModel spatialModel) {
        this.spatialModel = spatialModel;
    }

    public void reset() {
        anchorElevation = Float.NaN;
        anchorLocation = null;
        anchorLogicalFloor = 0;
        anchorConfirmed = false;
        pendingWifiFloor = Integer.MIN_VALUE;
        pendingWifiSinceMs = 0L;
        pendingCandidateFloor = Integer.MIN_VALUE;
        pendingCandidateSinceMs = 0L;
    }

    public boolean hasConfirmedAnchor() {
        return anchorConfirmed && !Float.isNaN(anchorElevation);
    }

    public void confirmManualFloor(float elevationMeters, int logicalFloor) {
        spatialModel.setCurrentLogicalFloor(logicalFloor);
        anchorLogicalFloor = spatialModel.getCurrentLogicalFloor();
        anchorElevation = elevationMeters;
        anchorLocation = null;
        anchorConfirmed = true;
        clearPendingWifiFloor();
        clearPendingCandidate();
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
            seedAnchor(currentPosition, elevationMeters, spatialModel.getCurrentLogicalFloor());
            return null;
        }

        float elevationDelta = elevationMeters - anchorElevation;
        int estimatedOffset = estimateSingleFloorOffset(elevationDelta, floorHeight);

        if (estimatedOffset == 0) {
            clearPendingCandidate();
            recenterAnchorIfStable(currentPosition, elevationMeters, floorHeight);
            return null;
        }

        int candidateFloor = spatialModel.clampLogicalFloor(
                spatialModel.getCurrentBuildingId(),
                anchorLogicalFloor + Integer.signum(estimatedOffset)
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
            seedAnchor(currentPosition, elevationMeters, spatialModel.getCurrentLogicalFloor());
            anchorConfirmed = true;
            clearPendingWifiFloor();
            clearPendingCandidate();
            return spatialModel.getCurrentLogicalFloor();
        }

        if (Math.abs(elevationMeters - anchorElevation) <= Math.max(1.0f, spatialModel.getCurrentFloorHeight() * 0.25f)) {
            seedAnchor(currentPosition, elevationMeters, spatialModel.getCurrentLogicalFloor());
            anchorConfirmed = true;
        }
        clearPendingWifiFloor();
        return null;
    }

    private int estimateSingleFloorOffset(float elevationDelta, float floorHeight) {
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
        return Integer.signum(roundedOffset);
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

    private void clearPendingWifiFloor() {
        pendingWifiFloor = Integer.MIN_VALUE;
        pendingWifiSinceMs = 0L;
    }

    private void clearPendingCandidate() {
        pendingCandidateFloor = Integer.MIN_VALUE;
        pendingCandidateSinceMs = 0L;
    }
}
