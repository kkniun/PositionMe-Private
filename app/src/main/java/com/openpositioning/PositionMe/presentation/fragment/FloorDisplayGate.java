package com.openpositioning.PositionMe.presentation.fragment;

import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

final class FloorDisplayGate {
    private static final String FLOOR_DIAG_TAG = "FloorDiag";

    interface AutoFloorSyncTarget {
        void syncDisplayedFloor(int floor);
    }

    private FloorDisplayGate() {
    }

    @Nullable
    static Integer resolveTrustedFloorForDisplay(boolean floorCalibrated, int currentFloor) {
        return resolveTrustedFloorForDisplay(floorCalibrated, Integer.valueOf(currentFloor));
    }

    @Nullable
    static Integer resolveTrustedFloorForDisplay(
            boolean floorCalibrated,
            @Nullable Integer currentFloor
    ) {
        if (!floorCalibrated || currentFloor == null) {
            logFloorGateDecision(
                    "event=display_gate_hidden floorCalibrated=" + floorCalibrated
                            + " floor=" + currentFloor
                            + " result=null"
            );
            return null;
        }
        logFloorGateDecision(
                "event=display_gate_show floorCalibrated=true"
                        + " floor=" + currentFloor
                        + " result=" + currentFloor
        );
        return currentFloor;
    }

    private static void logFloorGateDecision(@NonNull String message) {
        try {
            Log.d(FLOOR_DIAG_TAG, message);
        } catch (RuntimeException ignored) {
        }
    }

    static boolean shouldAllowAutoFloorSync(
            boolean autoFloorEnabled,
            boolean floorCalibrated,
            @Nullable Integer currentFloor
    ) {
        return autoFloorEnabled
                && resolveTrustedFloorForDisplay(floorCalibrated, currentFloor) != null;
    }

    static boolean shouldAllowBaselineRecalibration(
            boolean floorCalibrated,
            boolean manualFloorSelection,
            @Nullable Integer floor
    ) {
        return manualFloorSelection
                && resolveTrustedFloorForDisplay(floorCalibrated, floor) != null;
    }

    static boolean shouldAllowUserVisibleFloor(
            boolean floorCalibrated,
            @Nullable Integer floor
    ) {
        return resolveTrustedFloorForDisplay(floorCalibrated, floor) != null;
    }

    @Nullable
    static Integer resolveTrustedFloorForVenueLabel(
            boolean floorCalibrated,
            @Nullable Integer floor
    ) {
        return resolveTrustedFloorForDisplay(floorCalibrated, floor);
    }

    @Nullable
    static Integer resolveDisplayFloor(
            @Nullable Integer trustedFloor,
            @Nullable Integer currentMapFloor,
            @Nullable Integer lastRenderedFloor
    ) {
        if (trustedFloor != null) {
            return trustedFloor;
        }
        if (currentMapFloor != null) {
            return currentMapFloor;
        }
        return lastRenderedFloor;
    }

    @Nullable
    static Integer resolveReplayFloorForDisplay(@Nullable Integer recordedFloor) {
        return recordedFloor;
    }

    static boolean shouldShowUnknownFloorStatus(boolean floorCalibrated) {
        return !floorCalibrated;
    }

    static boolean syncTrustedAutoDisplayedFloor(
            boolean autoFloorEnabled,
            boolean floorCalibrated,
            int currentFloor,
            @Nullable AutoFloorSyncTarget target
    ) {
        return syncTrustedAutoDisplayedFloor(
                autoFloorEnabled,
                floorCalibrated,
                Integer.valueOf(currentFloor),
                target
        );
    }

    static boolean syncTrustedAutoDisplayedFloor(
            boolean autoFloorEnabled,
            boolean floorCalibrated,
            @Nullable Integer currentFloor,
            @Nullable AutoFloorSyncTarget target
    ) {
        if (!shouldAllowAutoFloorSync(autoFloorEnabled, floorCalibrated, currentFloor)
                || target == null) {
            return false;
        }
        Integer trustedFloor = resolveTrustedFloorForDisplay(floorCalibrated, currentFloor);
        if (trustedFloor == null) {
            return false;
        }
        target.syncDisplayedFloor(trustedFloor);
        return true;
    }
}
