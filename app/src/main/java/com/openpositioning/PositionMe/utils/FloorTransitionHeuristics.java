package com.openpositioning.PositionMe.utils;

import androidx.annotation.Nullable;

/**
 * 负责楼层切换的纯逻辑判定，便于单元测试。
 */
final class FloorTransitionHeuristics {

    private static final float STRONG_ABSOLUTE_SWITCH_METERS = 3.7f;
    private static final float FLOOR_HEIGHT_RATIO_THRESHOLD = 0.78f;
    private static final double STAIRS_MIN_HORIZONTAL_METERS = 1.8;
    private static final double STAIRS_MAX_VERTICAL_RATE_MPS = 2.8;
    private static final double LIFT_MAX_HORIZONTAL_METERS = 2.0;
    private static final double LIFT_MIN_VERTICAL_RATE_MPS = 0.9;

    enum Mode {
        NONE,
        STAIRS,
        LIFT
    }

    static final class Decision {
        private final int targetFloor;
        private final Mode mode;

        Decision(int targetFloor, Mode mode) {
            this.targetFloor = targetFloor;
            this.mode = mode;
        }

        int getTargetFloor() {
            return targetFloor;
        }

        Mode getMode() {
            return mode;
        }

        boolean shouldTransition(int currentFloor) {
            return mode != Mode.NONE && targetFloor != currentFloor;
        }
    }

    private FloorTransitionHeuristics() {
    }

    static Decision evaluate(int currentFloor,
                             float elevationDeltaMeters,
                             float floorHeightMeters,
                             double horizontalTravelMeters,
                             long transitionDurationMs,
                             boolean nearStairs,
                             boolean nearLift,
                             boolean elevatorHint,
                             @Nullable Integer wifiFloorHint) {
        float effectiveFloorHeight = Math.max(1.0f, floorHeightMeters);
        float floorChangeThreshold = Math.max(
                STRONG_ABSOLUTE_SWITCH_METERS,
                effectiveFloorHeight * FLOOR_HEIGHT_RATIO_THRESHOLD
        );
        float absoluteDelta = Math.abs(elevationDeltaMeters);
        if (absoluteDelta < floorChangeThreshold) {
            return new Decision(currentFloor, Mode.NONE);
        }

        int direction = elevationDeltaMeters > 0f ? 1 : -1;
        int rawMovedFloors = Math.max(1, Math.round(absoluteDelta / effectiveFloorHeight));
        double durationSeconds = Math.max(0.35d, transitionDurationMs / 1000d);
        double verticalRate = absoluteDelta / durationSeconds;

        boolean stairsLike = nearStairs
                && horizontalTravelMeters >= STAIRS_MIN_HORIZONTAL_METERS
                && verticalRate <= STAIRS_MAX_VERTICAL_RATE_MPS;
        boolean liftLike = nearLift
                && horizontalTravelMeters <= LIFT_MAX_HORIZONTAL_METERS
                && (elevatorHint || rawMovedFloors > 1 || verticalRate >= LIFT_MIN_VERTICAL_RATE_MPS);

        Mode resolvedMode = Mode.NONE;
        if (stairsLike && liftLike) {
            resolvedMode = elevatorHint && horizontalTravelMeters <= (LIFT_MAX_HORIZONTAL_METERS * 0.8d)
                    ? Mode.LIFT
                    : Mode.STAIRS;
        } else if (liftLike) {
            resolvedMode = Mode.LIFT;
        } else if (stairsLike) {
            resolvedMode = Mode.STAIRS;
        }

        if (resolvedMode == Mode.NONE) {
            return new Decision(currentFloor, Mode.NONE);
        }

        int movedFloors = resolvedMode == Mode.LIFT ? rawMovedFloors : 1;
        int targetFloor = currentFloor + direction * movedFloors;
        if (wifiFloorHint != null) {
            boolean wifiConsistent = Math.abs(wifiFloorHint - targetFloor) <= 1;
            if (wifiConsistent) {
                targetFloor = resolvedMode == Mode.STAIRS
                        ? currentFloor + direction
                        : wifiFloorHint;
            }
        }
        return new Decision(targetFloor, resolvedMode);
    }
}
