package com.openpositioning.PositionMe.utils;

final class AutoFloorSwitchGate {

    private int pendingFloor = Integer.MIN_VALUE;
    private long pendingSinceElapsedMs = Long.MIN_VALUE;

    boolean shouldApply(int candidateFloor, int currentFloor, long nowElapsedMs, long minStableMs) {
        if (candidateFloor == currentFloor) {
            reset();
            return false;
        }
        if (candidateFloor != pendingFloor) {
            pendingFloor = candidateFloor;
            pendingSinceElapsedMs = nowElapsedMs;
            return false;
        }
        return pendingSinceElapsedMs != Long.MIN_VALUE
                && nowElapsedMs - pendingSinceElapsedMs >= minStableMs;
    }

    void markApplied() {
        reset();
    }

    void reset() {
        pendingFloor = Integer.MIN_VALUE;
        pendingSinceElapsedMs = Long.MIN_VALUE;
    }
}
