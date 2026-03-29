package com.openpositioning.PositionMe.sensors;

final class CumulativeFloorTracker {

    static final float CUMULATIVE_FLOOR_CHANGE_THRESHOLD_M = 2.75f;
    private static final float MIN_FLOOR_CHANGE_RATIO_OF_FLOOR_HEIGHT = 0.75f;
    static final float STRONG_FLOOR_TRANSITION_EVIDENCE_M = 2.75f;
    static final long STRONG_FLOOR_TRANSITION_WINDOW_MS = 10_000L;
    private static final float STRONG_EVIDENCE_REQUIRED_RATIO = 0.95f;
    private static final float ELEVATION_LOW_PASS_ALPHA = 0.85f;
    private static final int FLOOR_CHANGE_CONFIRMATION_SAMPLES = 2;
    private static final long FLOOR_CHANGE_CONFIRMATION_WINDOW_MS = 1_000L;

    private boolean initialized;
    private float anchorElevationMeters;
    private float filteredRelativeElevationMeters;
    private int relativeFloor;
    private float pendingHeightDeltaMeters;
    private long lastTransitionTimestampMs = Long.MIN_VALUE;
    private int lastTransitionDirection;
    private float lastTransitionEvidenceMeters;
    private boolean lastTransitionStrong;
    private int pendingTransitionDirection;
    private int pendingTransitionSampleCount;
    private long pendingTransitionFirstTimestampMs = Long.MIN_VALUE;

    void reset() {
        initialized = false;
        anchorElevationMeters = 0f;
        filteredRelativeElevationMeters = 0f;
        relativeFloor = 0;
        pendingHeightDeltaMeters = 0f;
        lastTransitionTimestampMs = Long.MIN_VALUE;
        lastTransitionDirection = 0;
        lastTransitionEvidenceMeters = 0f;
        lastTransitionStrong = false;
        resetPendingTransition();
    }

    int update(float relativeElevationMeters, float floorHeightMeters, long timestampMs) {
        if (!Float.isFinite(relativeElevationMeters)) {
            return relativeFloor;
        }
        if (!initialized) {
            initialized = true;
            filteredRelativeElevationMeters = relativeElevationMeters;
            anchorElevationMeters = filteredRelativeElevationMeters;
            pendingHeightDeltaMeters = 0f;
            return relativeFloor;
        }
        filteredRelativeElevationMeters = filteredRelativeElevationMeters
                + ELEVATION_LOW_PASS_ALPHA * (relativeElevationMeters - filteredRelativeElevationMeters);
        if (!Float.isFinite(floorHeightMeters) || floorHeightMeters <= 0f) {
            pendingHeightDeltaMeters = filteredRelativeElevationMeters - anchorElevationMeters;
            resetPendingTransition();
            return relativeFloor;
        }

        float requiredTravelMeters = requiredTravelForFloorChange(floorHeightMeters);
        float deltaFromAnchorMeters = filteredRelativeElevationMeters - anchorElevationMeters;
        int transitionDirection = 0;
        float transitionEvidenceMeters = 0f;
        if (deltaFromAnchorMeters >= requiredTravelMeters) {
            transitionDirection = +1;
            transitionEvidenceMeters = deltaFromAnchorMeters;
        } else if (deltaFromAnchorMeters <= -requiredTravelMeters) {
            transitionDirection = -1;
            transitionEvidenceMeters = -deltaFromAnchorMeters;
        } else {
            resetPendingTransition();
        }

        if (transitionDirection != 0) {
            notePendingTransition(transitionDirection, timestampMs);
            if (isPendingTransitionConfirmed(timestampMs)) {
                recordTransition(transitionDirection, transitionEvidenceMeters, floorHeightMeters, timestampMs);
                relativeFloor += transitionDirection;
                anchorElevationMeters += transitionDirection * floorHeightMeters;
                resetPendingTransition();
            }
        }

        deltaFromAnchorMeters = filteredRelativeElevationMeters - anchorElevationMeters;
        pendingHeightDeltaMeters = deltaFromAnchorMeters;
        return relativeFloor;
    }

    int getRelativeFloor() {
        return relativeFloor;
    }

    float getPendingHeightDeltaMeters() {
        return pendingHeightDeltaMeters;
    }

    float getFilteredRelativeElevationMeters() {
        return filteredRelativeElevationMeters;
    }

    boolean isInitialized() {
        return initialized;
    }

    boolean clampRelativeFloor(int minRelativeFloor, int maxRelativeFloor) {
        if (minRelativeFloor > maxRelativeFloor) {
            int swap = minRelativeFloor;
            minRelativeFloor = maxRelativeFloor;
            maxRelativeFloor = swap;
        }
        int clampedRelativeFloor = Math.max(minRelativeFloor, Math.min(maxRelativeFloor, relativeFloor));
        if (clampedRelativeFloor == relativeFloor) {
            return false;
        }
        relativeFloor = clampedRelativeFloor;
        anchorElevationMeters = filteredRelativeElevationMeters;
        pendingHeightDeltaMeters = 0f;
        lastTransitionTimestampMs = Long.MIN_VALUE;
        lastTransitionDirection = 0;
        lastTransitionEvidenceMeters = 0f;
        lastTransitionStrong = false;
        resetPendingTransition();
        return true;
    }

    void reanchorToCurrentElevation(float relativeElevationMeters) {
        if (!Float.isFinite(relativeElevationMeters)) {
            reset();
            return;
        }
        initialized = true;
        anchorElevationMeters = relativeElevationMeters;
        filteredRelativeElevationMeters = relativeElevationMeters;
        relativeFloor = 0;
        pendingHeightDeltaMeters = 0f;
        lastTransitionTimestampMs = Long.MIN_VALUE;
        lastTransitionDirection = 0;
        lastTransitionEvidenceMeters = 0f;
        lastTransitionStrong = false;
        resetPendingTransition();
    }

    boolean hasRecentStrongTransitionEvidence(int previousFloor, int newFloor, long nowMs) {
        if (!hasRecentTransition(previousFloor, newFloor, nowMs)) {
            return false;
        }
        return lastTransitionStrong;
    }

    boolean hasRecentTransition(int previousFloor, int newFloor, long nowMs) {
        if (newFloor == previousFloor || lastTransitionTimestampMs == Long.MIN_VALUE) {
            return false;
        }
        int requestedDirection = Integer.signum(newFloor - previousFloor);
        if (requestedDirection == 0 || requestedDirection != lastTransitionDirection) {
            return false;
        }
        if (nowMs - lastTransitionTimestampMs > STRONG_FLOOR_TRANSITION_WINDOW_MS) {
            return false;
        }
        return true;
    }

    static float requiredTravelForFloorChange(float floorHeightMeters) {
        if (!Float.isFinite(floorHeightMeters) || floorHeightMeters <= 0f) {
            return Float.POSITIVE_INFINITY;
        }
        return Math.max(
                CUMULATIVE_FLOOR_CHANGE_THRESHOLD_M,
                floorHeightMeters * MIN_FLOOR_CHANGE_RATIO_OF_FLOOR_HEIGHT
        );
    }

    static float strongEvidenceThresholdForFloorChange(float floorHeightMeters) {
        float requiredTravelMeters = requiredTravelForFloorChange(floorHeightMeters);
        if (!Float.isFinite(requiredTravelMeters)) {
            return STRONG_FLOOR_TRANSITION_EVIDENCE_M;
        }
        return Math.max(
                STRONG_FLOOR_TRANSITION_EVIDENCE_M,
                requiredTravelMeters * STRONG_EVIDENCE_REQUIRED_RATIO
        );
    }

    private void recordTransition(int direction, float evidenceMeters, float floorHeightMeters, long timestampMs) {
        lastTransitionDirection = direction;
        lastTransitionEvidenceMeters = evidenceMeters;
        lastTransitionTimestampMs = timestampMs;
        lastTransitionStrong = evidenceMeters >= strongEvidenceThresholdForFloorChange(floorHeightMeters);
    }

    private void notePendingTransition(int direction, long timestampMs) {
        if (pendingTransitionDirection != direction) {
            pendingTransitionDirection = direction;
            pendingTransitionSampleCount = 1;
            pendingTransitionFirstTimestampMs = timestampMs;
            return;
        }
        pendingTransitionSampleCount++;
    }

    private boolean isPendingTransitionConfirmed(long timestampMs) {
        if (pendingTransitionDirection == 0
                || pendingTransitionSampleCount < FLOOR_CHANGE_CONFIRMATION_SAMPLES
                || pendingTransitionFirstTimestampMs == Long.MIN_VALUE) {
            return false;
        }
        return timestampMs - pendingTransitionFirstTimestampMs >= FLOOR_CHANGE_CONFIRMATION_WINDOW_MS;
    }

    private void resetPendingTransition() {
        pendingTransitionDirection = 0;
        pendingTransitionSampleCount = 0;
        pendingTransitionFirstTimestampMs = Long.MIN_VALUE;
    }
}
