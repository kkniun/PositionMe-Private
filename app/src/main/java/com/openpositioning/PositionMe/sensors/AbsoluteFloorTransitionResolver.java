package com.openpositioning.PositionMe.sensors;

import androidx.annotation.Nullable;

/**
 * Resolves whether an absolute-fix floor report should be accepted.
 *
 * <p>Policy:
 * 1. Accept immediately before PF initialization.
 * 2. Accept immediately when the reported floor matches the current floor.
 * 3. When transition constraints exist, only accept if the transition gate allows it.
 * 4. Without transition constraints, require repeated consistent reports before switching floors.
 */
final class AbsoluteFloorTransitionResolver {

    private static final int REQUIRED_SOFT_CONFIRMATIONS = 2;

    @Nullable
    private Integer pendingFloorCandidate;
    private int pendingFloorCandidateCount;

    void reset() {
        pendingFloorCandidate = null;
        pendingFloorCandidateCount = 0;
    }

    @Nullable
    Integer resolveAcceptedFloor(
            int currentFloor,
            @Nullable Integer reportedFloor,
            boolean pfInitialized,
            boolean hasTransitionConstraints,
            boolean transitionAllowed
    ) {
        if (reportedFloor == null) {
            reset();
            return null;
        }
        if (!pfInitialized || reportedFloor == currentFloor) {
            reset();
            return reportedFloor;
        }
        if (hasTransitionConstraints) {
            reset();
            return transitionAllowed ? reportedFloor : null;
        }
        if (reportedFloor.equals(pendingFloorCandidate)) {
            pendingFloorCandidateCount++;
        } else {
            pendingFloorCandidate = reportedFloor;
            pendingFloorCandidateCount = 1;
        }
        if (pendingFloorCandidateCount >= REQUIRED_SOFT_CONFIRMATIONS) {
            reset();
            return reportedFloor;
        }
        return null;
    }
}
