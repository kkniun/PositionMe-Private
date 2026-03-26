package com.openpositioning.PositionMe.sensors;

/**
 * Relative pedestrian motion increment for one predict step.
 */
public final class PdrDelta {

    private final float stepLengthMeters;
    private final float deltaHeadingRad;
    private final float heightDeltaMeters;
    private final boolean elevatorLikely;

    public PdrDelta(float stepLengthMeters, float deltaHeadingRad, float heightDeltaMeters) {
        this(stepLengthMeters, deltaHeadingRad, heightDeltaMeters, false);
    }

    public PdrDelta(
            float stepLengthMeters,
            float deltaHeadingRad,
            float heightDeltaMeters,
            boolean elevatorLikely
    ) {
        this.stepLengthMeters = stepLengthMeters;
        this.deltaHeadingRad = deltaHeadingRad;
        this.heightDeltaMeters = heightDeltaMeters;
        this.elevatorLikely = elevatorLikely;
    }

    public float getStepLengthMeters() {
        return stepLengthMeters;
    }

    public float getDeltaHeadingRad() {
        return deltaHeadingRad;
    }

    public float getHeightDeltaMeters() {
        return heightDeltaMeters;
    }

    public boolean isElevatorLikely() {
        return elevatorLikely;
    }
}
