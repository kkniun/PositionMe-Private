package com.openpositioning.PositionMe.sensors;

/**
 * Relative pedestrian motion increment for one predict step.
 */
public final class PdrDelta {

    private final float stepLengthMeters;
    private final float deltaHeadingRad;
    private final float heightDeltaMeters;

    public PdrDelta(float stepLengthMeters, float deltaHeadingRad, float heightDeltaMeters) {
        this.stepLengthMeters = stepLengthMeters;
        this.deltaHeadingRad = deltaHeadingRad;
        this.heightDeltaMeters = heightDeltaMeters;
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
}
