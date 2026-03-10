package com.openpositioning.PositionMe.sensors;

/**
 * Minimal immutable runtime output of the particle filter.
 */
public final class FusedPose {

    private final double x;
    private final double y;
    private final int floor;
    private final double confidence;
    private final long timestampMs;

    public FusedPose(double x, double y, int floor, double confidence, long timestampMs) {
        this.x = x;
        this.y = y;
        this.floor = floor;
        this.confidence = confidence;
        this.timestampMs = timestampMs;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public int getFloor() {
        return floor;
    }

    public double getConfidence() {
        return confidence;
    }

    public long getTimestampMs() {
        return timestampMs;
    }
}
