package com.openpositioning.PositionMe.sensors;

/**
 * Unified absolute observation used by the fusion pipeline.
 */
public final class AbsoluteFix {

    private final long timestampMs;
    private final double latitudeDeg;
    private final double longitudeDeg;
    private final float accuracyMeters;

    public AbsoluteFix(long timestampMs, double latitudeDeg, double longitudeDeg, float accuracyMeters) {
        this.timestampMs = timestampMs;
        this.latitudeDeg = latitudeDeg;
        this.longitudeDeg = longitudeDeg;
        this.accuracyMeters = accuracyMeters;
    }

    public long getTimestampMs() {
        return timestampMs;
    }

    public double getLatitudeDeg() {
        return latitudeDeg;
    }

    public double getLongitudeDeg() {
        return longitudeDeg;
    }

    public float getAccuracyMeters() {
        return accuracyMeters;
    }
}
