package com.openpositioning.PositionMe.sensors;

/**
 * Unified absolute observation used by the fusion pipeline.
 */
public final class AbsoluteFix {

    private final long timestampMs;
    private final double latitudeDeg;
    private final double longitudeDeg;
    private final float accuracyMeters;
    /** True when the fix comes from server WiFi fingerprinting (prone to occasional large errors). */
    private final boolean fromWifiPositioning;

    public AbsoluteFix(long timestampMs, double latitudeDeg, double longitudeDeg, float accuracyMeters) {
        this(timestampMs, latitudeDeg, longitudeDeg, accuracyMeters, false);
    }

    public AbsoluteFix(
            long timestampMs,
            double latitudeDeg,
            double longitudeDeg,
            float accuracyMeters,
            boolean fromWifiPositioning
    ) {
        this.timestampMs = timestampMs;
        this.latitudeDeg = latitudeDeg;
        this.longitudeDeg = longitudeDeg;
        this.accuracyMeters = accuracyMeters;
        this.fromWifiPositioning = fromWifiPositioning;
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

    public boolean isFromWifiPositioning() {
        return fromWifiPositioning;
    }
}
