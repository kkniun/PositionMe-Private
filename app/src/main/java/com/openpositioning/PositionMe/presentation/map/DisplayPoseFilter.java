package com.openpositioning.PositionMe.presentation.map;

import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.openpositioning.PositionMe.BuildConfig;
import com.google.android.gms.maps.model.LatLng;
import com.openpositioning.PositionMe.utils.UtilFunctions;

import java.util.Locale;

/**
 * UI-only pose smoothing for map display.
 *
 * <p>The filter intentionally leaves fusion, logging, uploads, and raw observations untouched.
 * It only smooths the marker/polyline displayed to the user.</p>
 */
public final class DisplayPoseFilter {
    private static final float HEADING_TURN_RESPONSE_THRESHOLD_DEG = 12.0f;
    private static final float HEADING_TURN_RESPONSE_ALPHA = 0.78f;
    private static final double MOTION_HEADING_MIN_DISTANCE_M = 0.35;
    private static final long MOTION_HEADING_MAX_AGE_MS = 2_500L;

    public static final class Config {
        final double positionDeadbandMeters;
        final double movementResponseDistanceMeters;
        final double snapDistanceMeters;
        final double stationaryMinAlpha;
        final double stationaryMaxAlpha;
        final double movingMinAlpha;
        final double movingMaxAlpha;
        final float headingDeadbandDegrees;
        final float stationaryHeadingAlpha;
        final float movingHeadingAlpha;
        final float headingSnapDegrees;

        public Config(
                double positionDeadbandMeters,
                double movementResponseDistanceMeters,
                double snapDistanceMeters,
                double stationaryMinAlpha,
                double stationaryMaxAlpha,
                double movingMinAlpha,
                double movingMaxAlpha,
                float headingDeadbandDegrees,
                float stationaryHeadingAlpha,
                float movingHeadingAlpha,
                float headingSnapDegrees
        ) {
            this.positionDeadbandMeters = positionDeadbandMeters;
            this.movementResponseDistanceMeters = movementResponseDistanceMeters;
            this.snapDistanceMeters = snapDistanceMeters;
            this.stationaryMinAlpha = stationaryMinAlpha;
            this.stationaryMaxAlpha = stationaryMaxAlpha;
            this.movingMinAlpha = movingMinAlpha;
            this.movingMaxAlpha = movingMaxAlpha;
            this.headingDeadbandDegrees = headingDeadbandDegrees;
            this.stationaryHeadingAlpha = stationaryHeadingAlpha;
            this.movingHeadingAlpha = movingHeadingAlpha;
            this.headingSnapDegrees = headingSnapDegrees;
        }
    }

    public static final class HeadingDebugSnapshot {
        @NonNull public final String headingSource;
        public final float motionHeadingDegrees;
        public final float deviceHeadingDegrees;
        public final float renderedHeadingDegrees;

        HeadingDebugSnapshot(
                @NonNull String headingSource,
                float motionHeadingDegrees,
                float deviceHeadingDegrees,
                float renderedHeadingDegrees
        ) {
            this.headingSource = headingSource;
            this.motionHeadingDegrees = motionHeadingDegrees;
            this.deviceHeadingDegrees = deviceHeadingDegrees;
            this.renderedHeadingDegrees = renderedHeadingDegrees;
        }
    }

    @NonNull
    public static Config defaultMarkerConfig() {
        return new Config(
                0.45,
                2.2,
                6.0,
                0.14,
                0.30,
                0.34,
                0.82,
                1.5f,
                0.28f,
                0.42f,
                24.0f
        );
    }

    @NonNull
    public static Config defaultTrajectoryConfig() {
        return new Config(
                0.30,
                1.6,
                6.0,
                0.12,
                0.24,
                0.24,
                0.56,
                4.0f,
                0.18f,
                0.35f,
                50.0f
        );
    }

    private final Config config;
    @Nullable
    private LatLng filteredPosition;
    private float filteredHeadingDegrees = Float.NaN;
    private float lastMotionHeadingDegrees = Float.NaN;
    private double lastMotionHeadingDistanceMeters = Double.NaN;
    private long lastMotionHeadingTimestampMs = Long.MIN_VALUE;
    private float lastDeviceHeadingDegrees = Float.NaN;
    private float lastRenderedHeadingDegrees = Float.NaN;
    @NonNull
    private String lastHeadingSource = "device";
    @NonNull
    private String lastHeadingLogState = "";
    private long fastFollowUntilTimestampMs = Long.MIN_VALUE;

    public DisplayPoseFilter(@NonNull Config config) {
        this.config = config;
    }

    public void reset() {
        filteredPosition = null;
        filteredHeadingDegrees = Float.NaN;
        lastMotionHeadingDegrees = Float.NaN;
        lastMotionHeadingDistanceMeters = Double.NaN;
        lastMotionHeadingTimestampMs = Long.MIN_VALUE;
        lastDeviceHeadingDegrees = Float.NaN;
        lastRenderedHeadingDegrees = Float.NaN;
        lastHeadingSource = "device";
        lastHeadingLogState = "";
        fastFollowUntilTimestampMs = Long.MIN_VALUE;
    }

    @NonNull
    public LatLng updatePosition(
            @NonNull LatLng rawPosition,
            boolean stationaryHint,
            boolean bypassFilter
    ) {
        return updatePosition(rawPosition, stationaryHint, bypassFilter, Long.MIN_VALUE);
    }

    @NonNull
    public LatLng updatePosition(
            @NonNull LatLng rawPosition,
            boolean stationaryHint,
            boolean bypassFilter,
            long timestampMs
    ) {
        if (!bypassFilter && filteredPosition != null) {
            noteMotionHeading(filteredPosition, rawPosition);
        }
        if (bypassFilter || isFastFollowActive(timestampMs) || filteredPosition == null) {
            filteredPosition = rawPosition;
            return rawPosition;
        }

        double distanceMeters = UtilFunctions.distanceBetweenPoints(filteredPosition, rawPosition);
        if (!Double.isFinite(distanceMeters) || distanceMeters >= config.snapDistanceMeters) {
            filteredPosition = rawPosition;
            return rawPosition;
        }

        if (stationaryHint && distanceMeters <= config.positionDeadbandMeters) {
            return filteredPosition;
        }

        double alpha = resolvePositionAlpha(distanceMeters, stationaryHint);
        filteredPosition = interpolate(filteredPosition, rawPosition, alpha);
        return filteredPosition;
    }

    public void armFastFollowWindow(long timestampMs, long durationMs) {
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        long safeDurationMs = Math.max(0L, durationMs);
        fastFollowUntilTimestampMs = Math.max(
                fastFollowUntilTimestampMs,
                safeTimestampMs + safeDurationMs
        );
    }

    public void clearFastFollowWindow() {
        fastFollowUntilTimestampMs = Long.MIN_VALUE;
    }

    public float updateHeading(
            float rawHeadingDegrees,
            boolean fastResponseHint,
            boolean bypassFilter
    ) {
        return updateHeading(rawHeadingDegrees, fastResponseHint, bypassFilter, null);
    }

    public float updateHeading(
            float rawHeadingDegrees,
            boolean fastResponseHint,
            boolean bypassFilter,
            @Nullable String holdReason
    ) {
        float normalizedRawHeading = normalizeDegrees(rawHeadingDegrees);
        lastDeviceHeadingDegrees = normalizedRawHeading;
        boolean useMotionHeading = hasFreshMotionHeading();
        if (Float.isFinite(filteredHeadingDegrees)
                && shouldFreezeForHoldReason(holdReason, useMotionHeading)) {
            lastHeadingSource = "low_confidence".equals(holdReason)
                    ? "low_confidence_freeze"
                    : "hold_last_stable";
            lastRenderedHeadingDegrees = filteredHeadingDegrees;
            logHeadingDecision(lastHeadingSource, holdReason, normalizedRawHeading, filteredHeadingDegrees);
            return filteredHeadingDegrees;
        }

        float targetHeadingDegrees = useMotionHeading
                ? lastMotionHeadingDegrees
                : normalizedRawHeading;
        lastHeadingSource = useMotionHeading ? "motion" : "device";
        boolean shouldUseFastResponse = fastResponseHint || useMotionHeading;

        if (bypassFilter || !Float.isFinite(filteredHeadingDegrees)) {
            filteredHeadingDegrees = targetHeadingDegrees;
            lastRenderedHeadingDegrees = filteredHeadingDegrees;
            logHeadingDecision(
                    useMotionHeading ? "update_from_motion" : "update_from_device",
                    "none",
                    targetHeadingDegrees,
                    filteredHeadingDegrees
            );
            return filteredHeadingDegrees;
        }

        float diff = shortestAngularDifference(targetHeadingDegrees, filteredHeadingDegrees);
        float absDiff = Math.abs(diff);
        if (absDiff <= config.headingDeadbandDegrees) {
            lastRenderedHeadingDegrees = filteredHeadingDegrees;
            logHeadingDecision(
                    useMotionHeading ? "update_from_motion" : "update_from_device",
                    "deadband",
                    targetHeadingDegrees,
                    filteredHeadingDegrees
            );
            return filteredHeadingDegrees;
        }

        float alpha;
        if (absDiff >= config.headingSnapDegrees) {
            alpha = 1.0f;
        } else if (shouldUseFastResponse) {
            if (absDiff >= HEADING_TURN_RESPONSE_THRESHOLD_DEG) {
                alpha = (float) interpolateAlpha(
                        HEADING_TURN_RESPONSE_ALPHA,
                        0.98f,
                        (absDiff - HEADING_TURN_RESPONSE_THRESHOLD_DEG)
                                / Math.max(
                                config.headingSnapDegrees - HEADING_TURN_RESPONSE_THRESHOLD_DEG,
                                1.0f
                        )
                );
            } else {
                alpha = (float) interpolateAlpha(
                        config.movingHeadingAlpha,
                        HEADING_TURN_RESPONSE_ALPHA,
                        absDiff / HEADING_TURN_RESPONSE_THRESHOLD_DEG
                );
            }
        } else {
            alpha = config.stationaryHeadingAlpha;
        }
        filteredHeadingDegrees = normalizeDegrees(filteredHeadingDegrees + diff * alpha);
        lastRenderedHeadingDegrees = filteredHeadingDegrees;
        logHeadingDecision(
                useMotionHeading ? "update_from_motion" : "update_from_device",
                "none",
                targetHeadingDegrees,
                filteredHeadingDegrees
        );
        return filteredHeadingDegrees;
    }

    @Nullable
    public LatLng getFilteredPosition() {
        return filteredPosition;
    }

    public float getFilteredHeadingDegrees() {
        return filteredHeadingDegrees;
    }

    @NonNull
    public HeadingDebugSnapshot getHeadingDebugSnapshot() {
        return new HeadingDebugSnapshot(
                lastHeadingSource,
                lastMotionHeadingDegrees,
                lastDeviceHeadingDegrees,
                Float.isFinite(lastRenderedHeadingDegrees)
                        ? lastRenderedHeadingDegrees
                        : filteredHeadingDegrees
        );
    }

    private double resolvePositionAlpha(double distanceMeters, boolean stationaryHint) {
        double responseDistance = Math.max(
                config.movementResponseDistanceMeters,
                config.positionDeadbandMeters + 1e-6
        );
        double normalized = (distanceMeters - config.positionDeadbandMeters)
                / (responseDistance - config.positionDeadbandMeters);
        normalized = clamp(normalized, 0.0, 1.0);
        if (stationaryHint) {
            return interpolateAlpha(
                    config.stationaryMinAlpha,
                    config.stationaryMaxAlpha,
                    normalized
            );
        }
        return interpolateAlpha(
                config.movingMinAlpha,
                config.movingMaxAlpha,
                normalized
        );
    }

    private static double interpolateAlpha(double minAlpha, double maxAlpha, double fraction) {
        double clampedFraction = clamp(fraction, 0.0, 1.0);
        return minAlpha + ((maxAlpha - minAlpha) * clampedFraction);
    }

    private boolean isFastFollowActive(long timestampMs) {
        if (fastFollowUntilTimestampMs == Long.MIN_VALUE) {
            return false;
        }
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        return safeTimestampMs <= fastFollowUntilTimestampMs;
    }

    private void noteMotionHeading(@NonNull LatLng from, @NonNull LatLng to) {
        double distanceMeters = UtilFunctions.distanceBetweenPoints(from, to);
        if (!Double.isFinite(distanceMeters)
                || distanceMeters < MOTION_HEADING_MIN_DISTANCE_M
                || distanceMeters >= config.snapDistanceMeters) {
            return;
        }
        lastMotionHeadingDegrees = bearingDegrees(from, to);
        lastMotionHeadingDistanceMeters = distanceMeters;
        lastMotionHeadingTimestampMs = System.currentTimeMillis();
    }

    private boolean hasFreshMotionHeading() {
        if (!Float.isFinite(lastMotionHeadingDegrees) || !Double.isFinite(lastMotionHeadingDistanceMeters)) {
            return false;
        }
        if (lastMotionHeadingTimestampMs == Long.MIN_VALUE) {
            return false;
        }
        long ageMs = Math.max(0L, System.currentTimeMillis() - lastMotionHeadingTimestampMs);
        return ageMs <= MOTION_HEADING_MAX_AGE_MS;
    }

    private boolean hasHoldReason(@Nullable String holdReason) {
        return holdReason != null && !holdReason.isEmpty() && !"none".equals(holdReason);
    }

    private boolean shouldFreezeForHoldReason(
            @Nullable String holdReason,
            boolean useMotionHeading
    ) {
        if (!hasHoldReason(holdReason)) {
            return false;
        }
        return "stationary".equals(holdReason)
                || "elevator".equals(holdReason)
                || "floor_switch".equals(holdReason);
    }

    private void logHeadingDecision(
            @NonNull String decision,
            @NonNull String reason,
            float targetHeadingDegrees,
            float renderedHeadingDegrees
    ) {
        if (!BuildConfig.DEBUG) {
            return;
        }
        String logState = decision + "|" + reason;
        if (logState.equals(lastHeadingLogState)) {
            return;
        }
        lastHeadingLogState = logState;
        try {
            boolean blocked = "hold_last_stable".equals(decision)
                    || "low_confidence_freeze".equals(decision);
            String sourceLabel;
            if (blocked) {
                sourceLabel = "HELD";
            } else if (decision.contains("motion")) {
                sourceLabel = "MOTION";
            } else {
                sourceLabel = "DEVICE";
            }
            float headingDeltaDegrees = Math.abs(shortestAngularDifference(
                    targetHeadingDegrees,
                    renderedHeadingDegrees
            ));
            Log.d(
                    "DisplayPoseFilter",
                    "DISPLAY_HEADING:" + decision
                            + " HEADING_SOURCE=" + sourceLabel
                            + " HEADING_REASON=" + reason
                            + " HEADING_BLOCKED=" + blocked
                            + " targetDeg=" + String.format(Locale.US, "%.1f", targetHeadingDegrees)
                            + " renderedDeg=" + String.format(Locale.US, "%.1f", renderedHeadingDegrees)
                            + " headingDeltaDeg=" + String.format(Locale.US, "%.1f", headingDeltaDegrees)
                            + " motionDistanceM=" + (
                            Double.isFinite(lastMotionHeadingDistanceMeters)
                                    ? String.format(Locale.US, "%.2f", lastMotionHeadingDistanceMeters)
                                    : "n/a"
                    )
            );
        } catch (RuntimeException ignored) {
        }
    }

    @NonNull
    private static LatLng interpolate(@NonNull LatLng from, @NonNull LatLng to, double alpha) {
        double clampedAlpha = clamp(alpha, 0.0, 1.0);
        return new LatLng(
                from.latitude + ((to.latitude - from.latitude) * clampedAlpha),
                from.longitude + ((to.longitude - from.longitude) * clampedAlpha)
        );
    }

    private static double clamp(double value, double minValue, double maxValue) {
        return Math.max(minValue, Math.min(maxValue, value));
    }

    private static float bearingDegrees(@NonNull LatLng from, @NonNull LatLng to) {
        double meanLatitudeRad = Math.toRadians((from.latitude + to.latitude) * 0.5);
        double northMeters = (to.latitude - from.latitude) * 111_320.0;
        double eastMeters = (to.longitude - from.longitude) * 111_320.0 * Math.cos(meanLatitudeRad);
        return normalizeDegrees((float) Math.toDegrees(Math.atan2(eastMeters, northMeters)));
    }

    private static float shortestAngularDifference(float targetDegrees, float currentDegrees) {
        return (targetDegrees - currentDegrees + 180f + 360f) % 360f - 180f;
    }

    private static float normalizeDegrees(float degrees) {
        float normalized = degrees % 360f;
        return normalized < 0f ? normalized + 360f : normalized;
    }
}
