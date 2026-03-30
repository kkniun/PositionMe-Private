package com.openpositioning.PositionMe.sensors;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

/**
 * Absolute fix 的最小策略集合：
 * 1. 统一不同来源的 accuracy 输入；
 * 2. 把 observation age 显式纳入普通融合路径；
 * 3. 控制 stationary / rollback 这些附加 gate 的保守程度。
 */
final class AbsoluteFixFusionPolicy {

    static final float WIFI_FALLBACK_ACCURACY_M = 5.0f;
    static final float GENERIC_ABSOLUTE_FALLBACK_ACCURACY_M = 4.0f;
    static final float MIN_NORMALIZED_ACCURACY_M = 1.0f;
    static final float MAX_NORMALIZED_ACCURACY_M = 25.0f;

    static final long FRESH_OBSERVATION_MAX_AGE_MS = 3_000L;
    static final long STALE_REJECT_AGE_MS = 15_000L;
    static final float MAX_STALE_ACCURACY_INFLATION_RATIO = 2.0f;

    private static final long STATIONARY_CORRECTION_MAX_AGE_MS = 3_000L;
    private static final float STATIONARY_CORRECTION_MAX_ACCURACY_M = 4.0f;

    private AbsoluteFixFusionPolicy() {
    }

    static final class AccuracyResolution {
        final float rawAccuracyMeters;
        final float normalizedAccuracyMeters;
        final boolean usedFallback;

        AccuracyResolution(float rawAccuracyMeters, float normalizedAccuracyMeters, boolean usedFallback) {
            this.rawAccuracyMeters = rawAccuracyMeters;
            this.normalizedAccuracyMeters = normalizedAccuracyMeters;
            this.usedFallback = usedFallback;
        }
    }

    static final class AgeAdjustedAccuracy {
        final boolean rejectObservation;
        final float adjustedAccuracyMeters;
        @NonNull
        final String reasonCode;

        AgeAdjustedAccuracy(boolean rejectObservation, float adjustedAccuracyMeters, @NonNull String reasonCode) {
            this.rejectObservation = rejectObservation;
            this.adjustedAccuracyMeters = adjustedAccuracyMeters;
            this.reasonCode = reasonCode;
        }
    }

    @NonNull
    static AccuracyResolution resolveAccuracy(@NonNull String source, @Nullable Float rawAccuracyMeters) {
        float fallbackAccuracyMeters = "WIFI".equals(source)
                ? WIFI_FALLBACK_ACCURACY_M
                : GENERIC_ABSOLUTE_FALLBACK_ACCURACY_M;
        if (rawAccuracyMeters == null
                || Float.isNaN(rawAccuracyMeters)
                || Float.isInfinite(rawAccuracyMeters)
                || rawAccuracyMeters <= 0f) {
            return new AccuracyResolution(Float.NaN, fallbackAccuracyMeters, true);
        }
        float normalizedAccuracyMeters = clampAccuracy(rawAccuracyMeters);
        return new AccuracyResolution(rawAccuracyMeters, normalizedAccuracyMeters, false);
    }

    @NonNull
    static AgeAdjustedAccuracy adjustForObservationAge(float normalizedAccuracyMeters, long observationAgeMs) {
        if (observationAgeMs < 0L || observationAgeMs <= FRESH_OBSERVATION_MAX_AGE_MS) {
            return new AgeAdjustedAccuracy(false, normalizedAccuracyMeters, "fresh_or_unknown_age");
        }
        if (observationAgeMs >= STALE_REJECT_AGE_MS) {
            return new AgeAdjustedAccuracy(true, normalizedAccuracyMeters, "rejected_stale_observation");
        }
        float progress = (float) (observationAgeMs - FRESH_OBSERVATION_MAX_AGE_MS)
                / (float) (STALE_REJECT_AGE_MS - FRESH_OBSERVATION_MAX_AGE_MS);
        float inflationRatio = 1.0f + progress * (MAX_STALE_ACCURACY_INFLATION_RATIO - 1.0f);
        return new AgeAdjustedAccuracy(
                false,
                clampAccuracy(normalizedAccuracyMeters * inflationRatio),
                "age_downweighted"
        );
    }

    static boolean shouldAllowStationaryCorrection(
            @NonNull String source,
            float adjustedAccuracyMeters,
            long observationAgeMs
    ) {
        if (!"WIFI".equals(source) && !"GNSS".equals(source)) {
            return false;
        }
        if (observationAgeMs < 0L || observationAgeMs > STATIONARY_CORRECTION_MAX_AGE_MS) {
            return false;
        }
        return adjustedAccuracyMeters <= STATIONARY_CORRECTION_MAX_ACCURACY_M;
    }

    static boolean shouldRollbackUnpublishedParticleChange(
            boolean absoluteFixReanchored,
            boolean absoluteFixRejectedByConstraints,
            boolean bootstrapInitializedParticleCloud,
            boolean poseAdvanced,
            boolean keepLimitedSupportCorrection
    ) {
        if (poseAdvanced || keepLimitedSupportCorrection) {
            return false;
        }
        return ((absoluteFixReanchored && !absoluteFixRejectedByConstraints)
                || bootstrapInitializedParticleCloud);
    }

    private static float clampAccuracy(float accuracyMeters) {
        return Math.max(
                MIN_NORMALIZED_ACCURACY_M,
                Math.min(MAX_NORMALIZED_ACCURACY_M, accuracyMeters)
        );
    }
}
