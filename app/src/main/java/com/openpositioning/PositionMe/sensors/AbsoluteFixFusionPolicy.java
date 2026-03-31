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
    static final double LOW_CONFIDENCE_THRESHOLD = 0.45;

    static final long FRESH_OBSERVATION_MAX_AGE_MS = 3_000L;
    static final long STALE_REJECT_AGE_MS = 15_000L;
    static final float MAX_STALE_ACCURACY_INFLATION_RATIO = 2.0f;

    private static final long STATIONARY_CORRECTION_MAX_AGE_MS = 3_000L;
    private static final float STATIONARY_CORRECTION_MAX_ACCURACY_M = 4.0f;
    private static final float CONFLICT_BASELINE_ACCURACY_M = 6.5f;
    private static final double LARGE_CURRENT_SHIFT_MIN_DISTANCE_M = 4.5;
    private static final double ABSOLUTE_SOURCE_DISAGREEMENT_MIN_DISTANCE_M = 6.0;
    private static final double TRUSTED_ANCHOR_SHIFT_MIN_DISTANCE_M = 5.5;
    private static final double STRONG_ABSOLUTE_SOURCE_DISAGREEMENT_M = 9.0;
    private static final double STRONG_TRUSTED_ANCHOR_SHIFT_M = 8.0;
    private static final double HARD_REJECT_MAX_CONFIDENCE = 0.35;
    private static final float TRUSTED_ANCHOR_MAX_ACCURACY_M = 4.5f;
    private static final double TRUSTED_ANCHOR_BOOTSTRAP_MIN_CONFIDENCE = 0.25;
    private static final double WEAK_ANCHOR_PDR_MIN_DISAGREEMENT_M = 6.0;
    private static final int RECOVERY_HINT_REJECT_STREAK_THRESHOLD = 3;
    private static final double RECOVERY_HINT_MAX_PEER_DISTANCE_M = 4.0;
    private static final double RECOVERY_HINT_MIN_CURRENT_SHIFT_M = 8.0;
    private static final float RECOVERY_HINT_MAX_ACCURACY_M = 4.5f;

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

    static final class ConflictAdjustedAbsoluteFix {
        final boolean rejectObservation;
        final float adjustedAccuracyMeters;
        final boolean weakAnchorMode;
        final boolean recoveryHint;
        final double guardDistanceMeters;
        @NonNull
        final String reasonCode;

        ConflictAdjustedAbsoluteFix(
                boolean rejectObservation,
                float adjustedAccuracyMeters,
                boolean weakAnchorMode,
                boolean recoveryHint,
                double guardDistanceMeters,
                @NonNull String reasonCode
        ) {
            this.rejectObservation = rejectObservation;
            this.adjustedAccuracyMeters = adjustedAccuracyMeters;
            this.weakAnchorMode = weakAnchorMode;
            this.recoveryHint = recoveryHint;
            this.guardDistanceMeters = guardDistanceMeters;
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

    @NonNull
    static ConflictAdjustedAbsoluteFix resolveConflictAdjustedAbsoluteFix(
            @NonNull String source,
            float adjustedAccuracyMeters,
            double fusedConfidence,
            double distanceToCurrentPoseMeters,
            @Nullable Double distanceToPeerAbsoluteMeters,
            @Nullable Double distanceToTrustedAnchorMeters
    ) {
        return resolveConflictAdjustedAbsoluteFix(
                source,
                adjustedAccuracyMeters,
                fusedConfidence,
                distanceToCurrentPoseMeters,
                distanceToPeerAbsoluteMeters,
                distanceToTrustedAnchorMeters,
                0
        );
    }

    @NonNull
    static ConflictAdjustedAbsoluteFix resolveConflictAdjustedAbsoluteFix(
            @NonNull String source,
            float adjustedAccuracyMeters,
            double fusedConfidence,
            double distanceToCurrentPoseMeters,
            @Nullable Double distanceToPeerAbsoluteMeters,
            @Nullable Double distanceToTrustedAnchorMeters,
            int rejectStreak
    ) {
        boolean lowConfidence = !Double.isFinite(fusedConfidence)
                || fusedConfidence <= LOW_CONFIDENCE_THRESHOLD;
        boolean weakAccuracy = adjustedAccuracyMeters >= WIFI_FALLBACK_ACCURACY_M;
        boolean largeCurrentShift = isLargeCurrentShift(distanceToCurrentPoseMeters, adjustedAccuracyMeters);
        boolean peerConflict = isPeerConflict(distanceToPeerAbsoluteMeters, adjustedAccuracyMeters);
        boolean trustedAnchorConflict = isTrustedAnchorConflict(
                distanceToTrustedAnchorMeters,
                adjustedAccuracyMeters
        );
        boolean shouldDownweight = (weakAccuracy && (peerConflict || trustedAnchorConflict || largeCurrentShift))
                || (lowConfidence && (peerConflict || trustedAnchorConflict || largeCurrentShift));
        boolean weakAnchorMode = lowConfidence && (peerConflict || trustedAnchorConflict || largeCurrentShift);

        double guardDistanceMeters = maxFinite(
                largeCurrentShift ? distanceToCurrentPoseMeters : Double.NaN,
                peerConflict && distanceToPeerAbsoluteMeters != null ? distanceToPeerAbsoluteMeters : Double.NaN,
                trustedAnchorConflict && distanceToTrustedAnchorMeters != null
                        ? distanceToTrustedAnchorMeters
                        : Double.NaN
        );
        boolean recoveryHint = shouldTriggerRecoveryHint(
                rejectStreak,
                distanceToCurrentPoseMeters,
                distanceToPeerAbsoluteMeters
        );

        if ("WIFI".equals(source)
                && lowConfidence
                && peerConflict
                && trustedAnchorConflict
                && largeCurrentShift
                && distanceToPeerAbsoluteMeters != null
                && distanceToPeerAbsoluteMeters >= STRONG_ABSOLUTE_SOURCE_DISAGREEMENT_M
                && distanceToTrustedAnchorMeters != null
                && distanceToTrustedAnchorMeters >= STRONG_TRUSTED_ANCHOR_SHIFT_M
                && fusedConfidence <= HARD_REJECT_MAX_CONFIDENCE) {
            return new ConflictAdjustedAbsoluteFix(
                    true,
                    clampAccuracy(Math.max(CONFLICT_BASELINE_ACCURACY_M, adjustedAccuracyMeters)),
                    true,
                    false,
                    guardDistanceMeters,
                    "rejected_low_conflict_anchor_shift"
            );
        }

        if (recoveryHint) {
            return new ConflictAdjustedAbsoluteFix(
                    false,
                    clampAccuracy(Math.min(adjustedAccuracyMeters, RECOVERY_HINT_MAX_ACCURACY_M)),
                    false,
                    true,
                    guardDistanceMeters,
                    "consistent_absolute_cluster_override"
            );
        }

        if (!shouldDownweight) {
            return new ConflictAdjustedAbsoluteFix(
                    false,
                    adjustedAccuracyMeters,
                    weakAnchorMode,
                    false,
                    guardDistanceMeters,
                    "stable"
            );
        }

        float inflatedAccuracyMeters = Math.max(adjustedAccuracyMeters, CONFLICT_BASELINE_ACCURACY_M);
        if (peerConflict) {
            inflatedAccuracyMeters += 2.5f;
        }
        if (trustedAnchorConflict) {
            inflatedAccuracyMeters += 3.0f;
        }
        if (largeCurrentShift) {
            inflatedAccuracyMeters += 1.5f;
        }
        if ("WIFI".equals(source) && lowConfidence) {
            inflatedAccuracyMeters += 1.0f;
        }

        return new ConflictAdjustedAbsoluteFix(
                false,
                clampAccuracy(inflatedAccuracyMeters),
                weakAnchorMode,
                false,
                guardDistanceMeters,
                peerConflict || trustedAnchorConflict
                        ? "downweighted_conflicting_absolute_fix"
                        : "downweighted_large_pose_shift"
        );
    }

    static boolean shouldPromoteTrustedAnchor(
            boolean hasExistingTrustedAnchor,
            float adjustedAccuracyMeters,
            double resultingConfidence,
            boolean weakAnchorMode,
            @Nullable Double distanceToPeerAbsoluteMeters
    ) {
        if (weakAnchorMode || adjustedAccuracyMeters > TRUSTED_ANCHOR_MAX_ACCURACY_M) {
            return false;
        }
        if (distanceToPeerAbsoluteMeters != null
                && Double.isFinite(distanceToPeerAbsoluteMeters)
                && distanceToPeerAbsoluteMeters >= STRONG_ABSOLUTE_SOURCE_DISAGREEMENT_M) {
            return false;
        }
        if (!Double.isFinite(resultingConfidence)) {
            return false;
        }
        double minimumConfidence = hasExistingTrustedAnchor
                ? LOW_CONFIDENCE_THRESHOLD
                : TRUSTED_ANCHOR_BOOTSTRAP_MIN_CONFIDENCE;
        return resultingConfidence >= minimumConfidence;
    }

    static float resolveWeakAnchorPdrTranslationScale(
            float stepLengthMeters,
            double fusedConfidence,
            double guardDistanceMeters,
            boolean weakAnchorMode
    ) {
        if (!weakAnchorMode || stepLengthMeters <= 0f) {
            return 1.0f;
        }
        double confidencePenalty = !Double.isFinite(fusedConfidence)
                ? 1.0
                : clamp01((LOW_CONFIDENCE_THRESHOLD - fusedConfidence) / LOW_CONFIDENCE_THRESHOLD);
        double disagreementPenalty = Double.isFinite(guardDistanceMeters)
                ? clamp01((guardDistanceMeters - WEAK_ANCHOR_PDR_MIN_DISAGREEMENT_M) / 8.0)
                : 0.0;
        double severity = Math.max(confidencePenalty, disagreementPenalty);
        if (severity <= 0.0) {
            return 1.0f;
        }
        double longStepPenalty = clamp01((stepLengthMeters - 0.55f) / 0.65f);
        double scale = 1.0 - (0.40 * severity) - (0.12 * longStepPenalty * severity);
        return clampScale((float) scale);
    }

    private static float clampAccuracy(float accuracyMeters) {
        return Math.max(
                MIN_NORMALIZED_ACCURACY_M,
                Math.min(MAX_NORMALIZED_ACCURACY_M, accuracyMeters)
        );
    }

    private static boolean shouldTriggerRecoveryHint(
            int rejectStreak,
            double distanceToCurrentPoseMeters,
            @Nullable Double distanceToPeerAbsoluteMeters
    ) {
        if (rejectStreak < RECOVERY_HINT_REJECT_STREAK_THRESHOLD) {
            return false;
        }
        if (!Double.isFinite(distanceToCurrentPoseMeters)
                || distanceToCurrentPoseMeters < RECOVERY_HINT_MIN_CURRENT_SHIFT_M) {
            return false;
        }
        return distanceToPeerAbsoluteMeters != null
                && Double.isFinite(distanceToPeerAbsoluteMeters)
                && distanceToPeerAbsoluteMeters <= RECOVERY_HINT_MAX_PEER_DISTANCE_M;
    }

    private static boolean isLargeCurrentShift(double distanceMeters, float adjustedAccuracyMeters) {
        if (!Double.isFinite(distanceMeters)) {
            return false;
        }
        return distanceMeters >= Math.max(
                LARGE_CURRENT_SHIFT_MIN_DISTANCE_M,
                adjustedAccuracyMeters * 1.25f
        );
    }

    private static boolean isPeerConflict(
            @Nullable Double distanceToPeerAbsoluteMeters,
            float adjustedAccuracyMeters
    ) {
        if (distanceToPeerAbsoluteMeters == null || !Double.isFinite(distanceToPeerAbsoluteMeters)) {
            return false;
        }
        return distanceToPeerAbsoluteMeters >= Math.max(
                ABSOLUTE_SOURCE_DISAGREEMENT_MIN_DISTANCE_M,
                adjustedAccuracyMeters * 1.5f
        );
    }

    private static boolean isTrustedAnchorConflict(
            @Nullable Double distanceToTrustedAnchorMeters,
            float adjustedAccuracyMeters
    ) {
        if (distanceToTrustedAnchorMeters == null || !Double.isFinite(distanceToTrustedAnchorMeters)) {
            return false;
        }
        return distanceToTrustedAnchorMeters >= Math.max(
                TRUSTED_ANCHOR_SHIFT_MIN_DISTANCE_M,
                adjustedAccuracyMeters * 1.4f
        );
    }

    private static double maxFinite(double first, double second, double third) {
        double max = Double.NaN;
        if (Double.isFinite(first)) {
            max = first;
        }
        if (Double.isFinite(second) && (!Double.isFinite(max) || second > max)) {
            max = second;
        }
        if (Double.isFinite(third) && (!Double.isFinite(max) || third > max)) {
            max = third;
        }
        return max;
    }

    private static double clamp01(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }

    private static float clampScale(float value) {
        return Math.max(0.55f, Math.min(1.0f, value));
    }
}
