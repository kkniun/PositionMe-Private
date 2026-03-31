package com.openpositioning.PositionMe.utils;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.google.android.gms.maps.model.LatLng;

/**
 * 统一管理“这次 pose 变化是否真的代表人在平移”。
 *
 * <p>这里故意偏保守：只有 PDR、最近接受的 step，或者 motion-resume 窗口内，
 * 才允许把位移当作真实走路。这样可以避免静止时的绝对定位小跳动被错误记成路径。</p>
 */
public final class LiveMotionGate {
    public static final long RECENT_STEP_WINDOW_MS = 1_500L;
    public static final double TRACKED_DISTANCE_MIN_INCREMENT_M = 0.35;
    public static final double PASSIVE_POSE_HOLD_THRESHOLD_M = 2.0;
    public static final double STATIONARY_JITTER_HOLD_THRESHOLD_M = 0.15;

    private LiveMotionGate() {
    }

    public static boolean hasRecentAcceptedStep(long poseTimestampMs, long lastAcceptedStepTimestampMs) {
        if (poseTimestampMs <= 0L || lastAcceptedStepTimestampMs == Long.MIN_VALUE) {
            return false;
        }
        return Math.max(0L, poseTimestampMs - lastAcceptedStepTimestampMs) <= RECENT_STEP_WINDOW_MS;
    }

    public static boolean hasTranslationalMotionEvidence(
            boolean motionResumeActive,
            @NonNull String poseSource,
            long poseTimestampMs,
            long lastAcceptedStepTimestampMs
    ) {
        // 这里故意不再把 motion-resume 单独当成“已经发生平移”的证据，
        // 否则静止时的 IMU 抖动会放大为 GNSS/WiFi 漂移和虚假路径增长。
        if ("PDR".equals(poseSource)) {
            return true;
        }
        return hasRecentAcceptedStep(poseTimestampMs, lastAcceptedStepTimestampMs);
    }

    public static boolean shouldAccumulateTrackedDistance(
            boolean hasFreshPose,
            double displacementMeters,
            boolean motionResumeActive,
            @NonNull String poseSource,
            long poseTimestampMs,
            long lastAcceptedStepTimestampMs
    ) {
        if (!hasFreshPose || !Double.isFinite(displacementMeters)) {
            return false;
        }
        if (displacementMeters < TRACKED_DISTANCE_MIN_INCREMENT_M) {
            return false;
        }
        return hasTranslationalMotionEvidence(
                motionResumeActive,
                poseSource,
                poseTimestampMs,
                lastAcceptedStepTimestampMs
        );
    }

    public static boolean shouldHoldPoseForPassiveUpdate(
            @Nullable LatLng currentLocation,
            @NonNull LatLng candidateLocation,
            boolean stationary,
            boolean motionResumeActive,
            @NonNull String poseSource,
            long poseTimestampMs,
            long lastAcceptedStepTimestampMs
    ) {
        if (currentLocation == null) {
            return false;
        }
        double displacementMeters = UtilFunctions.distanceBetweenPoints(currentLocation, candidateLocation);
        if (!Double.isFinite(displacementMeters)) {
            return false;
        }
        if (stationary && displacementMeters < STATIONARY_JITTER_HOLD_THRESHOLD_M) {
            return true;
        }
        if (hasTranslationalMotionEvidence(
                motionResumeActive,
                poseSource,
                poseTimestampMs,
                lastAcceptedStepTimestampMs
        )) {
            return false;
        }
        return displacementMeters < PASSIVE_POSE_HOLD_THRESHOLD_M;
    }
}
