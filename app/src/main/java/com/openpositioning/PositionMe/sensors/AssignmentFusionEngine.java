package com.openpositioning.PositionMe.sensors;

import androidx.annotation.Nullable;

import com.openpositioning.PositionMe.utils.PdrProcessing;

/**
 * Minimal assignment-style fusion: single 2D pose + floor, PDR prediction with map walkability,
 * and gentle pulls toward GNSS/WiFi in local easting/northing.
 *
 * <p>Replaces the previous particle filter for simpler field debugging and more predictable
 * behaviour while still satisfying the coursework fusion + map-matching hooks.</p>
 */
public final class AssignmentFusionEngine {

    private static final double MIN_HEIGHT_DELTA_FOR_FLOOR_CHANGE_M = 1.5;
    private static final double MIN_STAIRS_STEP_LENGTH_M = 0.25;

    @FunctionalInterface
    public interface Walkability {
        boolean isWalkable(double localXMeters, double localYMeters, int floorIndex);
    }

    private double x;
    private double y;
    private int floor;
    private double headingRad;
    private boolean initialized;
    private long lastTimestampMs;

    public void reset() {
        initialized = false;
        lastTimestampMs = 0L;
    }

    public boolean isInitialized() {
        return initialized;
    }

    public FusedPose initFromAbsolute(
            double localX,
            double localY,
            int floorIndex,
            long timestampMs,
            double headingRadians
    ) {
        this.x = localX;
        this.y = localY;
        this.floor = floorIndex;
        this.headingRad = normalizeHeading(headingRadians);
        this.initialized = true;
        this.lastTimestampMs = timestampMs;
        return estimatePose(timestampMs);
    }

    /**
     * Soft update toward an absolute fix (GNSS / WiFi) already expressed in local metres.
     */
    public FusedPose applyAbsoluteObservation(
            double localX,
            double localY,
            @Nullable Integer floorPrior,
            float accuracyMeters,
            long timestampMs
    ) {
        if (!initialized) {
            int f = floorPrior != null ? floorPrior : 0;
            return initFromAbsolute(localX, localY, f, timestampMs, headingRad);
        }

        double acc = Math.max(1.0, accuracyMeters);
        double gain = Math.min(0.5, 12.0 / (12.0 + acc));
        x += gain * (localX - x);
        y += gain * (localY - y);
        if (floorPrior != null) {
            floor = floorPrior;
        }
        lastTimestampMs = timestampMs;
        return estimatePose(timestampMs);
    }

    public FusedPose predictFromPdr(
            @Nullable PdrDelta delta,
            int barometerFloorHint,
            long timestampMs,
            @Nullable FloorTransitionGate floorGate,
            Walkability walkability
    ) {
        if (!initialized || delta == null) {
            return estimatePose(timestampMs);
        }

        double ox = x;
        double oy = y;
        int f0 = floor;

        double predHeading = normalizeHeading(headingRad + delta.getDeltaHeadingRad());
        double stepLen = Math.max(0.0, delta.getStepLengthMeters());

        int f1 = resolveSuggestedFloor(
                f0,
                barometerFloorHint,
                delta.getHeightDeltaMeters(),
                stepLen,
                delta.isElevatorLikely()
        );

        double[] full = PdrProcessing.projectStepToLocalFrame(stepLen, predHeading);
        double cx = ox + full[0];
        double cy = oy + full[1];
        int floorForMove = f1;
        if (floorForMove != f0 && floorGate != null
                && !floorGate.allowsFloorChange(ox, oy, cx, cy, f0, floorForMove)) {
            floorForMove = f0;
        }

        if (walkability.isWalkable(cx, cy, floorForMove)) {
            x = cx;
            y = cy;
            floor = floorForMove;
        } else {
            double[] half = PdrProcessing.projectStepToLocalFrame(stepLen * 0.5, predHeading);
            double hx = ox + half[0];
            double hy = oy + half[1];
            int floorHalf = f1;
            if (floorHalf != f0 && floorGate != null
                    && !floorGate.allowsFloorChange(ox, oy, hx, hy, f0, floorHalf)) {
                floorHalf = f0;
            }
            if (walkability.isWalkable(hx, hy, floorHalf)) {
                x = hx;
                y = hy;
                floor = floorHalf;
            }
        }

        headingRad = predHeading;
        lastTimestampMs = timestampMs;
        return estimatePose(timestampMs);
    }

    private int resolveSuggestedFloor(
            int previousFloor,
            int externalFloor,
            double heightDeltaMeters,
            double stepLengthMeters,
            boolean elevatorLikely
    ) {
        if (externalFloor == previousFloor) {
            return previousFloor;
        }
        if (Math.abs(heightDeltaMeters) < MIN_HEIGHT_DELTA_FOR_FLOOR_CHANGE_M) {
            return previousFloor;
        }
        if (!elevatorLikely && stepLengthMeters < MIN_STAIRS_STEP_LENGTH_M) {
            return previousFloor;
        }
        return externalFloor;
    }

    private FusedPose estimatePose(long timestampMs) {
        if (!initialized) {
            return null;
        }
        long ts = lastTimestampMs > 0 ? lastTimestampMs : timestampMs;
        return new FusedPose(x, y, floor, 1.0, ts);
    }

    private static double normalizeHeading(double headingRad) {
        double twoPi = Math.PI * 2.0;
        double n = headingRad % twoPi;
        if (n < 0.0) {
            n += twoPi;
        }
        return n;
    }
}
