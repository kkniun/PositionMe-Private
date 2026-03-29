package com.openpositioning.PositionMe.sensors;

/**
 * Decides whether the fused track may change floor during a PDR prediction step.
 * Used to enforce map-based rules (e.g. only near stairs/lift polygons).
 */
public interface FloorTransitionGate {

    /**
     * @param previousXMeters previous particle x in local metres
     * @param previousYMeters previous particle y in local metres
     * @param predictedXMeters predicted x after motion model
     * @param predictedYMeters predicted y after motion model
     * @param previousFloor     floor index before this predict step
     * @param newFloor          floor index requested by height / external floor estimate
     * @return true if the floor change from previousFloor to newFloor is allowed
     */
    boolean allowsFloorChange(
            double previousXMeters,
            double previousYMeters,
            double predictedXMeters,
            double predictedYMeters,
            int previousFloor,
            int newFloor
    );
}
