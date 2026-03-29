package com.openpositioning.PositionMe.sensors;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class SensorFusionStationaryGateTest {

    @Test
    public void stationaryWindowAcceptsLowMotionWithOnlyIsolatedSpikes() {
        List<Double> magnitudes = new ArrayList<>();
        for (int i = 0; i < 31; i++) {
            magnitudes.add(0.02);
        }
        magnitudes.add(0.24);

        assertTrue(SensorFusion.isStationaryFromLinearAccelerationWindow(magnitudes));
    }

    @Test
    public void stationaryWindowRejectsSustainedMotion() {
        List<Double> magnitudes = new ArrayList<>();
        for (int i = 0; i < 16; i++) {
            magnitudes.add(0.18);
        }
        for (int i = 0; i < 16; i++) {
            magnitudes.add(0.45);
        }

        assertFalse(SensorFusion.isStationaryFromLinearAccelerationWindow(magnitudes));
    }

    @Test
    public void stationaryWindowRejectsMultipleTremorSpikes() {
        List<Double> magnitudes = new ArrayList<>();
        for (int i = 0; i < 30; i++) {
            magnitudes.add(0.02);
        }
        magnitudes.add(0.28);
        magnitudes.add(0.27);

        assertFalse(SensorFusion.isStationaryFromLinearAccelerationWindow(magnitudes));
    }

    @Test
    public void stationaryWindowRequiresFullHistory() {
        List<Double> magnitudes = new ArrayList<>();
        for (int i = 0; i < 10; i++) {
            magnitudes.add(0.01);
        }

        assertFalse(SensorFusion.isStationaryFromLinearAccelerationWindow(magnitudes));
    }

    @Test
    public void immediateMotionResumeTriggersBeforeConfirmedStep() {
        assertTrue(SensorFusion.shouldForceMotionResume(0.12, 0.08, 0));
        assertTrue(SensorFusion.shouldForceMotionResume(0.05, 0.11, 0));
        assertTrue(SensorFusion.shouldForceMotionResume(0.05, 0.08, 2));
        assertFalse(SensorFusion.shouldForceMotionResume(0.05, 0.08, 0));
    }

    @Test
    public void motionResumeWindowStaysActiveUntilExpiry() {
        assertTrue(SensorFusion.isMotionResumeWindowActive(1500L, 1000L));
        assertTrue(SensorFusion.isMotionResumeWindowActive(1500L, 1500L));
        assertFalse(SensorFusion.isMotionResumeWindowActive(1500L, 1501L));
        assertFalse(SensorFusion.isMotionResumeWindowActive(Long.MIN_VALUE, 1000L));
    }
}
