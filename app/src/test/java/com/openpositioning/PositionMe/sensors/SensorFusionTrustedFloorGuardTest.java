package com.openpositioning.PositionMe.sensors;

import org.junit.After;
import org.junit.Test;

import java.lang.reflect.Field;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class SensorFusionTrustedFloorGuardTest {

    private final SensorFusion sensorFusion = SensorFusion.getInstance();

    @After
    public void tearDown() throws Exception {
        setField("pdrFloorOffset", 0);
        setField("isFloorOffsetInitialized", false);
        setField("latestFusedPose", null);
        setField("elevation", 0f);
    }

    @Test
    public void untrustedUiFloorIsNeverWrittenBackToBaseline() throws Exception {
        setField("pdrFloorOffset", 4);
        setField("isFloorOffsetInitialized", true);

        boolean recalibrated = sensorFusion.recalibrateAbsoluteFloorBaselineIfTrusted(
                0,
                false,
                10_000L
        );

        assertFalse(recalibrated);
        assertTrue((Boolean) getField("isFloorOffsetInitialized"));
        org.junit.Assert.assertEquals(4, getField("pdrFloorOffset"));
    }

    @Test
    public void trustedManualFloorSelectionCanStillRecalibrateBaseline() throws Exception {
        setField("pdrFloorOffset", 1);
        setField("isFloorOffsetInitialized", true);

        boolean recalibrated = sensorFusion.recalibrateAbsoluteFloorBaselineIfTrusted(
                3,
                true,
                11_000L
        );

        assertTrue(recalibrated);
        assertTrue((Boolean) getField("isFloorOffsetInitialized"));
        org.junit.Assert.assertEquals(3, getField("pdrFloorOffset"));
    }

    @Test
    public void untrustedFloorCannotRecalibrateBaselineThroughLegacyApi() throws Exception {
        setField("pdrFloorOffset", 5);
        setField("isFloorOffsetInitialized", false);

        sensorFusion.recalibrateAbsoluteFloorBaseline(0, 13_000L);

        assertFalse((Boolean) getField("isFloorOffsetInitialized"));
        org.junit.Assert.assertEquals(5, getField("pdrFloorOffset"));
    }

    @Test
    public void trustedFloorCanStillRecalibrateBaselineThroughLegacyApi() throws Exception {
        setField("pdrFloorOffset", 1);
        setField("isFloorOffsetInitialized", true);

        sensorFusion.recalibrateAbsoluteFloorBaseline(4, 14_000L);

        assertTrue((Boolean) getField("isFloorOffsetInitialized"));
        org.junit.Assert.assertEquals(4, getField("pdrFloorOffset"));
    }

    @Test
    public void groundFloorCanStillWorkWhenItIsActuallyTrusted() throws Exception {
        setField("pdrFloorOffset", 2);
        setField("isFloorOffsetInitialized", true);

        boolean recalibrated = sensorFusion.recalibrateAbsoluteFloorBaselineIfTrusted(
                0,
                true,
                12_000L
        );

        assertTrue(recalibrated);
        assertTrue((Boolean) getField("isFloorOffsetInitialized"));
        org.junit.Assert.assertEquals(0, getField("pdrFloorOffset"));
    }

    private void setField(String fieldName, Object value) throws Exception {
        Field field = SensorFusion.class.getDeclaredField(fieldName);
        field.setAccessible(true);
        field.set(sensorFusion, value);
    }

    private Object getField(String fieldName) throws Exception {
        Field field = SensorFusion.class.getDeclaredField(fieldName);
        field.setAccessible(true);
        return field.get(sensorFusion);
    }
}
