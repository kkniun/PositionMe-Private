package com.openpositioning.PositionMe.sensors;

import org.junit.Test;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class SensorFusionWifiFloorSanityTest {

    @Test
    public void rejectsWifiFloorJumpWhenBarometerHeightDoesNotSupportIt() {
        assertFalse(SensorFusion.isWifiFloorPlausibleWithBarometer(
                3,
                1,
                0.2f,
                3.6f,
                true
        ));
    }

    @Test
    public void acceptsWifiFloorWhenBarometerHeightMatchesTargetLevel() {
        assertTrue(SensorFusion.isWifiFloorPlausibleWithBarometer(
                2,
                1,
                2.5f,
                3.6f,
                true
        ));
    }

    @Test
    public void skipsWifiFloorSanityWhenBarometerIsNotReady() {
        assertTrue(SensorFusion.isWifiFloorPlausibleWithBarometer(
                3,
                1,
                0f,
                3.6f,
                false
        ));
    }
}
