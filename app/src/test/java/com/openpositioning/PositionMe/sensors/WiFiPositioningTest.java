package com.openpositioning.PositionMe.sensors;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

public class WiFiPositioningTest {

    @Test
    public void wifiMissingFloorDoesNotDefaultToGroundFloor() {
        assertNull(WiFiPositioning.parseFloorValue(null));
        assertNull(WiFiPositioning.parseFloorValue(""));
    }

    @Test
    public void wifiExplicitFloorStillParsesNormally() {
        assertEquals(Integer.valueOf(2), WiFiPositioning.parseFloorValue(2));
        assertEquals(Integer.valueOf(3), WiFiPositioning.parseFloorValue("3"));
    }
}
