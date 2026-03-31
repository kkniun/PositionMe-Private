package com.openpositioning.PositionMe.sensors;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

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

    @Test
    public void unknownHostErrorsAreClassifiedAsOffline() {
        assertTrue(WiFiPositioning.isLikelyOfflineErrorMessage(
                "Error message: java.net.UnknownHostException: Unable to resolve host \"openpositioning.org\""
        ));
        assertFalse(WiFiPositioning.isLikelyOfflineErrorMessage("Validation Error (422): invalid fingerprint"));
    }
}
