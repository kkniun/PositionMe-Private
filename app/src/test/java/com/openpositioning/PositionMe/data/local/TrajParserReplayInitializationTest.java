package com.openpositioning.PositionMe.data.local;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class TrajParserReplayInitializationTest {

    @Test
    public void resolveReplayInitialization_prefersFusedPoseAndEarliestAbsoluteFix() {
        JsonObject root = JsonParser.parseString("{"
                + "\"fused_pose\":[{\"relative_timestamp\":1000,\"x\":0.0,\"y\":0.0,\"floor\":0}],"
                + "\"gnss_data\":[{\"position\":{\"relative_timestamp\":5000,\"latitude\":55.9500,\"longitude\":-3.1900}}],"
                + "\"wifi_fingerprints\":[{"
                + "\"relative_timestamp\":2000,"
                + "\"rf_scans\":[{\"relative_timestamp\":2000,\"mac\":1,\"rssi\":-50,"
                + "\"position\":{\"latitude\":55.9444,\"longitude\":-3.1883}}]"
                + "}],"
                + "\"initial_position\":{\"latitude\":10.0,\"longitude\":20.0}"
                + "}").getAsJsonObject();

        TrajParser.ReplayInitialization initialization = TrajParser.resolveReplayInitialization(root);

        assertTrue(initialization.useFusedPose);
        assertEquals(TrajParser.ReplayInitializationSource.FUSED_POSE, initialization.source);
        assertNotNull(initialization.origin);
        assertEquals(55.9444, initialization.origin.latitude, 1e-6);
        assertEquals(-3.1883, initialization.origin.longitude, 1e-6);
    }

    @Test
    public void resolveReplayInitialization_usesLegacyFallbackOnlyWhenNoAbsoluteFixExists() {
        JsonObject root = JsonParser.parseString("{"
                + "\"pdr_data\":[{\"relative_timestamp\":100,\"x\":0.0,\"y\":0.0,\"floor\":0}],"
                + "\"initial_position\":{\"latitude\":55.9230,\"longitude\":-3.1740}"
                + "}").getAsJsonObject();

        TrajParser.ReplayInitialization initialization = TrajParser.resolveReplayInitialization(root);

        assertFalse(initialization.useFusedPose);
        assertEquals(TrajParser.ReplayInitializationSource.LEGACY_FALLBACK, initialization.source);
        assertNotNull(initialization.origin);
        assertEquals(55.9230, initialization.origin.latitude, 1e-6);
        assertEquals(-3.1740, initialization.origin.longitude, 1e-6);
    }
}
