package com.openpositioning.PositionMe.presentation.fragment;

import org.junit.Test;

import static org.junit.Assert.assertTrue;

/**
 * DebugInfoFormatter 的纯函数单测。
 */
public class DebugInfoFormatterTest {

    @Test
    public void format_containsTimestampAndFixedFieldNames() {
        String text = DebugInfoFormatter.format(
                "2026-02-11 10:00:00",
                new float[]{1f, 2f, 3f},
                new float[]{4f, 5f, 6f},
                new float[]{7f, 8f, 9f},
                new float[]{10f, 11f, 12f},
                50f,
                1000f,
                1f,
                new float[]{31.20f, 121.50f},
                new float[]{3.5f, 4.6f},
                7,
                -45,
                3,
                -60
        );

        assertTrue(text.contains("timestamp: 2026-02-11 10:00:00"));
        assertTrue(text.contains("Accelerometer:"));
        assertTrue(text.contains("GNSS(lat,long):"));
        assertTrue(text.contains("WiFi: apCount=7, strongestRssi=-45 dBm"));
        assertTrue(text.contains("BLE: deviceCount=3, strongestRssi=-60 dBm"));
    }

    @Test
    public void format_nullValues_outputsNAWithoutCrash() {
        String text = DebugInfoFormatter.format(
                null,
                null,
                null,
                null,
                null,
                null,
                null,
                null,
                null,
                null,
                null,
                null,
                null,
                null
        );

        assertTrue(text.contains("timestamp: N/A"));
        assertTrue(text.contains("Accelerometer: N/A"));
        assertTrue(text.contains("Light: N/A"));
        assertTrue(text.contains("WiFi: apCount=N/A, strongestRssi=N/A"));
        assertTrue(text.contains("BLE: deviceCount=N/A, strongestRssi=N/A"));
    }
}
