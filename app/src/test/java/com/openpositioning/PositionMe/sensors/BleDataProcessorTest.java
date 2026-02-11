package com.openpositioning.PositionMe.sensors;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

/**
 * 针对 BLE 统计纯函数的单元测试，确保 UI 概览计算稳定。
 */
public class BleDataProcessorTest {

    @Test
    public void calculateStrongestBleRssi_emptyInput_returnsDefaultValue() {
        assertEquals(-100, BleDataProcessor.calculateStrongestBleRssi(new int[]{}));
    }

    @Test
    public void calculateStrongestBleRssi_allNegative_returnsMaxValue() {
        int[] rssiValues = new int[]{-95, -71, -83, -60};
        assertEquals(-60, BleDataProcessor.calculateStrongestBleRssi(rssiValues));
    }

    @Test
    public void calculateStrongestBleRssi_mixedValues_returnsLargestValue() {
        int[] rssiValues = new int[]{-88, 3, -20, 0};
        assertEquals(3, BleDataProcessor.calculateStrongestBleRssi(rssiValues));
    }

    @Test
    public void countUniqueBleDeviceKeys_deduplicatesAndIgnoresEmpty() {
        String[] keys = new String[]{"mac:AA", "mac:AA", "uuid:01", "", null};
        assertEquals(2, BleDataProcessor.countUniqueBleDeviceKeys(keys));
    }

    @Test
    public void countUniqueBleDeviceKeys_emptyInput_returnsZero() {
        assertEquals(0, BleDataProcessor.countUniqueBleDeviceKeys(new String[]{}));
    }
}
