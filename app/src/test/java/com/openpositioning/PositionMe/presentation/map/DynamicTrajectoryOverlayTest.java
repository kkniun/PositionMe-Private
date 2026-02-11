package com.openpositioning.PositionMe.presentation.map;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

/**
 * 验证动态轨迹颜色映射规则（纯函数，无地图依赖）。
 */
public class DynamicTrajectoryOverlayTest {

    @Test
    public void resolveColorForRssi_strongSignal_returnsGreen() {
        assertEquals(DynamicTrajectoryOverlay.COLOR_GREEN, DynamicTrajectoryOverlay.resolveColorForRssi(-55));
        assertEquals(DynamicTrajectoryOverlay.COLOR_GREEN, DynamicTrajectoryOverlay.resolveColorForRssi(-30));
    }

    @Test
    public void resolveColorForRssi_mediumSignal_returnsYellow() {
        assertEquals(DynamicTrajectoryOverlay.COLOR_YELLOW, DynamicTrajectoryOverlay.resolveColorForRssi(-70));
        assertEquals(DynamicTrajectoryOverlay.COLOR_YELLOW, DynamicTrajectoryOverlay.resolveColorForRssi(-60));
    }

    @Test
    public void resolveColorForRssi_weakSignal_returnsRed() {
        assertEquals(DynamicTrajectoryOverlay.COLOR_RED, DynamicTrajectoryOverlay.resolveColorForRssi(-71));
        assertEquals(DynamicTrajectoryOverlay.COLOR_RED, DynamicTrajectoryOverlay.resolveColorForRssi(-95));
    }
}
