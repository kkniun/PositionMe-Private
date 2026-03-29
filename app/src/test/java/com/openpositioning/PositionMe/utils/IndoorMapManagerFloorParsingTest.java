package com.openpositioning.PositionMe.utils;

import org.junit.Test;

import java.util.Arrays;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

public class IndoorMapManagerFloorParsingTest {

    @Test
    public void invalidFloorNameDoesNotFallbackToGroundFloor() {
        assertNull(IndoorFloorKeyResolver.tryParseAbsoluteFloor("mystery-floor"));
    }

    @Test
    public void invalidFloorNameIsSkippedFromAvailableFloorSet() {
        List<Integer> availableFloors = IndoorFloorKeyResolver.collectAvailableFloors(
                Arrays.asList("GF", "mystery-floor", "L2")
        );

        assertEquals(Arrays.asList(0, 2), availableFloors);
    }

    @Test
    public void nearestFloorSnapIgnoresUnparseableFloorKeys() {
        List<Integer> availableFloors = IndoorFloorKeyResolver.collectAvailableFloors(
                Arrays.asList("GF", "broken-key", "L2")
        );

        assertEquals(2, IndoorFloorKeyResolver.snapToNearestFloor(3, availableFloors));
    }
}
