package com.openpositioning.PositionMe.presentation.fragment;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class MapMatchingStateResolverTest {

    @Test
    public void displayOnlyMapIsDistinctFromUnavailableState() {
        assertEquals(
                MapMatchingStateResolver.MapMatchingUiState.DISPLAY_ONLY,
                MapMatchingStateResolver.resolve(true, true, false, true, true, false)
        );
    }

    @Test
    public void missingVenueAfterLookupIsUnavailable() {
        assertEquals(
                MapMatchingStateResolver.MapMatchingUiState.UNAVAILABLE,
                MapMatchingStateResolver.resolve(true, true, false, false, true, false)
        );
    }

    @Test
    public void waitingForAbsoluteFixTakesPriorityOverMapState() {
        assertEquals(
                MapMatchingStateResolver.MapMatchingUiState.WAITING_FOR_ABSOLUTE_FIX,
                MapMatchingStateResolver.resolve(false, true, true, true, true, false)
        );
    }
}
