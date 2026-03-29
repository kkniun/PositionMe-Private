package com.openpositioning.PositionMe.presentation.fragment;

final class MapMatchingStateResolver {

    enum MapMatchingUiState {
        WAITING_FOR_ABSOLUTE_FIX,
        PENDING,
        ACTIVE,
        DISPLAY_ONLY,
        UNAVAILABLE
    }

    private MapMatchingStateResolver() {
    }

    static MapMatchingUiState resolve(
            boolean hasAbsoluteAnchor,
            boolean indoorMapManagerReady,
            boolean activeConstraints,
            boolean displayOnlyMap,
            boolean hasRequestedNearbyVenues,
            boolean venueRequestInFlight
    ) {
        if (!hasAbsoluteAnchor) {
            return MapMatchingUiState.WAITING_FOR_ABSOLUTE_FIX;
        }
        if (!indoorMapManagerReady) {
            return MapMatchingUiState.PENDING;
        }
        if (activeConstraints) {
            return MapMatchingUiState.ACTIVE;
        }
        if (displayOnlyMap) {
            return MapMatchingUiState.DISPLAY_ONLY;
        }
        if (!hasRequestedNearbyVenues || venueRequestInFlight) {
            return MapMatchingUiState.PENDING;
        }
        return MapMatchingUiState.UNAVAILABLE;
    }
}
