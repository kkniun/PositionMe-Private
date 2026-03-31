package com.openpositioning.PositionMe.presentation.fragment;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.google.android.gms.maps.model.LatLng;
import com.openpositioning.PositionMe.utils.MapConstraintRepository;
import com.openpositioning.PositionMe.utils.UtilFunctions;

final class MapDisplayConstraintFilter {
    private static final double MAX_CONSTRAINED_CORRECTION_DISTANCE_M = 12.0;
    private static final double MIN_CONSTRAINED_CORRECTION_PROGRESS_M = 0.05;
    private static final int CONSTRAINED_CORRECTION_ITERATIONS = 12;

    private MapDisplayConstraintFilter() {
    }

    static boolean isRenderablePoint(@Nullable LatLng point, int floor) {
        return point != null && MapConstraintRepository.isPointLegal(point, floor);
    }

    static boolean isRenderableObservationPoint(
            @Nullable LatLng point,
            @Nullable Integer observationFloor,
            @Nullable Integer displayedFloor
    ) {
        if (point == null) {
            return false;
        }
        if (observationFloor != null && displayedFloor != null && !observationFloor.equals(displayedFloor)) {
            return false;
        }
        if (observationFloor != null) {
            return isRenderablePoint(point, observationFloor);
        }
        if (displayedFloor != null) {
            return isRenderablePoint(point, displayedFloor);
        }
        return !MapConstraintRepository.hasVenueOutline()
                || MapConstraintRepository.isPointInsideVenueOutline(point);
    }

    static boolean isRenderableSegment(
            @Nullable LatLng start,
            @Nullable LatLng end,
            int startFloor,
            int endFloor
    ) {
        if (start == null || end == null) {
            return false;
        }
        if (startFloor != endFloor) {
            return isRenderablePoint(start, startFloor) && isRenderablePoint(end, endFloor);
        }
        return MapConstraintRepository.isPathLegal(start, end, endFloor);
    }

    static boolean shouldHoldPositionForIllegalTransition(
            @Nullable LatLng currentLocation,
            @Nullable LatLng candidateLocation,
            int currentFloor,
            int candidateFloor
    ) {
        return shouldHoldPositionForIllegalTransition(
                currentLocation,
                candidateLocation,
                currentFloor,
                candidateFloor,
                false
        );
    }

    static boolean shouldHoldPositionForIllegalTransition(
            @Nullable LatLng currentLocation,
            @Nullable LatLng candidateLocation,
            int currentFloor,
            int candidateFloor,
            boolean forceAcceptTransition
    ) {
        if (!isRenderablePoint(candidateLocation, candidateFloor)) {
            return true;
        }
        if (forceAcceptTransition) {
            return false;
        }
        if (currentLocation == null) {
            return false;
        }
        return currentFloor == candidateFloor
                && !isRenderableSegment(currentLocation, candidateLocation, currentFloor, candidateFloor);
    }

    @Nullable
    static LatLng constrainToRenderableSegmentPrefix(
            @Nullable LatLng currentLocation,
            @Nullable LatLng candidateLocation,
            int currentFloor,
            int candidateFloor
    ) {
        return constrainToRenderableSegmentPrefix(
                currentLocation,
                candidateLocation,
                currentFloor,
                candidateFloor,
                false
        );
    }

    @Nullable
    static LatLng constrainToRenderableSegmentPrefix(
            @Nullable LatLng currentLocation,
            @Nullable LatLng candidateLocation,
            int currentFloor,
            int candidateFloor,
            boolean forceAcceptTransition
    ) {
        if (currentLocation == null
                || candidateLocation == null
                || currentFloor != candidateFloor) {
            return null;
        }
        if (forceAcceptTransition && isRenderablePoint(candidateLocation, candidateFloor)) {
            return candidateLocation;
        }
        double distanceMeters = UtilFunctions.distanceBetweenPoints(currentLocation, candidateLocation);
        if (!Double.isFinite(distanceMeters)
                || distanceMeters < MIN_CONSTRAINED_CORRECTION_PROGRESS_M
                || distanceMeters > MAX_CONSTRAINED_CORRECTION_DISTANCE_M) {
            return null;
        }
        if (isRenderableSegment(currentLocation, candidateLocation, currentFloor, candidateFloor)) {
            return candidateLocation;
        }

        double low = 0.0;
        double high = 1.0;
        LatLng best = null;
        for (int i = 0; i < CONSTRAINED_CORRECTION_ITERATIONS; i++) {
            double fraction = (low + high) * 0.5;
            LatLng probe = interpolate(currentLocation, candidateLocation, fraction);
            if (isRenderableSegment(currentLocation, probe, currentFloor, candidateFloor)) {
                best = probe;
                low = fraction;
            } else {
                high = fraction;
            }
        }
        return best;
    }

    static boolean shouldBypassSmoothing(
            @Nullable LatLng currentDisplayedLocation,
            @Nullable LatLng candidateLocation,
            int currentFloor,
            int candidateFloor
    ) {
        if (currentDisplayedLocation == null || candidateLocation == null) {
            return false;
        }
        return currentFloor != candidateFloor
                || !isRenderableSegment(currentDisplayedLocation, candidateLocation, currentFloor, candidateFloor);
    }

    @Nullable
    static LatLng resolveBestRenderableDisplayLocation(
            @Nullable LatLng currentDisplayedLocation,
            @NonNull LatLng rawCandidateLocation,
            @Nullable LatLng displayCandidateLocation,
            int currentFloor,
            int candidateFloor,
            @Nullable LatLng crossFloorLandingLocation
    ) {
        return resolveBestRenderableDisplayLocation(
                currentDisplayedLocation,
                rawCandidateLocation,
                displayCandidateLocation,
                currentFloor,
                candidateFloor,
                crossFloorLandingLocation,
                false
        );
    }

    @Nullable
    static LatLng resolveBestRenderableDisplayLocation(
            @Nullable LatLng currentDisplayedLocation,
            @NonNull LatLng rawCandidateLocation,
            @Nullable LatLng displayCandidateLocation,
            int currentFloor,
            int candidateFloor,
            @Nullable LatLng crossFloorLandingLocation,
            boolean forceAcceptTransition
    ) {
        if (currentDisplayedLocation != null && currentFloor != candidateFloor) {
            if (isRenderablePoint(crossFloorLandingLocation, candidateFloor)) {
                return crossFloorLandingLocation;
            }
            if (displayCandidateLocation != null && isRenderablePoint(displayCandidateLocation, candidateFloor)) {
                return displayCandidateLocation;
            }
            return isRenderablePoint(rawCandidateLocation, candidateFloor)
                    ? rawCandidateLocation
                    : null;
        }
        if (forceAcceptTransition) {
            if (displayCandidateLocation != null && isRenderablePoint(displayCandidateLocation, candidateFloor)) {
                return displayCandidateLocation;
            }
            return isRenderablePoint(rawCandidateLocation, candidateFloor)
                    ? rawCandidateLocation
                    : null;
        }
        if (displayCandidateLocation != null
                && isRenderablePoint(displayCandidateLocation, candidateFloor)
                && !shouldHoldPositionForIllegalTransition(
                currentDisplayedLocation,
                displayCandidateLocation,
                currentFloor,
                candidateFloor,
                false
        )) {
            return displayCandidateLocation;
        }
        if (isRenderablePoint(rawCandidateLocation, candidateFloor)
                && !shouldHoldPositionForIllegalTransition(
                currentDisplayedLocation,
                rawCandidateLocation,
                currentFloor,
                candidateFloor,
                false
        )) {
            return rawCandidateLocation;
        }
        return constrainToRenderableSegmentPrefix(
                currentDisplayedLocation,
                rawCandidateLocation,
                currentFloor,
                candidateFloor,
                false
        );
    }

    @NonNull
    private static LatLng interpolate(
            @NonNull LatLng start,
            @NonNull LatLng end,
            double fraction
    ) {
        double safeFraction = Math.max(0.0, Math.min(1.0, fraction));
        return new LatLng(
                start.latitude + (end.latitude - start.latitude) * safeFraction,
                start.longitude + (end.longitude - start.longitude) * safeFraction
        );
    }
}
