package com.openpositioning.PositionMe.utils;

import androidx.annotation.Nullable;

public final class MapConstraintReadiness {

    private MapConstraintReadiness() {
    }

    public static boolean hasFloorLevelActiveMapMatchingConstraints(
            @Nullable String selectedVenueId,
            boolean hasVectorMapShapes,
            int currentFloor
    ) {
        return selectedVenueId != null
                && !selectedVenueId.trim().isEmpty()
                && hasVectorMapShapes
                && MapConstraintRepository.hasKnownFloor(currentFloor)
                && MapConstraintRepository.hasWallConstraints(currentFloor);
    }

    public static boolean hasFloorLevelParticleWallReadiness(
            boolean coordinateConverterReady,
            @Nullable Integer floor
    ) {
        return coordinateConverterReady
                && floor != null
                && MapConstraintRepository.hasKnownFloor(floor)
                && MapConstraintRepository.hasWallConstraints(floor);
    }
}
