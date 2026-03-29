package com.openpositioning.PositionMe.utils;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.ArrayList;
import java.util.List;

final class IndoorFloorKeyResolver {

    private IndoorFloorKeyResolver() {
    }

    @Nullable
    static Integer tryParseAbsoluteFloor(@Nullable String floorName) {
        return FloorKeyParser.tryParseFloorNameToAbsoluteFloor(floorName);
    }

    @NonNull
    static List<Integer> collectAvailableFloors(@NonNull List<String> floorKeys) {
        List<Integer> floors = new ArrayList<>(floorKeys.size());
        for (String floorKey : floorKeys) {
            Integer floor = tryParseAbsoluteFloor(floorKey);
            if (floor != null && !floors.contains(floor)) {
                floors.add(floor);
            }
        }
        return floors;
    }

    static int snapToNearestFloor(int requestedFloor, @NonNull List<Integer> availableFloors) {
        if (availableFloors.isEmpty()) {
            return requestedFloor;
        }
        int bestFloor = availableFloors.get(0);
        int bestDistance = Math.abs(bestFloor - requestedFloor);
        for (int floor : availableFloors) {
            int distance = Math.abs(floor - requestedFloor);
            if (distance < bestDistance || (distance == bestDistance && floor < bestFloor)) {
                bestFloor = floor;
                bestDistance = distance;
            }
        }
        return bestFloor;
    }
}
