package com.openpositioning.PositionMe.sensors;

import androidx.annotation.NonNull;

import com.google.android.gms.maps.model.LatLng;
import com.openpositioning.PositionMe.utils.UtilFunctions;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * 对连续的 WiFi 绝对定位结果做轻量稳定化，减少单次指纹跳点直接扰动融合状态。
 */
public final class WifiObservationSmoother {

    private static final int MAX_WINDOW_SIZE = 5;
    private static final long WINDOW_DURATION_MS = 5_000L;
    private static final double CLUSTER_RADIUS_METERS = 8.0;
    private static final double HARD_RESET_DISTANCE_METERS = 18.0;
    private static final int MIN_STABLE_OBSERVATIONS = 2;

    private final ArrayDeque<Observation> observations = new ArrayDeque<>();

    @NonNull
    public SmoothedObservation observe(@NonNull LatLng location, int floor, long timestampMillis) {
        prune(timestampMillis);

        Observation last = observations.peekLast();
        if (last != null) {
            double jumpMeters = UtilFunctions.distanceBetweenPoints(last.location, location);
            if (jumpMeters > HARD_RESET_DISTANCE_METERS && last.floor != floor) {
                observations.clear();
            }
        }

        observations.addLast(new Observation(location, floor, timestampMillis));
        while (observations.size() > MAX_WINDOW_SIZE) {
            observations.removeFirst();
        }

        List<Observation> cluster = buildRecentCluster(location, floor);
        if (cluster.size() < MIN_STABLE_OBSERVATIONS) {
            return new SmoothedObservation(location, floor, 1);
        }

        double latSum = 0d;
        double lngSum = 0d;
        for (Observation observation : cluster) {
            latSum += observation.location.latitude;
            lngSum += observation.location.longitude;
        }

        return new SmoothedObservation(
                new LatLng(latSum / cluster.size(), lngSum / cluster.size()),
                floor,
                cluster.size()
        );
    }

    public void reset() {
        observations.clear();
    }

    private void prune(long timestampMillis) {
        while (!observations.isEmpty()
                && timestampMillis - observations.peekFirst().timestampMillis > WINDOW_DURATION_MS) {
            observations.removeFirst();
        }
    }

    @NonNull
    private List<Observation> buildRecentCluster(@NonNull LatLng anchorLocation, int floor) {
        ArrayList<Observation> cluster = new ArrayList<>();
        Iterator<Observation> descendingIterator = observations.descendingIterator();
        while (descendingIterator.hasNext()) {
            Observation observation = descendingIterator.next();
            if (observation.floor != floor) {
                continue;
            }
            double distanceMeters = UtilFunctions.distanceBetweenPoints(
                    observation.location,
                    anchorLocation
            );
            if (distanceMeters <= CLUSTER_RADIUS_METERS) {
                cluster.add(observation);
            }
        }
        return cluster;
    }

    public static final class SmoothedObservation {
        private final LatLng location;
        private final int floor;
        private final int supportCount;

        SmoothedObservation(@NonNull LatLng location, int floor, int supportCount) {
            this.location = location;
            this.floor = floor;
            this.supportCount = supportCount;
        }

        @NonNull
        public LatLng getLocation() {
            return location;
        }

        public int getFloor() {
            return floor;
        }

        public int getSupportCount() {
            return supportCount;
        }
    }

    private static final class Observation {
        private final LatLng location;
        private final int floor;
        private final long timestampMillis;

        private Observation(@NonNull LatLng location, int floor, long timestampMillis) {
            this.location = location;
            this.floor = floor;
            this.timestampMillis = timestampMillis;
        }
    }
}
