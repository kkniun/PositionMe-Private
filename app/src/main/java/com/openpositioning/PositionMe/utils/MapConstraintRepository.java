package com.openpositioning.PositionMe.utils;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.google.android.gms.maps.model.LatLng;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Lightweight in-memory store for map-matching constraints extracted from map_shapes.
 */
public final class MapConstraintRepository {
    private static ConstraintSnapshot snapshot = ConstraintSnapshot.empty();

    private MapConstraintRepository() {
    }

    public static synchronized void updateCurrentFloorConstraints(
            @Nullable String venueId,
            @Nullable String floorKey,
            int floorIndex,
            @Nullable List<LatLng> venueOutline,
            @Nullable List<List<LatLng>> wallPolygons,
            @Nullable List<List<LatLng>> transitionPolygons
    ) {
        snapshot = new ConstraintSnapshot(
                venueId,
                floorKey,
                floorIndex,
                copyPoints(venueOutline),
                copyPolygons(wallPolygons),
                copyPolygons(transitionPolygons)
        );
    }

    public static synchronized void clear() {
        snapshot = ConstraintSnapshot.empty();
    }

    public static synchronized boolean hasWallConstraints() {
        return !snapshot.wallPolygons.isEmpty();
    }

    public static synchronized boolean isPointInsideWall(@NonNull LatLng point) {
        for (List<LatLng> wallPolygon : snapshot.wallPolygons) {
            if (pointInPolygon(point, wallPolygon)) {
                return true;
            }
        }
        return false;
    }

    public static synchronized boolean hasVenueOutline() {
        return snapshot.venueOutline.size() >= 3;
    }

    public static synchronized boolean isPointInsideVenueOutline(@NonNull LatLng point) {
        if (snapshot.venueOutline.size() < 3) {
            return true;
        }
        return pointInPolygon(point, snapshot.venueOutline);
    }

    /**
     * When false, floor-change gating is not applied (fail-open for barometer / PDR floor).
     */
    public static synchronized boolean hasTransitionConstraints() {
        return !snapshot.transitionPolygons.isEmpty();
    }

    /**
     * True if the point lies inside any stairs/lift/elevator polygon for the current map floor.
     */
    public static synchronized boolean isPointInsideTransitionZone(@NonNull LatLng point) {
        for (List<LatLng> poly : snapshot.transitionPolygons) {
            if (pointInPolygon(point, poly)) {
                return true;
            }
        }
        return false;
    }

    @NonNull
    private static List<LatLng> copyPoints(@Nullable List<LatLng> points) {
        if (points == null || points.isEmpty()) {
            return Collections.emptyList();
        }
        return new ArrayList<>(points);
    }

    @NonNull
    private static List<List<LatLng>> copyPolygons(@Nullable List<List<LatLng>> polygons) {
        if (polygons == null || polygons.isEmpty()) {
            return Collections.emptyList();
        }
        List<List<LatLng>> copied = new ArrayList<>(polygons.size());
        for (List<LatLng> polygon : polygons) {
            if (polygon != null && polygon.size() >= 3) {
                copied.add(new ArrayList<>(polygon));
            }
        }
        return copied;
    }

    /**
     * Ray casting algorithm.
     */
    private static boolean pointInPolygon(@NonNull LatLng point, @NonNull List<LatLng> polygon) {
        if (polygon.size() < 3) {
            return false;
        }

        int numCrossings = 0;
        for (int i = 0; i < polygon.size(); i++) {
            LatLng a = polygon.get(i);
            int j = i + 1;
            if (j >= polygon.size()) {
                j = 0;
            }
            LatLng b = polygon.get(j);
            if (crossingSegment(point, a, b)) {
                numCrossings++;
            }
        }
        return (numCrossings % 2 == 1);
    }

    private static boolean crossingSegment(@NonNull LatLng point, @NonNull LatLng a, @NonNull LatLng b) {
        double pointLng = point.longitude;
        double pointLat = point.latitude;
        double aLng = a.longitude;
        double aLat = a.latitude;
        double bLng = b.longitude;
        double bLat = b.latitude;

        if (aLat > bLat) {
            aLng = b.longitude;
            aLat = b.latitude;
            bLng = a.longitude;
            bLat = a.latitude;
        }
        if (pointLng < 0 || aLng < 0 || bLng < 0) {
            pointLng += 360;
            aLng += 360;
            bLng += 360;
        }
        if (pointLat == aLat || pointLat == bLat) {
            pointLat += 0.00000001;
        }

        if ((pointLat > bLat || pointLat < aLat) || (pointLng > Math.max(aLng, bLng))) {
            return false;
        } else if (pointLng < Math.min(aLng, bLng)) {
            return true;
        } else {
            double slope1 = (aLng != bLng) ? ((bLat - aLat) / (bLng - aLng)) : Double.POSITIVE_INFINITY;
            double slope2 = (aLng != pointLng) ? ((pointLat - aLat) / (pointLng - aLng)) : Double.POSITIVE_INFINITY;
            return (slope2 >= slope1);
        }
    }

    private static final class ConstraintSnapshot {
        @Nullable
        final String venueId;
        @Nullable
        final String floorKey;
        final int floorIndex;
        @NonNull
        final List<LatLng> venueOutline;
        @NonNull
        final List<List<LatLng>> wallPolygons;
        @NonNull
        final List<List<LatLng>> transitionPolygons;

        ConstraintSnapshot(
                @Nullable String venueId,
                @Nullable String floorKey,
                int floorIndex,
                @NonNull List<LatLng> venueOutline,
                @NonNull List<List<LatLng>> wallPolygons,
                @NonNull List<List<LatLng>> transitionPolygons
        ) {
            this.venueId = venueId;
            this.floorKey = floorKey;
            this.floorIndex = floorIndex;
            this.venueOutline = venueOutline;
            this.wallPolygons = wallPolygons;
            this.transitionPolygons = transitionPolygons;
        }

        @NonNull
        static ConstraintSnapshot empty() {
            return new ConstraintSnapshot(
                    null, null, -1,
                    Collections.emptyList(),
                    Collections.emptyList(),
                    Collections.emptyList()
            );
        }
    }
}
