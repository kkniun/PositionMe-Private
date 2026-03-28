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
            @Nullable List<List<LatLng>> liftPolygons,
            @Nullable List<List<LatLng>> stairsPolygons
    ) {
        snapshot = new ConstraintSnapshot(
                venueId,
                floorKey,
                floorIndex,
                copyPoints(venueOutline),
                copyPolygons(wallPolygons),
                copyPolygons(liftPolygons),
                copyPolygons(stairsPolygons)
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

    /**
     * True if the movement segment passes through a wall obstacle polygon (blocked interior),
     * including the case where an endpoint lies inside a wall polygon.
     * Uses planar lat/lng math; adequate for building-scale geometries.
     */
    public static synchronized boolean segmentIntersectsAnyWallPolygon(@NonNull LatLng a, @NonNull LatLng b) {
        if (!hasWallConstraints()) {
            return false;
        }
        for (List<LatLng> wall : snapshot.wallPolygons) {
            if (wall == null || wall.size() < 3) {
                continue;
            }
            if (segmentIntersectsWallObstacle(a, b, wall)) {
                return true;
            }
        }
        return false;
    }

    /**
     * True if a substantial portion of the segment interior lies outside the venue footprint (concave outlines).
     * Endpoints are checked separately by callers; mid-sample failures use a tolerance so boundary float noise
     * does not reject every short step.
     */
    public static synchronized boolean segmentLeavesVenueInterior(@NonNull LatLng a, @NonNull LatLng b) {
        if (!hasVenueOutline()) {
            return false;
        }
        final int samples = 24;
        int outsideMid = 0;
        int midCount = 0;
        for (int i = 1; i < samples; i++) {
            midCount++;
            double t = i / (double) samples;
            double lat = a.latitude + (b.latitude - a.latitude) * t;
            double lng = a.longitude + (b.longitude - a.longitude) * t;
            if (!isPointInsideVenueOutline(new LatLng(lat, lng))) {
                outsideMid++;
            }
        }
        if (midCount == 0) {
            return false;
        }
        int minOutsideToFail = Math.max(3, (int) Math.ceil(midCount * 0.22));
        return outsideMid >= minOutsideToFail;
    }

    private static boolean segmentIntersectsWallObstacle(
            @NonNull LatLng a,
            @NonNull LatLng b,
            @NonNull List<LatLng> wall
    ) {
        if (pointInPolygon(a, wall) || pointInPolygon(b, wall)) {
            return true;
        }
        int n = wall.size();
        for (int i = 0; i < n; i++) {
            LatLng p = wall.get(i);
            LatLng q = wall.get((i + 1) % n);
            if (segmentsIntersect(a, b, p, q)) {
                return true;
            }
        }
        return false;
    }

    private static final double SEGMENT_EPS = 1e-12;

    private static boolean segmentsIntersect(@NonNull LatLng p1, @NonNull LatLng p2, @NonNull LatLng p3, @NonNull LatLng p4) {
        double o1 = cross2(p1, p2, p3);
        double o2 = cross2(p1, p2, p4);
        double o3 = cross2(p3, p4, p1);
        double o4 = cross2(p3, p4, p2);

        if (Math.signum(o1) != Math.signum(o2) && Math.abs(o1) > SEGMENT_EPS && Math.abs(o2) > SEGMENT_EPS
                && Math.signum(o3) != Math.signum(o4) && Math.abs(o3) > SEGMENT_EPS && Math.abs(o4) > SEGMENT_EPS) {
            return true;
        }
        return pointOnSegment(p3, p1, p2) || pointOnSegment(p4, p1, p2)
                || pointOnSegment(p1, p3, p4) || pointOnSegment(p2, p3, p4);
    }

    private static double cross2(@NonNull LatLng o, @NonNull LatLng a, @NonNull LatLng b) {
        return (a.longitude - o.longitude) * (b.latitude - o.latitude)
                - (a.latitude - o.latitude) * (b.longitude - o.longitude);
    }

    private static boolean pointOnSegment(@NonNull LatLng p, @NonNull LatLng a, @NonNull LatLng b) {
        return Math.abs(cross2(a, b, p)) <= SEGMENT_EPS * 1e4
                && p.latitude >= Math.min(a.latitude, b.latitude) - 1e-12
                && p.latitude <= Math.max(a.latitude, b.latitude) + 1e-12
                && p.longitude >= Math.min(a.longitude, b.longitude) - 1e-12
                && p.longitude <= Math.max(a.longitude, b.longitude) + 1e-12;
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
        return !snapshot.liftPolygons.isEmpty() || !snapshot.stairsPolygons.isEmpty();
    }

    /**
     * True if the point lies inside any lift/stairs polygon for the current map floor.
     */
    public static synchronized boolean isPointInsideTransitionZone(@NonNull LatLng point) {
        return isPointInsideLiftZone(point) || isPointInsideStairsZone(point);
    }

    public static synchronized boolean isPointInsideLiftZone(@NonNull LatLng point) {
        for (List<LatLng> poly : snapshot.liftPolygons) {
            if (pointInPolygon(point, poly)) {
                return true;
            }
        }
        return false;
    }

    public static synchronized boolean isPointInsideStairsZone(@NonNull LatLng point) {
        for (List<LatLng> poly : snapshot.stairsPolygons) {
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
        final List<List<LatLng>> liftPolygons;
        @NonNull
        final List<List<LatLng>> stairsPolygons;

        ConstraintSnapshot(
                @Nullable String venueId,
                @Nullable String floorKey,
                int floorIndex,
                @NonNull List<LatLng> venueOutline,
                @NonNull List<List<LatLng>> wallPolygons,
                @NonNull List<List<LatLng>> liftPolygons,
                @NonNull List<List<LatLng>> stairsPolygons
        ) {
            this.venueId = venueId;
            this.floorKey = floorKey;
            this.floorIndex = floorIndex;
            this.venueOutline = venueOutline;
            this.wallPolygons = wallPolygons;
            this.liftPolygons = liftPolygons;
            this.stairsPolygons = stairsPolygons;
        }

        @NonNull
        static ConstraintSnapshot empty() {
            return new ConstraintSnapshot(
                    null, null, -1,
                    Collections.emptyList(),
                    Collections.emptyList(),
                    Collections.emptyList(),
                    Collections.emptyList()
            );
        }
    }
}
