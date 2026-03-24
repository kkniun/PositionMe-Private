package com.openpositioning.PositionMe.utils;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.google.android.gms.maps.model.LatLng;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Lightweight in-memory store for map-matching constraints extracted from map_shapes.
 */
public final class MapConstraintRepository {
    @Nullable
    private static String venueId;
    @NonNull
    private static List<LatLng> venueOutline = Collections.emptyList();
    private static int activeFloorIndex = -1;
    @NonNull
    private static final Map<Integer, String> floorKeysByIndex = new HashMap<>();
    @NonNull
    private static final Map<Integer, List<List<LatLng>>> wallsByFloor = new HashMap<>();
    @NonNull
    private static final Map<Integer, List<List<LatLng>>> stairsByFloor = new HashMap<>();
    @NonNull
    private static final Map<Integer, List<List<LatLng>>> liftsByFloor = new HashMap<>();
    @NonNull
    private static final Map<Integer, List<List<LatLng>>> transitionsByFloor = new HashMap<>();

    private MapConstraintRepository() {
    }

    public static synchronized void replaceVenueConstraints(
            @Nullable String venueId,
            @Nullable List<LatLng> venueOutline
    ) {
        MapConstraintRepository.venueId = venueId;
        MapConstraintRepository.venueOutline = copyPoints(venueOutline);
        activeFloorIndex = -1;
        floorKeysByIndex.clear();
        wallsByFloor.clear();
        stairsByFloor.clear();
        liftsByFloor.clear();
        transitionsByFloor.clear();
    }

    public static synchronized void setConstraintsForFloor(
            int floorIndex,
            @Nullable List<List<LatLng>> wallPolygons,
            @Nullable List<List<LatLng>> transitionPolygons
    ) {
        setConstraintsForFloor(floorIndex, null, wallPolygons, transitionPolygons);
    }

    public static synchronized void setConstraintsForFloor(
            int floorIndex,
            @Nullable String floorKey,
            @Nullable List<List<LatLng>> wallPolygons,
            @Nullable List<List<LatLng>> transitionPolygons
    ) {
        if (floorKey != null) {
            floorKeysByIndex.put(floorIndex, floorKey);
        }
        setPolygonsForFloor(wallsByFloor, floorIndex, wallPolygons);
        setPolygonsForFloor(transitionsByFloor, floorIndex, transitionPolygons);
        stairsByFloor.remove(floorIndex);
        liftsByFloor.remove(floorIndex);
    }

    public static synchronized void setStairsForFloor(
            int floorIndex,
            @Nullable List<List<LatLng>> stairPolygons
    ) {
        setPolygonsForFloor(stairsByFloor, floorIndex, stairPolygons);
        recomputeCombinedTransitionsForFloor(floorIndex);
    }

    public static synchronized void setLiftsForFloor(
            int floorIndex,
            @Nullable List<List<LatLng>> liftPolygons
    ) {
        setPolygonsForFloor(liftsByFloor, floorIndex, liftPolygons);
        recomputeCombinedTransitionsForFloor(floorIndex);
    }

    public static synchronized void setActiveFloor(int floorIndex) {
        activeFloorIndex = floorIndex;
    }

    public static synchronized void updateCurrentFloorConstraints(
            @Nullable String venueId,
            @Nullable String floorKey,
            int floorIndex,
            @Nullable List<LatLng> venueOutline,
            @Nullable List<List<LatLng>> wallPolygons,
            @Nullable List<List<LatLng>> transitionPolygons
    ) {
        replaceVenueConstraints(venueId, venueOutline);
        setConstraintsForFloor(floorIndex, floorKey, wallPolygons, transitionPolygons);
        setActiveFloor(floorIndex);
    }

    public static synchronized void clear() {
        replaceVenueConstraints(null, null);
    }

    public static synchronized boolean hasWallConstraints() {
        return hasWallConstraints(activeFloorIndex);
    }

    public static synchronized boolean hasWallConstraints(int floorIndex) {
        return !getStoredPolygons(wallsByFloor, floorIndex).isEmpty();
    }

    public static synchronized boolean isPointInsideWall(@NonNull LatLng point) {
        return isPointInsideWall(point, activeFloorIndex);
    }

    public static synchronized boolean isPointInsideWall(@NonNull LatLng point, int floorIndex) {
        for (List<LatLng> wallPolygon : getStoredPolygons(wallsByFloor, floorIndex)) {
            if (pointInPolygon(point, wallPolygon)) {
                return true;
            }
        }
        return false;
    }

    public static synchronized boolean doesPathIntersectWall(
            @NonNull LatLng start,
            @NonNull LatLng end,
            int floorIndex
    ) {
        return pathIntersectsAnyPolygon(start, end, getStoredPolygons(wallsByFloor, floorIndex));
    }

    public static synchronized boolean hasVenueOutline() {
        return venueOutline.size() >= 3;
    }

    public static synchronized boolean isPointInsideVenueOutline(@NonNull LatLng point) {
        if (venueOutline.size() < 3) {
            return true;
        }
        return pointInPolygon(point, venueOutline);
    }

    /**
     * When false, floor-change gating is not applied (fail-open for barometer / PDR floor).
     */
    public static synchronized boolean hasTransitionConstraints() {
        return hasTransitionConstraints(activeFloorIndex);
    }

    public static synchronized boolean hasTransitionConstraints(int floorIndex) {
        return !getStoredPolygons(transitionsByFloor, floorIndex).isEmpty();
    }

    public static synchronized boolean hasAnyConstraints() {
        return !wallsByFloor.isEmpty()
                || !stairsByFloor.isEmpty()
                || !liftsByFloor.isEmpty()
                || !transitionsByFloor.isEmpty();
    }

    /**
     * True if the point lies inside any stairs/lift/elevator polygon for the current map floor.
     */
    public static synchronized boolean isPointInsideTransitionZone(@NonNull LatLng point) {
        return isPointInsideTransitionZone(point, activeFloorIndex);
    }

    public static synchronized boolean isPointInsideTransitionZone(@NonNull LatLng point, int floorIndex) {
        for (List<LatLng> poly : getStoredPolygons(transitionsByFloor, floorIndex)) {
            if (pointInPolygon(point, poly)) {
                return true;
            }
        }
        return false;
    }

    public static synchronized boolean doesPathIntersectTransitionZone(
            @NonNull LatLng start,
            @NonNull LatLng end,
            int floorIndex
    ) {
        return pathIntersectsAnyPolygon(start, end, getStoredPolygons(transitionsByFloor, floorIndex));
    }

    public static synchronized boolean isPointInsideStairs(@NonNull LatLng point, int floorIndex) {
        for (List<LatLng> poly : getStoredPolygons(stairsByFloor, floorIndex)) {
            if (pointInPolygon(point, poly)) {
                return true;
            }
        }
        return false;
    }

    public static synchronized boolean doesPathIntersectStairs(
            @NonNull LatLng start,
            @NonNull LatLng end,
            int floorIndex
    ) {
        return pathIntersectsAnyPolygon(start, end, getStoredPolygons(stairsByFloor, floorIndex));
    }

    public static synchronized boolean isPointInsideLift(@NonNull LatLng point, int floorIndex) {
        for (List<LatLng> poly : getStoredPolygons(liftsByFloor, floorIndex)) {
            if (pointInPolygon(point, poly)) {
                return true;
            }
        }
        return false;
    }

    public static synchronized boolean doesPathIntersectLift(
            @NonNull LatLng start,
            @NonNull LatLng end,
            int floorIndex
    ) {
        return pathIntersectsAnyPolygon(start, end, getStoredPolygons(liftsByFloor, floorIndex));
    }

    @NonNull
    public static synchronized List<List<LatLng>> getWallsForFloor(int floorIndex) {
        return copyPolygons(getStoredPolygons(wallsByFloor, floorIndex));
    }

    @NonNull
    public static synchronized List<List<LatLng>> getTransitionsForFloor(int floorIndex) {
        return copyPolygons(getStoredPolygons(transitionsByFloor, floorIndex));
    }

    @NonNull
    public static synchronized List<List<LatLng>> getStairsForFloor(int floorIndex) {
        return copyPolygons(getStoredPolygons(stairsByFloor, floorIndex));
    }

    @NonNull
    public static synchronized List<List<LatLng>> getLiftsForFloor(int floorIndex) {
        return copyPolygons(getStoredPolygons(liftsByFloor, floorIndex));
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

    private static void setPolygonsForFloor(
            @NonNull Map<Integer, List<List<LatLng>>> target,
            int floorIndex,
            @Nullable List<List<LatLng>> polygons
    ) {
        List<List<LatLng>> copied = copyPolygons(polygons);
        if (copied.isEmpty()) {
            target.remove(floorIndex);
            return;
        }
        target.put(floorIndex, copied);
    }

    @NonNull
    private static List<List<LatLng>> getStoredPolygons(
            @NonNull Map<Integer, List<List<LatLng>>> source,
            int floorIndex
    ) {
        List<List<LatLng>> polygons = source.get(floorIndex);
        return polygons == null ? Collections.emptyList() : polygons;
    }

    private static void recomputeCombinedTransitionsForFloor(int floorIndex) {
        List<List<LatLng>> combined = new ArrayList<>();
        appendPolygons(combined, getStoredPolygons(stairsByFloor, floorIndex));
        appendPolygons(combined, getStoredPolygons(liftsByFloor, floorIndex));
        if (combined.isEmpty()) {
            transitionsByFloor.remove(floorIndex);
            return;
        }
        transitionsByFloor.put(floorIndex, combined);
    }

    private static void appendPolygons(
            @NonNull List<List<LatLng>> target,
            @NonNull List<List<LatLng>> polygons
    ) {
        for (List<LatLng> polygon : polygons) {
            if (polygon != null && polygon.size() >= 3) {
                target.add(new ArrayList<>(polygon));
            }
        }
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

    private static boolean pathIntersectsAnyPolygon(
            @NonNull LatLng start,
            @NonNull LatLng end,
            @NonNull List<List<LatLng>> polygons
    ) {
        for (List<LatLng> polygon : polygons) {
            if (segmentIntersectsPolygon(start, end, polygon)) {
                return true;
            }
        }
        return false;
    }

    private static boolean segmentIntersectsPolygon(
            @NonNull LatLng start,
            @NonNull LatLng end,
            @NonNull List<LatLng> polygon
    ) {
        if (polygon.size() < 3) {
            return false;
        }
        if (pointInPolygon(start, polygon) || pointInPolygon(end, polygon)) {
            return true;
        }
        for (int i = 0; i < polygon.size(); i++) {
            LatLng a = polygon.get(i);
            LatLng b = polygon.get((i + 1) % polygon.size());
            if (segmentsIntersect(start, end, a, b)) {
                return true;
            }
        }
        return false;
    }

    private static boolean segmentsIntersect(
            @NonNull LatLng a1,
            @NonNull LatLng a2,
            @NonNull LatLng b1,
            @NonNull LatLng b2
    ) {
        double o1 = orientation(a1, a2, b1);
        double o2 = orientation(a1, a2, b2);
        double o3 = orientation(b1, b2, a1);
        double o4 = orientation(b1, b2, a2);

        if (hasDifferentSigns(o1, o2) && hasDifferentSigns(o3, o4)) {
            return true;
        }

        return isPointOnSegment(b1, a1, a2)
                || isPointOnSegment(b2, a1, a2)
                || isPointOnSegment(a1, b1, b2)
                || isPointOnSegment(a2, b1, b2);
    }

    private static boolean hasDifferentSigns(double first, double second) {
        double epsilon = 1e-12;
        if (Math.abs(first) <= epsilon || Math.abs(second) <= epsilon) {
            return false;
        }
        return (first < 0.0 && second > 0.0) || (first > 0.0 && second < 0.0);
    }

    private static double orientation(@NonNull LatLng a, @NonNull LatLng b, @NonNull LatLng c) {
        return ((b.longitude - a.longitude) * (c.latitude - a.latitude))
                - ((b.latitude - a.latitude) * (c.longitude - a.longitude));
    }

    private static boolean isPointOnSegment(
            @NonNull LatLng point,
            @NonNull LatLng segmentStart,
            @NonNull LatLng segmentEnd
    ) {
        double epsilon = 1e-12;
        double cross = orientation(segmentStart, segmentEnd, point);
        if (Math.abs(cross) > epsilon) {
            return false;
        }
        return point.longitude >= Math.min(segmentStart.longitude, segmentEnd.longitude) - epsilon
                && point.longitude <= Math.max(segmentStart.longitude, segmentEnd.longitude) + epsilon
                && point.latitude >= Math.min(segmentStart.latitude, segmentEnd.latitude) - epsilon
                && point.latitude <= Math.max(segmentStart.latitude, segmentEnd.latitude) + epsilon;
    }

}
