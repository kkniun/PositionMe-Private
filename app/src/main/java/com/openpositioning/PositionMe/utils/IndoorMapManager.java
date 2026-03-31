package com.openpositioning.PositionMe.utils;

import android.graphics.Color;
import android.util.Log;

import androidx.annotation.Nullable;

import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Polygon;
import com.google.android.gms.maps.model.PolygonOptions;
import com.google.android.gms.maps.model.Polyline;
import com.google.android.gms.maps.model.PolylineOptions;
import com.openpositioning.PositionMe.data.remote.FloorplanApiClient;
import com.openpositioning.PositionMe.sensors.SensorFusion;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * Manages indoor floor map display for all supported buildings
 * (Nucleus, Library, Murchison). Uses vector shape data from the floorplan API
 * to dynamically draw walls, rooms, and other indoor features on the Google Map.
 * Provides unified floor indexing, floor switching, and building detection.
 *
 * @see BuildingPolygon Describes the bounds of buildings and the methods to check if a point is
 *                      within a building
 * @see FloorplanApiClient.FloorShapes Per-floor vector shape data
 */
public class IndoorMapManager {

    private static final String TAG = "IndoorMapManager";

    /** Building identifiers for tracking which building the user is in. */
    public static final int BUILDING_NONE = 0;
    public static final int BUILDING_NUCLEUS = 1;
    public static final int BUILDING_LIBRARY = 2;
    public static final int BUILDING_MURCHISON = 3;

    private GoogleMap gMap;
    private LatLng currentLocation;
    private boolean isIndoorMapSet = false;
    private int currentFloor;
    private int currentBuilding = BUILDING_NONE;
    private float floorHeight;

    // Vector shapes currently drawn on the map (cleared on floor switch or exit)
    private final List<Polygon> drawnPolygons = new ArrayList<>();
    private final List<Polyline> drawnPolylines = new ArrayList<>();

    // Per-floor vector shape data for the current building
    private List<FloorplanApiClient.FloorShapes> currentFloorShapes;

    // Average floor heights per building (meters), used for barometric auto-floor
    public static final float NUCLEUS_FLOOR_HEIGHT = 4.2F;
    public static final float LIBRARY_FLOOR_HEIGHT = 3.6F;
    public static final float MURCHISON_FLOOR_HEIGHT = 4.0F;

    // Colours for different indoor feature types
    private static final int WALL_STROKE = Color.argb(200, 80, 80, 80);
    private static final int ROOM_STROKE = Color.argb(180, 33, 150, 243);
    private static final int ROOM_FILL = Color.argb(40, 33, 150, 243);
    private static final int STAIRS_STROKE = Color.argb(220, 255, 152, 0);
    private static final int STAIRS_FILL = Color.argb(70, 255, 152, 0);
    private static final int LIFT_STROKE = Color.argb(220, 0, 137, 123);
    private static final int LIFT_FILL = Color.argb(70, 0, 137, 123);
    private static final int DEFAULT_STROKE = Color.argb(150, 100, 100, 100);
    private static final double ROUTE_NODE_SNAP_METERS = 4.0;
    private static final double MAX_ROUTE_EDGE_METERS = 35.0;
    private static final double INTERIOR_NODE_PULL_RATIO = 0.28;

    private int cachedRouteFloor = Integer.MIN_VALUE;
    private List<LatLng> cachedRouteNodes = new ArrayList<>();

    /**
     * Constructor to set the map instance.
     *
     * @param map the map on which the indoor floor map shapes are drawn
     */
    public IndoorMapManager(GoogleMap map) {
        this.gMap = map;
    }

    /**
     * Updates the current location of the user and displays the indoor map
     * if the user is in a building with indoor maps available.
     *
     * @param currentLocation new location of user
     */
    public void setCurrentLocation(LatLng currentLocation) {
        this.currentLocation = currentLocation;
        setBuildingOverlay();
    }

    /**
     * Returns the current building's floor height.
     *
     * @return the floor height of the current building the user is in
     */
    public float getFloorHeight() {
        return floorHeight;
    }

    /**
     * Returns whether an indoor floor map is currently being displayed.
     *
     * @return true if an indoor map is visible to the user, false otherwise
     */
    public boolean getIsIndoorMapSet() {
        return isIndoorMapSet;
    }

    /**
     * Returns the identifier of the building the user is currently in.
     *
     * @return one of {@link #BUILDING_NONE}, {@link #BUILDING_NUCLEUS},
     *         {@link #BUILDING_LIBRARY}, or {@link #BUILDING_MURCHISON}
     */
    public int getCurrentBuilding() {
        return currentBuilding;
    }

    /**
     * Returns the current floor index being displayed.
     *
     * @return the current floor index in the active building's floor list
     */
    public int getCurrentFloor() {
        return currentFloor;
    }

    /**
     * Returns the logical floor number used by the auto-floor logic
     * (e.g. G=0, LG=-1, 1=1).
     */
    public int getCurrentLogicalFloor() {
        Integer parsed = parseLogicalFloor(currentFloor);
        if (parsed != null) {
            return parsed;
        }
        return currentFloor - getAutoFloorBias();
    }

    /**
     * Returns the display name for the current floor (e.g. "LG", "G", "1").
     * Falls back to the numeric index if no display name is available.
     *
     * @return human-readable floor label
     */
    public String getCurrentFloorDisplayName() {
        if (currentFloorShapes != null
                && currentFloor >= 0
                && currentFloor < currentFloorShapes.size()) {
            return currentFloorShapes.get(currentFloor).getDisplayName();
        }
        return String.valueOf(currentFloor);
    }

    /**
     * Returns the auto-floor bias for the current building. Buildings with a
     * lower-ground floor at index 0 need a +1 bias so that WiFi/barometric
     * floor 0 (ground) maps to the correct floor index.
     *
     * @return the floor index offset for auto-floor conversion
     */
    public int getAutoFloorBias() {
        switch (currentBuilding) {
            case BUILDING_NUCLEUS:
            case BUILDING_MURCHISON:
                return 1; // LG at index 0, so G = index 1
            case BUILDING_LIBRARY:
            default:
                return 0; // G at index 0
        }
    }

    /**
     * Sets the floor to display. When called from auto-floor, the floor number
     * is a logical floor (0=G, -1=LG, 1=Floor 1, etc.) and the building bias
     * is applied. When called manually, the floor number is the direct index.
     *
     * @param newFloor  the floor the user is at
     * @param autoFloor true if called by auto-floor feature
     */
    public void setCurrentFloor(int newFloor, boolean autoFloor) {
        if (currentFloorShapes == null || currentFloorShapes.isEmpty()) return;

        if (autoFloor) {
            Integer mappedFloor = resolveFloorIndexForLogicalFloor(newFloor);
            if (mappedFloor != null) {
                newFloor = mappedFloor;
            } else {
                newFloor += getAutoFloorBias();
            }
        }

        if (newFloor >= 0 && newFloor < currentFloorShapes.size()
                && newFloor != this.currentFloor) {
            this.currentFloor = newFloor;
            drawFloorShapes(newFloor);
        }
    }

    /**
     * Increments the current floor and changes to a higher floor's map
     * (if a higher floor exists).
     */
    public void increaseFloor() {
        this.setCurrentFloor(currentFloor + 1, false);
    }

    /**
     * Decrements the current floor and changes to the lower floor's map
     * (if a lower floor exists).
     */
    public void decreaseFloor() {
        this.setCurrentFloor(currentFloor - 1, false);
    }

    /**
     * Clips a candidate trajectory segment to the last legal point before a wall intersection.
     */
    public LatLng constrainToLegalPath(LatLng previousLocation, LatLng candidateLocation) {
        if (!isIndoorMapSet
                || currentFloorShapes == null
                || previousLocation == null
                || candidateLocation == null) {
            return candidateLocation;
        }

        if (!isBlocked(previousLocation, candidateLocation)) {
            return candidateLocation;
        }

        double low = 0d;
        double high = 1d;
        LatLng safe = previousLocation;
        for (int i = 0; i < 14; i++) {
            double mid = (low + high) / 2d;
            LatLng probe = interpolate(previousLocation, candidateLocation, mid);
            if (isBlocked(previousLocation, probe)) {
                high = mid;
            } else {
                low = mid;
                safe = probe;
            }
        }
        return safe;
    }

    /**
     * Builds a display polyline that keeps the live user position untouched while rerouting
     * only the line segments that would otherwise cross walls.
     */
    public List<LatLng> buildLegalDisplayPath(@Nullable List<LatLng> rawHistory) {
        if (rawHistory == null || rawHistory.isEmpty()) {
            return Collections.emptyList();
        }
        if (!isIndoorMapSet || currentFloorShapes == null
                || currentFloor < 0 || currentFloor >= currentFloorShapes.size()) {
            return new ArrayList<>(rawHistory);
        }

        List<LatLng> routedPath = new ArrayList<>();
        LatLng previous = rawHistory.get(0);
        routedPath.add(previous);

        for (int i = 1; i < rawHistory.size(); i++) {
            LatLng current = rawHistory.get(i);
            appendLegalSegment(routedPath, previous, current);
            previous = current;
        }
        return routedPath;
    }

    /**
     * Returns true if the point lies inside or close to the requested indoor feature type.
     */
    public boolean isNearIndoorFeature(LatLng point, String indoorType, double radiusMeters) {
        if (!isIndoorMapSet || currentFloorShapes == null || point == null) {
            return false;
        }

        FloorplanApiClient.FloorShapes floor = currentFloorShapes.get(currentFloor);
        for (FloorplanApiClient.MapShapeFeature feature : floor.getFeatures()) {
            if (!indoorType.equals(feature.getIndoorType())) {
                continue;
            }

            boolean polygonGeometry = isPolygonGeometry(feature.getGeometryType());
            for (List<LatLng> part : feature.getParts()) {
                if (part.size() < 2) {
                    continue;
                }
                if (polygonGeometry && part.size() >= 3 && BuildingPolygon.pointInPolygon(point, part)) {
                    return true;
                }
                if (distancePointToPathMeters(point, part, polygonGeometry) <= radiusMeters) {
                    return true;
                }
            }
        }
        return false;
    }

    private void appendLegalSegment(List<LatLng> routedPath, LatLng start, LatLng end) {
        if (start == null || end == null) {
            return;
        }

        if (!isBlocked(start, end) && !isInsideWall(end)) {
            addDistinctPoint(routedPath, end);
            return;
        }

        List<LatLng> detour = routeShortestLegalPath(start, end);
        if (detour.size() >= 2) {
            for (int i = 1; i < detour.size(); i++) {
                addDistinctPoint(routedPath, detour.get(i));
            }
            return;
        }

        LatLng safe = constrainToLegalPath(start, end);
        if (safe != null && !samePoint(routedPath.get(routedPath.size() - 1), safe)) {
            addDistinctPoint(routedPath, safe);
        }
    }

    private List<LatLng> routeShortestLegalPath(LatLng rawStart, LatLng rawEnd) {
        List<LatLng> routeNodes = getRouteNodes();
        LatLng start = snapRouteEndpoint(rawStart, routeNodes);
        LatLng end = snapRouteEndpoint(rawEnd, routeNodes);
        if (start == null || end == null) {
            return Collections.emptyList();
        }
        if (!isBlocked(start, end) && !isInsideWall(end)) {
            List<LatLng> direct = new ArrayList<>();
            direct.add(start);
            direct.add(end);
            return direct;
        }

        List<LatLng> graphNodes = new ArrayList<>(routeNodes.size() + 2);
        graphNodes.add(start);
        graphNodes.add(end);
        graphNodes.addAll(routeNodes);

        int nodeCount = graphNodes.size();
        double[] distance = new double[nodeCount];
        int[] previous = new int[nodeCount];
        boolean[] visited = new boolean[nodeCount];
        for (int i = 0; i < nodeCount; i++) {
            distance[i] = Double.POSITIVE_INFINITY;
            previous[i] = -1;
        }
        distance[0] = 0d;

        for (int iteration = 0; iteration < nodeCount; iteration++) {
            int currentIndex = -1;
            double currentDistance = Double.POSITIVE_INFINITY;
            for (int i = 0; i < nodeCount; i++) {
                if (!visited[i] && distance[i] < currentDistance) {
                    currentDistance = distance[i];
                    currentIndex = i;
                }
            }

            if (currentIndex < 0 || currentIndex == 1) {
                break;
            }

            visited[currentIndex] = true;
            for (int nextIndex = 0; nextIndex < nodeCount; nextIndex++) {
                if (nextIndex == currentIndex || visited[nextIndex]) {
                    continue;
                }

                LatLng from = graphNodes.get(currentIndex);
                LatLng to = graphNodes.get(nextIndex);
                if (!isRouteEdgeAllowed(from, to, currentIndex <= 1 || nextIndex <= 1)) {
                    continue;
                }

                double edgeDistance = UtilFunctions.distanceBetweenPoints(from, to);
                double alternativeDistance = distance[currentIndex] + edgeDistance;
                if (alternativeDistance < distance[nextIndex]) {
                    distance[nextIndex] = alternativeDistance;
                    previous[nextIndex] = currentIndex;
                }
            }
        }

        if (Double.isInfinite(distance[1])) {
            return Collections.emptyList();
        }

        List<LatLng> path = new ArrayList<>();
        for (int index = 1; index >= 0; index = previous[index]) {
            path.add(graphNodes.get(index));
            if (index == 0) {
                break;
            }
        }
        Collections.reverse(path);
        return path;
    }

    @Nullable
    private LatLng snapRouteEndpoint(LatLng rawPoint, List<LatLng> routeNodes) {
        if (!isInsideWall(rawPoint)) {
            return rawPoint;
        }

        LatLng nearest = null;
        double nearestDistance = Double.MAX_VALUE;
        for (LatLng candidate : routeNodes) {
            if (candidate == null || isInsideWall(candidate)) {
                continue;
            }
            double distance = UtilFunctions.distanceBetweenPoints(rawPoint, candidate);
            if (distance < nearestDistance) {
                nearestDistance = distance;
                nearest = candidate;
            }
        }

        if (nearestDistance <= ROUTE_NODE_SNAP_METERS || nearest != null) {
            return nearest;
        }
        return null;
    }

    private boolean isRouteEdgeAllowed(LatLng start, LatLng end, boolean allowLongEdge) {
        if (start == null || end == null || isInsideWall(start) || isInsideWall(end)) {
            return false;
        }

        double distance = UtilFunctions.distanceBetweenPoints(start, end);
        if (!allowLongEdge && distance > MAX_ROUTE_EDGE_METERS) {
            return false;
        }
        return !isBlocked(start, end);
    }

    private List<LatLng> getRouteNodes() {
        if (cachedRouteFloor == currentFloor && !cachedRouteNodes.isEmpty()) {
            return cachedRouteNodes;
        }

        List<LatLng> nodes = new ArrayList<>();
        FloorplanApiClient.FloorShapes floor = currentFloorShapes.get(currentFloor);
        for (FloorplanApiClient.MapShapeFeature feature : floor.getFeatures()) {
            if ("wall".equals(feature.getIndoorType())) {
                continue;
            }

            boolean polygonGeometry = isPolygonGeometry(feature.getGeometryType());
            for (List<LatLng> part : feature.getParts()) {
                if (part == null || part.isEmpty()) {
                    continue;
                }

                if (polygonGeometry && part.size() >= 3) {
                    LatLng centroid = computeCentroid(part);
                    addRouteNode(nodes, centroid);
                    for (LatLng vertex : part) {
                        addRouteNode(nodes, interpolate(vertex, centroid, INTERIOR_NODE_PULL_RATIO));
                    }
                } else {
                    for (LatLng point : part) {
                        addRouteNode(nodes, point);
                    }
                }
            }
        }

        cachedRouteFloor = currentFloor;
        cachedRouteNodes = nodes;
        return cachedRouteNodes;
    }

    private void addRouteNode(List<LatLng> nodes, @Nullable LatLng candidate) {
        if (candidate == null || isInsideWall(candidate)) {
            return;
        }

        for (LatLng existing : nodes) {
            if (UtilFunctions.distanceBetweenPoints(existing, candidate) < 0.75) {
                return;
            }
        }
        nodes.add(candidate);
    }

    private void addDistinctPoint(List<LatLng> points, LatLng candidate) {
        if (points.isEmpty() || !samePoint(points.get(points.size() - 1), candidate)) {
            points.add(candidate);
        }
    }

    private boolean samePoint(LatLng a, LatLng b) {
        return UtilFunctions.distanceBetweenPoints(a, b) < 0.15;
    }

    private LatLng computeCentroid(List<LatLng> polygon) {
        double latitudeSum = 0d;
        double longitudeSum = 0d;
        for (LatLng point : polygon) {
            latitudeSum += point.latitude;
            longitudeSum += point.longitude;
        }
        return new LatLng(latitudeSum / polygon.size(), longitudeSum / polygon.size());
    }

    /**
     * Sets the map overlay for the building if the user's current location is
     * inside a building and the overlay is not already set. Removes the overlay
     * if the user leaves all buildings.
     *
     * <p>Detection priority: floorplan API real polygon outlines first,
     * then legacy hard-coded rectangular boundaries as fallback.</p>
     */
    private void setBuildingOverlay() {
        try {
            int detected = detectCurrentBuilding();
            boolean inAnyBuilding = (detected != BUILDING_NONE);

            if (inAnyBuilding && !isIndoorMapSet) {
                currentBuilding = detected;
                String apiName;

                switch (detected) {
                    case BUILDING_NUCLEUS:
                        apiName = "nucleus_building";
                        currentFloor = 1;
                        floorHeight = NUCLEUS_FLOOR_HEIGHT;
                        break;
                    case BUILDING_LIBRARY:
                        apiName = "library";
                        currentFloor = 0;
                        floorHeight = LIBRARY_FLOOR_HEIGHT;
                        break;
                    case BUILDING_MURCHISON:
                        apiName = "murchison_house";
                        currentFloor = 1;
                        floorHeight = MURCHISON_FLOOR_HEIGHT;
                        break;
                    default:
                        return;
                }

                // Load floor shapes from cached API data
                FloorplanApiClient.BuildingInfo building =
                        SensorFusion.getInstance().getFloorplanBuilding(apiName);
                if (building != null) {
                    currentFloorShapes = building.getFloorShapesList();
                }

                if (currentFloorShapes != null && !currentFloorShapes.isEmpty()) {
                    Integer groundFloorIndex = resolveFloorIndexForLogicalFloor(0);
                    if (groundFloorIndex != null) {
                        currentFloor = groundFloorIndex;
                    }
                    drawFloorShapes(currentFloor);
                    isIndoorMapSet = true;
                }

            } else if (!inAnyBuilding && isIndoorMapSet) {
                clearDrawnShapes();
                isIndoorMapSet = false;
                currentBuilding = BUILDING_NONE;
                currentFloor = 0;
                currentFloorShapes = null;
                invalidateRouteCache();
            }
        } catch (Exception ex) {
            Log.e(TAG, "Error with overlay: " + ex.toString());
        }
    }

    /**
     * Draws all vector shapes for the given floor index on the Google Map.
     * Clears any previously drawn shapes before drawing the new floor.
     *
     * @param floorIndex the floor index (0-based, matching FloorShapes list order)
     */
    private void drawFloorShapes(int floorIndex) {
        clearDrawnShapes();
        invalidateRouteCache();

        if (currentFloorShapes == null || floorIndex < 0
                || floorIndex >= currentFloorShapes.size()) return;

        FloorplanApiClient.FloorShapes floor = currentFloorShapes.get(floorIndex);
        for (FloorplanApiClient.MapShapeFeature feature : floor.getFeatures()) {
            String geoType = feature.getGeometryType();
            String indoorType = feature.getIndoorType();

            if ("MultiPolygon".equals(geoType) || "Polygon".equals(geoType)) {
                for (List<LatLng> ring : feature.getParts()) {
                    if (ring.size() < 3) continue;
                    Polygon p = gMap.addPolygon(new PolygonOptions()
                            .addAll(ring)
                            .strokeColor(getStrokeColor(indoorType))
                            .strokeWidth(5f)
                            .fillColor(getFillColor(indoorType)));
                    drawnPolygons.add(p);
                }
            } else if ("MultiLineString".equals(geoType)
                    || "LineString".equals(geoType)) {
                for (List<LatLng> line : feature.getParts()) {
                    if (line.size() < 2) continue;
                    Polyline pl = gMap.addPolyline(new PolylineOptions()
                            .addAll(line)
                            .color(getStrokeColor(indoorType))
                            .width(6f));
                    drawnPolylines.add(pl);
                }
            }
        }
    }

    /**
     * Removes all vector shapes currently drawn on the map.
     */
    private void clearDrawnShapes() {
        for (Polygon p : drawnPolygons) p.remove();
        for (Polyline p : drawnPolylines) p.remove();
        drawnPolygons.clear();
        drawnPolylines.clear();
    }

    /**
     * Returns the stroke colour for a given indoor feature type.
     *
     * @param indoorType the indoor_type property value
     * @return ARGB colour value
     */
    private int getStrokeColor(String indoorType) {
        if ("wall".equals(indoorType)) return WALL_STROKE;
        if ("room".equals(indoorType)) return ROOM_STROKE;
        if ("stairs".equals(indoorType)) return STAIRS_STROKE;
        if ("lift".equals(indoorType)) return LIFT_STROKE;
        return DEFAULT_STROKE;
    }

    /**
     * Returns the fill colour for a given indoor feature type.
     *
     * @param indoorType the indoor_type property value
     * @return ARGB colour value
     */
    private int getFillColor(String indoorType) {
        if ("room".equals(indoorType)) return ROOM_FILL;
        if ("stairs".equals(indoorType)) return STAIRS_FILL;
        if ("lift".equals(indoorType)) return LIFT_FILL;
        return Color.TRANSPARENT;
    }

    /**
     * Detects which building the user is currently in.
     * Checks floorplan API outline polygons first; falls back to legacy
     * hard-coded rectangular boundaries if no API match is found.
     *
     * @return building type constant, or {@link #BUILDING_NONE}
     */
    private int detectCurrentBuilding() {
        // Phase 1: API real polygon outlines
        List<FloorplanApiClient.BuildingInfo> apiBuildings =
                SensorFusion.getInstance().getFloorplanBuildings();
        for (FloorplanApiClient.BuildingInfo building : apiBuildings) {
            List<LatLng> outline = building.getOutlinePolygon();
            if (outline != null && outline.size() >= 3
                    && BuildingPolygon.pointInPolygon(currentLocation, outline)) {
                int type = resolveBuildingType(building.getName());
                if (type != BUILDING_NONE) return type;
            }
        }

        // Phase 2: legacy hard-coded fallback
        if (BuildingPolygon.inNucleus(currentLocation)) return BUILDING_NUCLEUS;
        if (BuildingPolygon.inLibrary(currentLocation)) return BUILDING_LIBRARY;
        if (BuildingPolygon.inMurchison(currentLocation)) return BUILDING_MURCHISON;

        return BUILDING_NONE;
    }

    /**
     * Maps a floorplan API building name to a building type constant.
     *
     * @param apiName building name from API (e.g. "nucleus_building")
     * @return building type constant, or {@link #BUILDING_NONE} if unrecognised
     */
    private int resolveBuildingType(String apiName) {
        if (apiName == null) return BUILDING_NONE;
        switch (apiName) {
            case "nucleus_building": return BUILDING_NUCLEUS;
            case "murchison_house":  return BUILDING_MURCHISON;
            case "library":          return BUILDING_LIBRARY;
            default:                 return BUILDING_NONE;
        }
    }

    /**
     * Draws green polyline indicators around all buildings with available
     * indoor floor maps. Uses floorplan API outlines when available,
     * falls back to legacy hard-coded polygons otherwise.
     */
    public void setIndicationOfIndoorMap() {
        List<FloorplanApiClient.BuildingInfo> apiBuildings =
                SensorFusion.getInstance().getFloorplanBuildings();

        boolean nucleusDrawn = false, libraryDrawn = false, murchisonDrawn = false;

        // Phase 1: draw API outlines
        for (FloorplanApiClient.BuildingInfo building : apiBuildings) {
            List<LatLng> outline = building.getOutlinePolygon();
            if (outline == null || outline.size() < 3) continue;

            List<LatLng> closed = new ArrayList<>(outline);
            closed.add(closed.get(0));
            gMap.addPolyline(new PolylineOptions().color(Color.GREEN).addAll(closed));

            switch (building.getName()) {
                case "nucleus_building": nucleusDrawn = true; break;
                case "library":          libraryDrawn = true; break;
                case "murchison_house":  murchisonDrawn = true; break;
            }
        }

        // Phase 2: fallback for buildings not covered by API
        if (!nucleusDrawn) {
            List<LatLng> pts = new ArrayList<>(BuildingPolygon.NUCLEUS_POLYGON);
            pts.add(pts.get(0));
            gMap.addPolyline(new PolylineOptions().color(Color.GREEN).addAll(pts));
        }
        if (!libraryDrawn) {
            List<LatLng> pts = new ArrayList<>(BuildingPolygon.LIBRARY_POLYGON);
            pts.add(pts.get(0));
            gMap.addPolyline(new PolylineOptions().color(Color.GREEN).addAll(pts));
        }
        if (!murchisonDrawn) {
            List<LatLng> pts = new ArrayList<>(BuildingPolygon.MURCHISON_POLYGON);
            pts.add(pts.get(0));
            gMap.addPolyline(new PolylineOptions().color(Color.GREEN).addAll(pts));
        }
    }

    private boolean isBlocked(LatLng start, LatLng end) {
        return isInsideWall(end) || segmentCrossesWall(start, end);
    }

    private boolean isInsideWall(LatLng point) {
        FloorplanApiClient.FloorShapes floor = currentFloorShapes.get(currentFloor);
        for (FloorplanApiClient.MapShapeFeature feature : floor.getFeatures()) {
            if (!"wall".equals(feature.getIndoorType())
                    || !isPolygonGeometry(feature.getGeometryType())) {
                continue;
            }

            for (List<LatLng> part : feature.getParts()) {
                if (part.size() >= 3 && BuildingPolygon.pointInPolygon(point, part)) {
                    return true;
                }
            }
        }
        return false;
    }

    private boolean segmentCrossesWall(LatLng start, LatLng end) {
        FloorplanApiClient.FloorShapes floor = currentFloorShapes.get(currentFloor);
        for (FloorplanApiClient.MapShapeFeature feature : floor.getFeatures()) {
            if (!"wall".equals(feature.getIndoorType())) {
                continue;
            }

            boolean polygonGeometry = isPolygonGeometry(feature.getGeometryType());
            for (List<LatLng> part : feature.getParts()) {
                if (part.size() < 2) {
                    continue;
                }

                if (pathIntersects(start, end, part, polygonGeometry)) {
                    return true;
                }
            }
        }
        return false;
    }

    private boolean isPolygonGeometry(String geometryType) {
        return "Polygon".equals(geometryType) || "MultiPolygon".equals(geometryType);
    }

    private boolean pathIntersects(LatLng start, LatLng end, List<LatLng> path, boolean closed) {
        int segmentCount = closed ? path.size() : path.size() - 1;
        for (int i = 0; i < segmentCount; i++) {
            LatLng segStart = path.get(i);
            LatLng segEnd = path.get((i + 1) % path.size());
            if (segmentsIntersect(start, end, segStart, segEnd)) {
                return true;
            }
        }
        return false;
    }

    private boolean segmentsIntersect(LatLng a, LatLng b, LatLng c, LatLng d) {
        double o1 = orientation(a, b, c);
        double o2 = orientation(a, b, d);
        double o3 = orientation(c, d, a);
        double o4 = orientation(c, d, b);

        if ((o1 > 0 && o2 < 0 || o1 < 0 && o2 > 0)
                && (o3 > 0 && o4 < 0 || o3 < 0 && o4 > 0)) {
            return true;
        }

        return Math.abs(o1) < 1e-10 && onSegment(a, c, b)
                || Math.abs(o2) < 1e-10 && onSegment(a, d, b)
                || Math.abs(o3) < 1e-10 && onSegment(c, a, d)
                || Math.abs(o4) < 1e-10 && onSegment(c, b, d);
    }

    private double orientation(LatLng a, LatLng b, LatLng c) {
        return (b.longitude - a.longitude) * (c.latitude - a.latitude)
                - (b.latitude - a.latitude) * (c.longitude - a.longitude);
    }

    private boolean onSegment(LatLng a, LatLng p, LatLng b) {
        return p.longitude >= Math.min(a.longitude, b.longitude) - 1e-10
                && p.longitude <= Math.max(a.longitude, b.longitude) + 1e-10
                && p.latitude >= Math.min(a.latitude, b.latitude) - 1e-10
                && p.latitude <= Math.max(a.latitude, b.latitude) + 1e-10;
    }

    private LatLng interpolate(LatLng start, LatLng end, double ratio) {
        return new LatLng(
                start.latitude + (end.latitude - start.latitude) * ratio,
                start.longitude + (end.longitude - start.longitude) * ratio
        );
    }

    private double distancePointToPathMeters(LatLng point, List<LatLng> path, boolean closed) {
        double bestDistance = Double.MAX_VALUE;
        int segmentCount = closed ? path.size() : path.size() - 1;
        for (int i = 0; i < segmentCount; i++) {
            LatLng segStart = path.get(i);
            LatLng segEnd = path.get((i + 1) % path.size());
            bestDistance = Math.min(
                    bestDistance,
                    distancePointToSegmentMeters(point, segStart, segEnd)
            );
        }
        return bestDistance;
    }

    private double distancePointToSegmentMeters(LatLng point, LatLng segStart, LatLng segEnd) {
        double cosLat = Math.cos(Math.toRadians(point.latitude));
        double ax = (segStart.longitude - point.longitude) * 111_111d * cosLat;
        double ay = (segStart.latitude - point.latitude) * 111_111d;
        double bx = (segEnd.longitude - point.longitude) * 111_111d * cosLat;
        double by = (segEnd.latitude - point.latitude) * 111_111d;
        double abx = bx - ax;
        double aby = by - ay;
        double abSquared = abx * abx + aby * aby;
        if (abSquared <= 1e-9) {
            return Math.hypot(ax, ay);
        }

        double projection = -(ax * abx + ay * aby) / abSquared;
        projection = Math.max(0d, Math.min(1d, projection));
        double closestX = ax + projection * abx;
        double closestY = ay + projection * aby;
        return Math.hypot(closestX, closestY);
    }

    @Nullable
    private Integer resolveFloorIndexForLogicalFloor(int logicalFloor) {
        if (currentFloorShapes == null) {
            return null;
        }
        for (int i = 0; i < currentFloorShapes.size(); i++) {
            Integer parsed = parseLogicalFloorLabel(
                    currentFloorShapes.get(i).getDisplayName(),
                    currentFloorShapes.get(i).getKey()
            );
            if (parsed != null && parsed == logicalFloor) {
                return i;
            }
        }
        return null;
    }

    @Nullable
    private Integer parseLogicalFloor(int floorIndex) {
        if (currentFloorShapes == null || floorIndex < 0 || floorIndex >= currentFloorShapes.size()) {
            return null;
        }
        FloorplanApiClient.FloorShapes floor = currentFloorShapes.get(floorIndex);
        return parseLogicalFloorLabel(floor.getDisplayName(), floor.getKey());
    }

    @Nullable
    private Integer parseLogicalFloorLabel(String displayName, String fallbackKey) {
        Integer parsed = parseSingleFloorLabel(displayName);
        if (parsed != null) {
            return parsed;
        }
        return parseSingleFloorLabel(fallbackKey);
    }

    @Nullable
    private Integer parseSingleFloorLabel(String rawLabel) {
        if (rawLabel == null) {
            return null;
        }

        String normalized = rawLabel.trim().toUpperCase(Locale.UK);
        if (normalized.isEmpty()) {
            return null;
        }

        normalized = normalized
                .replace("FLOOR", "")
                .replace("LEVEL", "")
                .replace("STOREY", "")
                .replace("STORY", "")
                .replace("_", "")
                .replace("-", "")
                .replace(" ", "");

        if ("G".equals(normalized) || "GF".equals(normalized) || "GROUND".equals(normalized)) {
            return 0;
        }
        if ("LG".equals(normalized) || "LOWGROUND".equals(normalized)
                || "LOWERGROUND".equals(normalized)) {
            return -1;
        }
        if ("UG".equals(normalized) || "UPGROUND".equals(normalized)
                || "UPPERGROUND".equals(normalized)) {
            return 1;
        }

        if (normalized.startsWith("B") && normalized.length() > 1) {
            try {
                return -Integer.parseInt(normalized.substring(1));
            } catch (NumberFormatException ignored) {
                return null;
            }
        }

        if (normalized.startsWith("L") && normalized.length() > 1) {
            try {
                return Integer.parseInt(normalized.substring(1));
            } catch (NumberFormatException ignored) {
                return null;
            }
        }

        try {
            return Integer.parseInt(normalized);
        } catch (NumberFormatException ignored) {
            return null;
        }
    }

    private void invalidateRouteCache() {
        cachedRouteFloor = Integer.MIN_VALUE;
        cachedRouteNodes = new ArrayList<>();
    }
}
