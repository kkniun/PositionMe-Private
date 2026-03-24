package com.openpositioning.PositionMe.utils;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.location.Location;
import com.openpositioning.PositionMe.BuildConfig;
import android.os.Handler;
import android.os.Looper;
import android.text.TextUtils;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.model.BitmapDescriptor;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.GroundOverlay;
import com.google.android.gms.maps.model.GroundOverlayOptions;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.LatLngBounds;
import com.google.android.gms.maps.model.Polyline;
import com.google.android.gms.maps.model.PolylineOptions;
import com.google.android.gms.maps.model.Polygon;
import com.google.android.gms.maps.model.PolygonOptions;
import com.openpositioning.PositionMe.sensors.Wifi;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import okhttp3.Call;
import okhttp3.Callback;
import okhttp3.MediaType;
import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.RequestBody;
import okhttp3.Response;
import okhttp3.ResponseBody;

public class IndoorMapManager {
    private final Map<String, List<String>> shapeFloorKeysCache = new HashMap<>();

    private static final String TAG = "IndoorMapManager";
    private static final String FLOORPLAN_REQUEST_URL =
            "https://openpositioning.org/api/live/floorplan/request/"
                    + BuildConfig.OPENPOSITIONING_API_KEY
                    + "?key=" + BuildConfig.OPENPOSITIONING_MASTER_KEY;
    private static final MediaType JSON = MediaType.parse("application/json; charset=utf-8");
    private static final long REQUEST_INTERVAL_MS = 8_000L;
    private static final float REQUEST_DISTANCE_M = 8f;
    private static final float DEFAULT_FLOOR_HEIGHT_M = 3.6f;
    private static final double WALL_LINE_HALF_WIDTH_M = 0.2;
    private static final double TRANSITION_LINE_HALF_WIDTH_M = 1.0;
    private static final int LOG_PREVIEW_LIMIT = 220;

    private static final int VENUE_STROKE = Color.argb(220, 0, 190, 255);
    private static final int VENUE_FILL = Color.argb(45, 0, 190, 255);
    private static final int SELECTED_STROKE = Color.argb(255, 255, 193, 7);
    private static final int SELECTED_FILL = Color.argb(90, 255, 193, 7);
    private static final int FLOOR_SHAPE_STROKE = Color.argb(220, 255, 255, 255);
    private static final int FLOOR_SHAPE_FILL = Color.argb(40, 255, 255, 255);
    private static final float FLOOR_SHAPE_STROKE_WIDTH = 3f;

    private final GoogleMap gMap;
    private final OkHttpClient httpClient;
    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private final Map<String, VenueModel> venuesById = new LinkedHashMap<>();
    private final Map<String, Polygon> polygonsByVenueId = new HashMap<>();
    private final Map<String, BitmapDescriptor> floorImageCache = new HashMap<>();
    private final List<Polygon> floorShapePolygons = new ArrayList<>();
    private final List<Polyline> floorShapePolylines = new ArrayList<>();

    private GroundOverlay groundOverlay;
    private LatLng currentLocation;
    private LatLng lastRequestLocation;
    private long lastRequestTs;
    private boolean requestInFlight;
    private boolean isIndoorMapSet;
    private boolean hasVectorMapShapes;
    private int currentFloor;
    private float floorHeight = DEFAULT_FLOOR_HEIGHT_M;
    private int floorImageToken;
    private String selectedVenueId;
    private boolean autoSelectFirstVenue;
    private VenueSelectionListener venueSelectionListener;

    public interface VenueSelectionListener {
        void onVenueSelected(@Nullable String venueId, @Nullable String venueName);
    }

    public IndoorMapManager(@NonNull GoogleMap map) {
        this.gMap = map;
        this.httpClient = new OkHttpClient();
    }

    public IndoorMapManager(@NonNull Context context, @NonNull GoogleMap map) {
        this(map);
    }

    public void setVenueSelectionListener(@Nullable VenueSelectionListener listener) {
        this.venueSelectionListener = listener;
    }

    public void setAutoSelectFirstVenue(boolean autoSelectFirstVenue) {
        this.autoSelectFirstVenue = autoSelectFirstVenue;
    }

    public void setCurrentLocation(@Nullable LatLng location) {
        this.currentLocation = location;
    }

    public void refreshNearbyVenues(@Nullable LatLng location, @Nullable List<Wifi> observedAps) {
        if (location == null || requestInFlight) {
            return;
        }
        long now = System.currentTimeMillis();
        if (lastRequestLocation != null) {
            if (now - lastRequestTs < REQUEST_INTERVAL_MS
                    && distanceMeters(lastRequestLocation, location) < REQUEST_DISTANCE_M) {
                return;
            }
        }

        JSONObject payload;
        try {
            payload = buildRequestPayload(location, observedAps);
        } catch (JSONException e) {
            Log.e(TAG, "Cannot build floorplan payload", e);
            return;
        }

        requestInFlight = true;
        lastRequestTs = now;
        lastRequestLocation = location;

        String payloadString = payload.toString();
        Request request = new Request.Builder()
                .url(FLOORPLAN_REQUEST_URL)
                .post(RequestBody.create(payloadString, JSON))
                .addHeader("accept", "application/json")
                .build();

        if (BuildConfig.DEBUG) {
            Log.d(TAG, "floorplan request method=" + request.method()
                    + " url=" + request.url()
                    + " bodyPreview=" + previewForLog(payloadString));
        }

        httpClient.newCall(request).enqueue(new Callback() {
            @Override
            public void onFailure(@NonNull Call call, @NonNull IOException e) {
                requestInFlight = false;
                Log.e(TAG, "Nearby floorplan request failed", e);
            }

            @Override
            public void onResponse(@NonNull Call call, @NonNull Response response) throws IOException {
                try (ResponseBody body = response.body()) {
                    String rawBody = body == null ? "" : body.string();
                    List<VenueModel> venues = Collections.emptyList();
                    if (!response.isSuccessful() || body == null) {
                        Log.w(TAG, "Nearby floorplan request not successful: " + response.code());
                    } else {
                        venues = parseVenueResponse(rawBody);
                        List<VenueModel> finalVenues = venues;
                        mainHandler.post(() -> applyNearbyVenues(finalVenues));
                    }
                    if (BuildConfig.DEBUG) {
                        Log.d(TAG, "floorplan response http_status=" + response.code()
                                + " body_preview=" + previewForLog(rawBody)
                                + " venuesCount=" + venues.size());
                    }
                } catch (Exception ex) {
                    Log.e(TAG, "Failed to parse nearby floorplan response", ex);
                } finally {
                    requestInFlight = false;
                }
            }
        });
    }

    public boolean onVenuePolygonClicked(@Nullable Polygon polygon) {
        if (polygon == null || polygon.getTag() == null) {
            return false;
        }
        return selectVenue(String.valueOf(polygon.getTag()));
    }

    public boolean selectVenue(@Nullable String venueId) {
        if (TextUtils.isEmpty(venueId)) {
            return false;
        }
        VenueModel venue = venuesById.get(venueId);
        if (venue == null) {
            return false;
        }

        selectedVenueId = venue.id;
        if (BuildConfig.DEBUG) {
            Log.d(TAG, "FLOOR_UI: venue selected id=" + venue.id);
        }
        floorHeight = venue.floorHeight;
        if (!venue.floors.isEmpty()) {
            currentFloor = snapToNearestFloor(currentFloor, getAvailableImageFloors(venue));
        }
        loadFloorplanForVenue(venue);
        updatePolygonStyle();

        if (venueSelectionListener != null) {
            venueSelectionListener.onVenueSelected(venue.id, venue.name);
        }
        return true;
    }

    public boolean getIsIndoorMapSet() {
        return isIndoorMapSet;
    }

    public boolean hasVectorMapShapes() {
        return hasVectorMapShapes;
    }

    public float getFloorHeight() {
        return floorHeight;
    }

    public boolean hasActiveMapMatchingConstraints() {
        return !TextUtils.isEmpty(selectedVenueId)
                && hasVectorMapShapes
                && (MapConstraintRepository.hasAnyConstraints()
                || MapConstraintRepository.hasVenueOutline());
    }

    public int getFloorCount() {
        VenueModel selected = getSelectedVenue();
        if (selected == null) return 0;
        if (!selected.floors.isEmpty()) return selected.floors.size();
        if (TextUtils.isEmpty(selected.mapShapesPayload)) return 0;
        return getShapeFloorKeys(selected).size();
    }


    public int getCurrentFloor() {
        return currentFloor;
    }

    @Nullable
    public String getSelectedVenueId() {
        return selectedVenueId;
    }

    @Nullable
    public String getSelectedVenueName() {
        VenueModel selected = getSelectedVenue();
        return selected == null ? null : selected.name;
    }

    public void setCurrentFloor(int newFloor, boolean autoFloor) {
        VenueModel selected = getSelectedVenue();
        if (selected == null) {
            return;
        }

        // ---- map_shapes 模式：floors 为空，但 mapShapesPayload 有内容 ----
        if (selected.floors.isEmpty()) {
            List<Integer> availableFloors = getAvailableShapeFloors(selected);
            if (availableFloors.isEmpty()) return;

            int bounded = snapToNearestFloor(newFloor, availableFloors);
            Log.d(TAG, "UI wants newFloor=" + newFloor
                    + " map_shapes floors=" + availableFloors
                    + " currentFloor(before)=" + currentFloor
                    + " bounded(absFloor)=" + bounded);

            if (bounded == currentFloor && isIndoorMapSet) {
                Log.d(TAG, "NOOP map_shapes: same floor");
                return;
            }

            currentFloor = bounded;

            // 关键：触发重新绘制当前 floorKey 的 shapes（不要 renderCurrentFloor）
            loadFloorplanForVenue(selected);
            updatePolygonStyle();
            return;
        }

        // ---- image overlay 模式：floors 不为空，原逻辑保留 ----
        Log.d(TAG, "UI wants newFloor=" + newFloor
                + " floors.size=" + selected.floors.size()
                + " currentFloor(before)=" + currentFloor
                + " floorIndexList=" + dumpFloorIndexList(selected));

        int bounded = snapToNearestFloor(newFloor, getAvailableImageFloors(selected));
        Log.d(TAG, "bounded(absFloor)=" + bounded);

        if (bounded == currentFloor && isIndoorMapSet) {
            Log.d(TAG, "NOOP: bounded == currentFloor and map set, return");
            return;
        }

        currentFloor = bounded;
        renderCurrentFloor();
    }


    private String dumpFloorIndexList(VenueModel venue) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < venue.floors.size(); i++) {
            FloorModel f = venue.floors.get(i);
            sb.append("[")
                    .append(i)
                    .append("->floorIndex=").append(f.floorIndex)
                    .append(", floorName=").append(f.floorName)
                    .append("] ");
        }
        return sb.toString();
    }



    public void increaseFloor() {
        VenueModel selected = getSelectedVenue();
        if (selected == null) {
            return;
        }
        Integer nextFloor = findAdjacentFloor(selected, currentFloor, true);
        if (nextFloor != null) {
            setCurrentFloor(nextFloor, false);
        }
    }

    public void decreaseFloor() {
        VenueModel selected = getSelectedVenue();
        if (selected == null) {
            return;
        }
        Integer nextFloor = findAdjacentFloor(selected, currentFloor, false);
        if (nextFloor != null) {
            setCurrentFloor(nextFloor, false);
        }
    }

    private void renderCurrentFloor() {
        VenueModel selected = getSelectedVenue();
        if (selected == null || selected.floors.isEmpty()) {
            removeGroundOverlay();
            return;
        }
        currentFloor = snapToNearestFloor(currentFloor, getAvailableImageFloors(selected));
        if (hasVectorMapShapes) {
            MapConstraintRepository.setActiveFloor(currentFloor);
        }
        FloorModel floor = findFloorModelForAbsoluteFloor(selected, currentFloor);
        if (floor == null) {
            removeGroundOverlay();
            return;
        }
        BitmapDescriptor cached = floorImageCache.get(floor.imageUrl);
        if (cached != null) {
            setGroundOverlay(cached, floor.bounds);
            return;
        }

        int requestToken = ++floorImageToken;
        if (BuildConfig.DEBUG) {
            Log.d(TAG, "FLOOR_NET: request floors venueId=" + selected.id + " url=" + floor.imageUrl);
        }
        Request req = new Request.Builder().url(floor.imageUrl).get().build();
        httpClient.newCall(req).enqueue(new Callback() {
            @Override
            public void onFailure(@NonNull Call call, @NonNull IOException e) {
                Log.e(TAG, "Failed to download floor image: " + floor.imageUrl, e);
            }

            @Override
            public void onResponse(@NonNull Call call, @NonNull Response response) throws IOException {
                try (ResponseBody body = response.body()) {
                    if (!response.isSuccessful() || body == null) {
                        if (BuildConfig.DEBUG) {
                            Log.d(TAG, "FLOOR_NET: response http_status=" + response.code()
                                    + " body_preview=");
                        }
                        return;
                    }
                    byte[] bytes = body.bytes();
                    if (BuildConfig.DEBUG) {
                        Log.d(TAG, "FLOOR_NET: response http_status=" + response.code()
                                + " body_preview=bytes=" + bytes.length);
                    }
                    Bitmap bitmap = BitmapFactory.decodeByteArray(bytes, 0, bytes.length);
                    if (bitmap == null) {
                        return;
                    }
                    BitmapDescriptor descriptor = BitmapDescriptorFactory.fromBitmap(bitmap);
                    floorImageCache.put(floor.imageUrl, descriptor);
                    mainHandler.post(() -> {
                        if (requestToken != floorImageToken) {
                            return;
                        }
                        VenueModel latest = getSelectedVenue();
                        if (latest == null || latest.floors.isEmpty()) {
                            return;
                        }
                        FloorModel current = findFloorModelForAbsoluteFloor(latest, currentFloor);
                        if (current == null) {
                            return;
                        }
                        if (!floor.imageUrl.equals(current.imageUrl)) {
                            return;
                        }
                        setGroundOverlay(descriptor, current.bounds);
                    });
                }
            }
        });
    }

    private void loadFloorplanForVenue(@NonNull VenueModel venue) {
        clearFloorShapeOverlays();
        boolean hasImageFloors = !venue.floors.isEmpty();
        if (hasImageFloors) {
            currentFloor = snapToNearestFloor(currentFloor, getAvailableImageFloors(venue));
        } else {
            removeGroundOverlay();
        }
        if (TextUtils.isEmpty(venue.mapShapesPayload)) {
            hasVectorMapShapes = false;
            MapConstraintRepository.clear();
            if (hasImageFloors) {
                renderCurrentFloor();
            } else {
                isIndoorMapSet = false;
            }
            return;
        }
        String payload = venue.mapShapesPayload;
        if (BuildConfig.DEBUG) {
            Log.d(TAG, "FLOOR_NET: request venueId=" + venue.id + " url=inline://map_shapes");
            Log.d(TAG, "FLOOR_NET: response http_status=200 body_preview=" + previewForLog(payload));
        }
        List<String> keys = getShapeFloorKeys(venue);
        if (keys.isEmpty()) {
            hasVectorMapShapes = false;
            MapConstraintRepository.clear();
            if (hasImageFloors) {
                renderCurrentFloor();
            } else {
                isIndoorMapSet = false;
            }
            return;
        }
        hasVectorMapShapes = true;

// clamp currentFloor 到 key 范围
        if (!hasImageFloors) {
            currentFloor = snapToNearestFloor(currentFloor, getAvailableShapeFloors(keys));
        }
        String key = findShapeFloorKeyForAbsoluteFloor(keys, currentFloor);
        if (key == null) {
            if (!hasImageFloors) {
                hasVectorMapShapes = false;
                MapConstraintRepository.clear();
                isIndoorMapSet = false;
                return;
            }
        }

        MapConstraintRepository.replaceVenueConstraints(venue.id, venue.outline);
        List<List<LatLng>> currentFloorWallPolygons = Collections.emptyList();
        List<List<LatLng>> currentFloorTransitionPolygons = Collections.emptyList();
        for (String floorKey : keys) {
            int floorIndex = parseFloorNameToAbsoluteFloor(floorKey);
            List<List<LatLng>> wallPolygons = extractWallPolygonsForKey(payload, floorKey);
            TransitionPolygonSet transitionPolygons = extractTransitionPolygonsForKey(payload, floorKey);
            List<List<LatLng>> allTransitionPolygons = transitionPolygons.getAllPolygons();
            MapConstraintRepository.setConstraintsForFloor(
                    floorIndex,
                    floorKey,
                    wallPolygons,
                    allTransitionPolygons
            );
            MapConstraintRepository.setStairsForFloor(floorIndex, transitionPolygons.stairsPolygons);
            MapConstraintRepository.setLiftsForFloor(floorIndex, transitionPolygons.liftPolygons);
            if (floorIndex == currentFloor) {
                currentFloorWallPolygons = wallPolygons;
                currentFloorTransitionPolygons = allTransitionPolygons;
            }
        }
        MapConstraintRepository.setActiveFloor(currentFloor);
        if (com.openpositioning.PositionMe.sensors.SensorFusion.DEBUG_FUSION_TRACE) {
            Log.d(TAG, "MAP_DBG ts=" + System.currentTimeMillis()
                    + " source=MAP_CONSTRAINT"
                    + " venueId=" + venue.id
                    + " floorKey=" + key
                    + " currentFloor=" + currentFloor
                    + " wallCount=" + currentFloorWallPolygons.size()
                    + " transitionCount=" + currentFloorTransitionPolygons.size()
                    + " currentLocation=" + currentLocation);
        }

        int[] counts = new int[]{0, 0};
        if (key != null) {
            drawMapShapesForKey(payload, key, counts);
        }

        isIndoorMapSet = counts[0] > 0 || counts[1] > 0;

        if (hasImageFloors) {
            renderCurrentFloor();
            if (!isIndoorMapSet) {
                isIndoorMapSet = hasActiveMapMatchingConstraints();
            }
        }

        if (BuildConfig.DEBUG) {
            Log.d(TAG, "FLOOR_DRAW: floorKey=" + key
                    + " drawnPolylines=" + counts[0] + " drawnPolygons=" + counts[1]);
        }

    }

    @NonNull
    private List<List<LatLng>> extractWallPolygonsForKey(@NonNull String payload, @NonNull String floorKey) {
        List<List<LatLng>> wallPolygons = new ArrayList<>();
        try {
            JSONObject root = new JSONObject(payload.trim());
            Object child = root.opt(floorKey);
            if (child instanceof JSONObject) {
                collectWallPolygonsFromObject((JSONObject) child, wallPolygons);
            } else if (child instanceof String) {
                String raw = ((String) child).trim();
                if (raw.startsWith("{")) {
                    collectWallPolygonsFromObject(new JSONObject(raw), wallPolygons);
                }
            }
        } catch (JSONException e) {
            Log.e(TAG, "Failed to extract wall polygons for key=" + floorKey, e);
        }
        return wallPolygons;
    }

    private void collectWallPolygonsFromObject(@NonNull JSONObject obj, @NonNull List<List<LatLng>> wallPolygons) {
        if ("FeatureCollection".equalsIgnoreCase(obj.optString("type")) || obj.has("features")) {
            JSONArray features = obj.optJSONArray("features");
            if (features == null) {
                return;
            }
            for (int i = 0; i < features.length(); i++) {
                JSONObject feature = features.optJSONObject(i);
                if (feature == null) {
                    continue;
                }
                JSONObject properties = feature.optJSONObject("properties");
                if (!isWallFeature(properties)) {
                    continue;
                }
                JSONObject geometry = feature.optJSONObject("geometry");
                collectWallPolygonsFromGeometry(geometry, wallPolygons);
            }
            return;
        }

        JSONArray names = obj.names();
        if (names == null) {
            return;
        }
        for (int i = 0; i < names.length(); i++) {
            String key = names.optString(i, "");
            Object child = obj.opt(key);
            if (child instanceof JSONObject) {
                collectWallPolygonsFromObject((JSONObject) child, wallPolygons);
            } else if (child instanceof String) {
                String raw = ((String) child).trim();
                try {
                    if (raw.startsWith("{")) {
                        collectWallPolygonsFromObject(new JSONObject(raw), wallPolygons);
                    }
                } catch (JSONException ignored) {
                }
            }
        }
    }

    /**
     * Walls only: explicit tags, avoids mis-classifying stairs/lifts or arbitrary strings containing "wall".
     */
    private boolean isWallFeature(@Nullable JSONObject properties) {
        String tag = primarySemanticTag(properties);
        if (tag.isEmpty()) {
            return false;
        }
        if (isTransitionSemanticTag(tag)) {
            return false;
        }
        return "wall".equals(tag) || "walls".equals(tag);
    }

    private boolean isStairsFeature(@Nullable JSONObject properties) {
        return isStairsSemanticTag(primarySemanticTag(properties));
    }

    private boolean isLiftFeature(@Nullable JSONObject properties) {
        return isLiftSemanticTag(primarySemanticTag(properties));
    }

    @NonNull
    private static String primarySemanticTag(@Nullable JSONObject properties) {
        if (properties == null) {
            return "";
        }
        String[] candidateKeys = new String[]{
                "indoor_type", "feature_type", "type", "class", "kind", "category", "subtype"
        };
        for (String key : candidateKeys) {
            String value = properties.optString(key, "").trim().toLowerCase(Locale.US);
            if (!TextUtils.isEmpty(value) && !"null".equals(value)) {
                return value;
            }
        }
        return "";
    }

    private static boolean isStairsSemanticTag(@NonNull String tag) {
        return "stairs".equals(tag) || "stair".equals(tag) || "escalator".equals(tag);
    }

    private static boolean isLiftSemanticTag(@NonNull String tag) {
        return "lift".equals(tag) || "elevator".equals(tag);
    }

    private static boolean isTransitionSemanticTag(@NonNull String tag) {
        return isStairsSemanticTag(tag) || isLiftSemanticTag(tag);
    }

    private void collectWallPolygonsFromGeometry(
            @Nullable JSONObject geometry,
            @NonNull List<List<LatLng>> wallPolygons
    ) {
        collectConstraintPolygonsFromGeometry(geometry, wallPolygons, WALL_LINE_HALF_WIDTH_M);
    }

    private void collectConstraintPolygonsFromGeometry(
            @Nullable JSONObject geometry,
            @NonNull List<List<LatLng>> polygons,
            double lineHalfWidthMeters
    ) {
        if (geometry == null) {
            return;
        }
        String type = geometry.optString("type", "");
        if ("GeometryCollection".equalsIgnoreCase(type)) {
            JSONArray geometries = geometry.optJSONArray("geometries");
            if (geometries == null) {
                return;
            }
            for (int i = 0; i < geometries.length(); i++) {
                JSONObject child = geometries.optJSONObject(i);
                if (child != null) {
                    collectConstraintPolygonsFromGeometry(child, polygons, lineHalfWidthMeters);
                }
            }
            return;
        }

        JSONArray coordinates = geometry.optJSONArray("coordinates");
        if (coordinates == null) {
            return;
        }

        if ("Polygon".equalsIgnoreCase(type)) {
            JSONArray outerRing = coordinates.optJSONArray(0);
            addWallPolygonIfValid(outerRing, polygons);
            return;
        }
        if ("MultiPolygon".equalsIgnoreCase(type)) {
            for (int i = 0; i < coordinates.length(); i++) {
                JSONArray polygon = coordinates.optJSONArray(i);
                if (polygon == null) {
                    continue;
                }
                JSONArray outerRing = polygon.optJSONArray(0);
                addWallPolygonIfValid(outerRing, polygons);
            }
            return;
        }
        if ("LineString".equalsIgnoreCase(type)) {
            addBufferedLineStringPolygons(parsePointArray(coordinates), lineHalfWidthMeters, polygons);
            return;
        }
        if ("MultiLineString".equalsIgnoreCase(type)) {
            for (int i = 0; i < coordinates.length(); i++) {
                JSONArray line = coordinates.optJSONArray(i);
                if (line != null) {
                    addBufferedLineStringPolygons(parsePointArray(line), lineHalfWidthMeters, polygons);
                }
            }
        }
    }

    private void addWallPolygonIfValid(@Nullable JSONArray ring, @NonNull List<List<LatLng>> wallPolygons) {
        if (ring == null) {
            return;
        }
        List<LatLng> points = parsePointArray(ring);
        if (points.size() >= 3) {
            wallPolygons.add(points);
        }
    }

    private void addBufferedLineStringPolygons(
            @NonNull List<LatLng> linePoints,
            double halfWidthMeters,
            @NonNull List<List<LatLng>> polygons
    ) {
        if (linePoints.size() < 2 || halfWidthMeters <= 0.0) {
            return;
        }
        for (int i = 1; i < linePoints.size(); i++) {
            List<LatLng> bufferedSegment = ConstraintGeometryUtils.buildBufferedSegmentPolygon(
                    linePoints.get(i - 1),
                    linePoints.get(i),
                    halfWidthMeters
            );
            if (bufferedSegment.size() >= 3) {
                polygons.add(bufferedSegment);
            }
        }
    }

    @NonNull
    private TransitionPolygonSet extractTransitionPolygonsForKey(@NonNull String payload, @NonNull String floorKey) {
        TransitionPolygonSet transitionPolygons = new TransitionPolygonSet();
        try {
            JSONObject root = new JSONObject(payload.trim());
            Object child = root.opt(floorKey);
            if (child instanceof JSONObject) {
                collectTransitionPolygonsFromObject((JSONObject) child, transitionPolygons);
            } else if (child instanceof String) {
                String raw = ((String) child).trim();
                if (raw.startsWith("{")) {
                    collectTransitionPolygonsFromObject(new JSONObject(raw), transitionPolygons);
                }
            }
        } catch (JSONException e) {
            Log.e(TAG, "Failed to extract transition polygons for key=" + floorKey, e);
        }
        return transitionPolygons;
    }

    private void collectTransitionPolygonsFromObject(
            @NonNull JSONObject obj,
            @NonNull TransitionPolygonSet transitionPolygons
    ) {
        if ("FeatureCollection".equalsIgnoreCase(obj.optString("type")) || obj.has("features")) {
            JSONArray features = obj.optJSONArray("features");
            if (features == null) {
                return;
            }
            for (int i = 0; i < features.length(); i++) {
                JSONObject feature = features.optJSONObject(i);
                if (feature == null) {
                    continue;
                }
                JSONObject properties = feature.optJSONObject("properties");
                JSONObject geometry = feature.optJSONObject("geometry");
                if (isLiftFeature(properties)) {
                    collectConstraintPolygonsFromGeometry(
                            geometry,
                            transitionPolygons.liftPolygons,
                            TRANSITION_LINE_HALF_WIDTH_M
                    );
                    continue;
                }
                if (isStairsFeature(properties)) {
                    collectConstraintPolygonsFromGeometry(
                            geometry,
                            transitionPolygons.stairsPolygons,
                            TRANSITION_LINE_HALF_WIDTH_M
                    );
                }
            }
            return;
        }

        JSONArray names = obj.names();
        if (names == null) {
            return;
        }
        for (int i = 0; i < names.length(); i++) {
            String key = names.optString(i, "");
            Object child = obj.opt(key);
            if (child instanceof JSONObject) {
                collectTransitionPolygonsFromObject((JSONObject) child, transitionPolygons);
            } else if (child instanceof String) {
                String raw = ((String) child).trim();
                try {
                    if (raw.startsWith("{")) {
                        collectTransitionPolygonsFromObject(new JSONObject(raw), transitionPolygons);
                    }
                } catch (JSONException ignored) {
                }
            }
        }
    }

    private static final class TransitionPolygonSet {
        @NonNull
        final List<List<LatLng>> stairsPolygons = new ArrayList<>();
        @NonNull
        final List<List<LatLng>> liftPolygons = new ArrayList<>();

        @NonNull
        List<List<LatLng>> getAllPolygons() {
            List<List<LatLng>> combined = new ArrayList<>(stairsPolygons.size() + liftPolygons.size());
            combined.addAll(stairsPolygons);
            combined.addAll(liftPolygons);
            return combined;
        }
    }

    private void clearFloorShapeOverlays() {
        for (Polyline polyline : floorShapePolylines) {
            polyline.remove();
        }
        floorShapePolylines.clear();
        for (Polygon polygon : floorShapePolygons) {
            polygon.remove();
        }
        floorShapePolygons.clear();
    }

    @NonNull
    private List<String> getShapeFloorKeys(@NonNull VenueModel venue) {
        List<String> cached = shapeFloorKeysCache.get(venue.id);
        if (cached != null) return cached;

        List<String> keys = new ArrayList<>();
        try {
            String trimmed = venue.mapShapesPayload == null ? "" : venue.mapShapesPayload.trim();
            if (trimmed.startsWith("{")) {
                JSONObject root = new JSONObject(trimmed);
                JSONArray names = root.names();
                if (names != null) {
                    for (int i = 0; i < names.length(); i++) {
                        String k = names.optString(i, "");
                        if (!TextUtils.isEmpty(k)) keys.add(k);
                    }
                }
            }
        } catch (JSONException ignored) {}

        // 可选：排序，让 UI 顺序稳定（B1,G,1,2... 你也可以自定义排序规则）
        Collections.sort(keys, (left, right) -> {
            int leftFloor = parseFloorNameToAbsoluteFloor(left);
            int rightFloor = parseFloorNameToAbsoluteFloor(right);
            if (leftFloor != rightFloor) {
                return Integer.compare(leftFloor, rightFloor);
            }
            return left.compareToIgnoreCase(right);
        });

        shapeFloorKeysCache.put(venue.id, keys);
        return keys;
    }

    private static int parseFloorNameToAbsoluteFloor(@Nullable String floorName) {
        if (TextUtils.isEmpty(floorName)) {
            Log.e(TAG, "Failed to parse floor key: empty, fallback=0");
            return 0;
        }

        String normalized = floorName.trim();
        if (normalized.isEmpty()) {
            Log.e(TAG, "Failed to parse floor key: blank, fallback=0");
            return 0;
        }

        String upper = normalized.toUpperCase(Locale.US).trim();
        try {
            if ("GF".equals(upper) || "G".equals(upper) || "GROUND".equals(upper)) {
                return 0;
            }
            if (upper.startsWith("B")) {
                String basementDigits = upper.substring(1).replaceAll("[^\\d]", "");
                if (!basementDigits.isEmpty()) {
                    return -Integer.parseInt(basementDigits);
                }
            }
            if (upper.startsWith("F") || upper.startsWith("L")) {
                String floorDigits = upper.substring(1).replaceAll("[^\\d]", "");
                if (!floorDigits.isEmpty()) {
                    return Integer.parseInt(floorDigits);
                }
            }
            if (upper.matches("-?\\d+")) {
                return Integer.parseInt(upper);
            }
        } catch (RuntimeException e) {
            Log.e(TAG, "Failed to parse floor key=\"" + floorName + "\", fallback=0", e);
            return 0;
        }

        Log.e(TAG, "Failed to parse floor key=\"" + floorName + "\", fallback=0");
        return 0;
    }

    @NonNull
    private List<Integer> getAvailableShapeFloors(@NonNull VenueModel venue) {
        return getAvailableShapeFloors(getShapeFloorKeys(venue));
    }

    @NonNull
    private List<Integer> getAvailableShapeFloors(@NonNull List<String> floorKeys) {
        List<Integer> floors = new ArrayList<>(floorKeys.size());
        for (String floorKey : floorKeys) {
            int floor = parseFloorNameToAbsoluteFloor(floorKey);
            if (!floors.contains(floor)) {
                floors.add(floor);
            }
        }
        return floors;
    }

    @NonNull
    private List<Integer> getAvailableImageFloors(@NonNull VenueModel venue) {
        List<Integer> floors = new ArrayList<>(venue.floors.size());
        for (FloorModel floor : venue.floors) {
            if (!floors.contains(floor.floorIndex)) {
                floors.add(floor.floorIndex);
            }
        }
        return floors;
    }

    private int snapToNearestFloor(int requestedFloor, @NonNull List<Integer> availableFloors) {
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

    @Nullable
    private Integer findAdjacentFloor(
            @NonNull VenueModel venue,
            int fromFloor,
            boolean upward
    ) {
        List<Integer> availableFloors = venue.floors.isEmpty()
                ? getAvailableShapeFloors(venue)
                : getAvailableImageFloors(venue);
        Integer candidate = null;
        for (int floor : availableFloors) {
            if (upward) {
                if (floor > fromFloor && (candidate == null || floor < candidate)) {
                    candidate = floor;
                }
            } else if (floor < fromFloor && (candidate == null || floor > candidate)) {
                candidate = floor;
            }
        }
        return candidate;
    }

    @Nullable
    private String findShapeFloorKeyForAbsoluteFloor(
            @NonNull List<String> floorKeys,
            int absoluteFloor
    ) {
        for (String floorKey : floorKeys) {
            if (parseFloorNameToAbsoluteFloor(floorKey) == absoluteFloor) {
                return floorKey;
            }
        }
        return null;
    }

    @Nullable
    private FloorModel findFloorModelForAbsoluteFloor(
            @NonNull VenueModel venue,
            int absoluteFloor
    ) {
        for (FloorModel floor : venue.floors) {
            if (floor.floorIndex == absoluteFloor) {
                return floor;
            }
        }
        return null;
    }

    private void drawMapShapesForKey(@NonNull String payload, @NonNull String floorKey, @NonNull int[] counts) {
        try {
            JSONObject root = new JSONObject(payload.trim());
            Object child = root.opt(floorKey);
            if (child instanceof JSONObject) {
                drawMapShapesFromObject((JSONObject) child, counts);
            } else if (child instanceof String) {
                String raw = ((String) child).trim();
                if (raw.startsWith("{")) {
                    drawMapShapesFromObject(new JSONObject(raw), counts);
                }
            }
        } catch (JSONException e) {
            Log.e(TAG, "Failed to parse map_shapes payload for key=" + floorKey, e);
        }
    }

    @NonNull
    private int[] drawMapShapes(@NonNull String payload) {
        int[] counts = new int[]{0, 0}; // [polylines, polygons]
        try {
            String trimmed = payload.trim();
            if (trimmed.startsWith("{")) {
                JSONObject root = new JSONObject(trimmed);
                drawMapShapesFromObject(root, counts);
            } else if (trimmed.startsWith("[")) {
                JSONArray arr = new JSONArray(trimmed);
                for (int i = 0; i < arr.length(); i++) {
                    JSONObject obj = arr.optJSONObject(i);
                    if (obj != null) {
                        drawMapShapesFromObject(obj, counts);
                    }
                }
            }
        } catch (JSONException e) {
            Log.e(TAG, "Failed to parse map_shapes payload", e);
        }
        return counts;
    }

    private void drawMapShapesFromObject(@NonNull JSONObject obj, @NonNull int[] counts) {
        if ("FeatureCollection".equalsIgnoreCase(obj.optString("type")) || obj.has("features")) {
            drawFeatureCollection(obj, counts);
            return;
        }
        JSONArray names = obj.names();
        if (names == null) {
            return;
        }
        for (int i = 0; i < names.length(); i++) {
            String key = names.optString(i, "");
            Object child = obj.opt(key);
            if (child instanceof JSONObject) {
                drawMapShapesFromObject((JSONObject) child, counts);
            } else if (child instanceof String) {
                String raw = ((String) child).trim();
                try {
                    if (raw.startsWith("{")) {
                        drawMapShapesFromObject(new JSONObject(raw), counts);
                    }
                } catch (JSONException ignored) {
                }
            }
        }
    }

    private void drawFeatureCollection(@NonNull JSONObject collection, @NonNull int[] counts) {
        JSONArray features = collection.optJSONArray("features");
        if (features == null) {
            return;
        }
        for (int i = 0; i < features.length(); i++) {
            JSONObject feature = features.optJSONObject(i);
            if (feature == null) {
                continue;
            }
            JSONObject geometry = feature.optJSONObject("geometry");
            if (geometry == null) {
                continue;
            }
            drawGeometry(geometry, counts);
        }
    }

    private void drawGeometry(@NonNull JSONObject geometry, @NonNull int[] counts) {
        String type = geometry.optString("type", "");
        JSONArray coordinates = geometry.optJSONArray("coordinates");
        if (coordinates == null) {
            return;
        }
        if ("LineString".equalsIgnoreCase(type)) {
            drawLineString(coordinates, counts);
            return;
        }
        if ("MultiLineString".equalsIgnoreCase(type)) {
            for (int i = 0; i < coordinates.length(); i++) {
                JSONArray line = coordinates.optJSONArray(i);
                if (line != null) {
                    drawLineString(line, counts);
                }
            }
            return;
        }
        if ("Polygon".equalsIgnoreCase(type)) {
            drawPolygonGeometry(coordinates, counts);
            return;
        }
        if ("MultiPolygon".equalsIgnoreCase(type)) {
            for (int i = 0; i < coordinates.length(); i++) {
                JSONArray polygon = coordinates.optJSONArray(i);
                if (polygon != null) {
                    drawPolygonGeometry(polygon, counts);
                }
            }
        }
    }

    private void drawPolygonGeometry(@NonNull JSONArray polygonCoordinates, @NonNull int[] counts) {
        JSONArray outerRing = polygonCoordinates.optJSONArray(0);
        if (outerRing == null) {
            return;
        }
        List<LatLng> points = parsePointArray(outerRing);
        if (points.size() < 3) {
            return;
        }
        Polygon polygon = gMap.addPolygon(new PolygonOptions()
                .addAll(points)
                .strokeColor(FLOOR_SHAPE_STROKE)
                .fillColor(FLOOR_SHAPE_FILL)
                .strokeWidth(FLOOR_SHAPE_STROKE_WIDTH)
                .clickable(false)
                .zIndex(8f));
        floorShapePolygons.add(polygon);
        counts[1]++;
    }

    private void drawLineString(@NonNull JSONArray lineCoordinates, @NonNull int[] counts) {
        List<LatLng> points = parsePointArray(lineCoordinates);
        if (points.size() < 2) {
            return;
        }
        Polyline polyline = gMap.addPolyline(new PolylineOptions()
                .addAll(points)
                .color(FLOOR_SHAPE_STROKE)
                .width(FLOOR_SHAPE_STROKE_WIDTH)
                .zIndex(8f));
        floorShapePolylines.add(polyline);
        counts[0]++;
    }

    private void setGroundOverlay(@NonNull BitmapDescriptor descriptor, @NonNull LatLngBounds bounds) {
        removeGroundOverlay();
        groundOverlay = gMap.addGroundOverlay(new GroundOverlayOptions()
                .image(descriptor)
                .positionFromBounds(bounds)
                .zIndex(10f));
        if (BuildConfig.DEBUG) {
            Log.d(TAG, "FLOOR_DRAW: addGroundOverlay");
        }
        isIndoorMapSet = groundOverlay != null;
    }

    private void removeGroundOverlay() {
        if (groundOverlay != null) {
            groundOverlay.remove();
            groundOverlay = null;
        }
        isIndoorMapSet = false;
    }

    private void applyNearbyVenues(@NonNull List<VenueModel> venues) {
        venuesById.clear();
        for (VenueModel venue : venues) {
            venuesById.put(venue.id, venue);
        }

        for (Polygon polygon : polygonsByVenueId.values()) {
            polygon.remove();
        }
        polygonsByVenueId.clear();
        clearFloorShapeOverlays();

        for (VenueModel venue : venues) {
            if (venue.outline.size() < 3) {
                continue;
            }
            Polygon polygon = gMap.addPolygon(new PolygonOptions()
                    .addAll(venue.outline)
                    .strokeColor(VENUE_STROKE)
                    .fillColor(VENUE_FILL)
                    .strokeWidth(4f)
                    .clickable(true)
                    .zIndex(5f));
            if (BuildConfig.DEBUG) {
                Log.d(TAG, "addPolygon called venueId=" + venue.id + " points=" + venue.outline.size());
            }
            polygon.setTag(venue.id);
            polygonsByVenueId.put(venue.id, polygon);
        }
        updatePolygonStyle();

        if (!TextUtils.isEmpty(selectedVenueId) && venuesById.containsKey(selectedVenueId)) {
            selectVenue(selectedVenueId);
        } else if (autoSelectFirstVenue && !venues.isEmpty()) {
            VenueModel preferredVenue = choosePreferredVenue(venues);
            selectVenue(preferredVenue.id);
        } else {
            MapConstraintRepository.clear();
            selectedVenueId = null;
            currentFloor = 0;
            floorHeight = DEFAULT_FLOOR_HEIGHT_M;
            hasVectorMapShapes = false;
            removeGroundOverlay();
            if (venueSelectionListener != null) {
                venueSelectionListener.onVenueSelected(null, null);
            }
        }
    }

    private void updatePolygonStyle() {
        for (Map.Entry<String, Polygon> entry : polygonsByVenueId.entrySet()) {
            boolean selected = entry.getKey().equals(selectedVenueId);
            Polygon polygon = entry.getValue();
            polygon.setStrokeColor(selected ? SELECTED_STROKE : VENUE_STROKE);
            polygon.setFillColor(selected ? SELECTED_FILL : VENUE_FILL);
            polygon.setStrokeWidth(selected ? 7f : 4f);
        }
    }

    @Nullable
    private VenueModel getSelectedVenue() {
        if (TextUtils.isEmpty(selectedVenueId)) {
            return null;
        }
        return venuesById.get(selectedVenueId);
    }

    @NonNull
    private JSONObject buildRequestPayload(@NonNull LatLng location, @Nullable List<Wifi> observedAps) throws JSONException {
        JSONObject root = new JSONObject();
        root.put("lat", location.latitude);
        root.put("lon", location.longitude);
        JSONArray macs = new JSONArray();
        if (observedAps != null) {
            for (Wifi wifi : observedAps) {
                long bssid = wifi.getBssid();
                if (bssid > 0L) {
                    macs.put(toMacString(bssid));
                }
            }
        }
        root.put("macs", macs);
        return root;
    }

    @NonNull
    private String toMacString(long bssid) {
        String hex = String.format(Locale.US, "%012x", bssid);
        StringBuilder out = new StringBuilder(17);
        for (int i = 0; i < hex.length(); i += 2) {
            if (i > 0) {
                out.append(':');
            }
            out.append(hex, i, i + 2);
        }
        return out.toString();
    }

    @NonNull
    private String previewForLog(@Nullable String text) {
        if (text == null) {
            return "";
        }
        String normalized = text.replace('\n', ' ').replace('\r', ' ');
        if (normalized.length() <= LOG_PREVIEW_LIMIT) {
            return normalized;
        }
        return normalized.substring(0, LOG_PREVIEW_LIMIT) + "...";
    }

    @NonNull
    private List<VenueModel> parseVenueResponse(@NonNull String payload) throws JSONException {
        String trimmed = payload.trim();
        if (trimmed.isEmpty()) {
            if (BuildConfig.DEBUG) {
                Log.d(TAG, "parseVenueResponse: topLevel=empty venuesCount=0");
            }
            return Collections.emptyList();
        }

        JSONArray venueArray = null;
        String topLevel;
        if (trimmed.startsWith("[")) {
            topLevel = "array";
            venueArray = new JSONArray(trimmed);
        } else {
            topLevel = "object";
            JSONObject root = new JSONObject(trimmed);
            String[] candidateKeys = new String[]{"venues", "results", "data", "maps", "floorplans"};
            for (String key : candidateKeys) {
                venueArray = root.optJSONArray(key);
                if (venueArray != null) {
                    break;
                }
            }
            if (venueArray == null && looksLikeVenue(root)) {
                venueArray = new JSONArray().put(root);
            }
        }
        if (venueArray == null) {
            if (BuildConfig.DEBUG) {
                Log.d(TAG, "parseVenueResponse: topLevel=" + topLevel + " venuesCount=0");
            }
            return Collections.emptyList();
        }

        List<VenueModel> venues = new ArrayList<>();
        for (int i = 0; i < venueArray.length(); i++) {
            JSONObject v = venueArray.optJSONObject(i);
            if (v == null) {
                continue;
            }
            VenueModel parsed = parseVenue(v, i);
            if (parsed != null) {
                venues.add(parsed);
            }
        }
        if (BuildConfig.DEBUG) {
            Log.d(TAG, "parseVenueResponse: topLevel=" + topLevel + " venuesCount=" + venues.size());
        }
        return venues;
    }

    private boolean looksLikeVenue(@NonNull JSONObject obj) {
        return obj.has("polygon") || obj.has("outline") || obj.has("geometry") || obj.has("floors");
    }

    @Nullable
    private VenueModel parseVenue(@NonNull JSONObject obj, int index) {
        String id = firstNonEmpty(obj, "id", "venue_id", "slug", "name", "venueId");
        if (TextUtils.isEmpty(id)) {
            id = "venue_" + index;
        }
        String name = firstNonEmpty(obj, "name", "venue_name", "title", "building");
        if (TextUtils.isEmpty(name)) {
            name = id;
        }

        List<LatLng> outline = parseOutline(obj);
        LatLngBounds bounds = parseBounds(obj);
        if ((outline == null || outline.size() < 3) && bounds != null) {
            outline = boundsToPolygon(bounds);
        }
        if (BuildConfig.DEBUG) {
            int outlinePoints = outline == null ? 0 : outline.size();
            Log.d(TAG, "parsed venue: id=" + id + " outlinePoints=" + outlinePoints);
        }
        if (outline == null || outline.size() < 3) {
            return null;
        }
        if (bounds == null) {
            bounds = calculateBounds(outline);
        }
        if (bounds == null) {
            return null;
        }

        List<FloorModel> floors = parseFloors(obj, bounds);
        float parsedFloorHeight = (float) optDouble(obj, DEFAULT_FLOOR_HEIGHT_M, "floor_height", "floorHeight");
        String mapShapesPayload = toJsonPayload(obj.opt("map_shapes"));
        return new VenueModel(id, name, outline, floors, parsedFloorHeight, mapShapesPayload);
    }

    @NonNull
    private List<FloorModel> parseFloors(@NonNull JSONObject venue, @NonNull LatLngBounds fallbackBounds) {
        JSONArray arr = null;
        String[] keys = new String[]{"floors", "floorplans", "maps", "levels"};
        for (String key : keys) {
            arr = venue.optJSONArray(key);
            if (arr != null) {
                break;
            }
        }
        if (arr == null) {
            return Collections.emptyList();
        }

        List<FloorModel> floors = new ArrayList<>();
        for (int i = 0; i < arr.length(); i++) {
            JSONObject f = arr.optJSONObject(i);
            if (f == null) {
                continue;
            }
            String imageUrl = firstNonEmpty(f, "image_url", "imageUrl", "url", "floorplan_url", "map_url");
            if (TextUtils.isEmpty(imageUrl)) {
                JSONObject imageObj = f.optJSONObject("image");
                if (imageObj != null) {
                    imageUrl = firstNonEmpty(imageObj, "url", "src", "href");
                }
            }
            imageUrl = normalizeUrl(imageUrl);
            if (TextUtils.isEmpty(imageUrl)) {
                continue;
            }
            if (BuildConfig.DEBUG) {
                Log.d(TAG, "FLOOR_NET: floor imageUrl=" + imageUrl);
            }

            int idx = (int) optDouble(f, i, "floor", "index", "level", "floor_index");
            String floorName = firstNonEmpty(f, "name", "label", "title");
            if (TextUtils.isEmpty(floorName)) {
                floorName = String.format(Locale.US, "Floor %d", idx);
            }
            LatLngBounds bounds = parseBounds(f);
            if (bounds == null) {
                bounds = fallbackBounds;
            }
            if (BuildConfig.DEBUG) {
                int shapeCount = countShapeElements(f);
                if (shapeCount > 0) {
                    Log.d(TAG, "FLOOR_NET: walls/polylines/polygons count=" + shapeCount);
                }
            }
            floors.add(new FloorModel(idx, floorName, imageUrl, bounds));
        }
        if (BuildConfig.DEBUG) {
            int venueShapeCount = countShapeElements(venue);
            if (venueShapeCount > 0) {
                Log.d(TAG, "FLOOR_NET: walls/polylines/polygons count=" + venueShapeCount);
            }
        }
        floors.sort((a, b) -> Integer.compare(a.floorIndex, b.floorIndex));
        return floors;
    }

    private int countShapeElements(@NonNull JSONObject container) {
        int count = 0;
        String[] shapeKeys = new String[]{"walls", "polylines", "polygons", "shapes"};
        for (String key : shapeKeys) {
            JSONArray arr = container.optJSONArray(key);
            if (arr != null) {
                count += arr.length();
            }
        }
        Object mapShapes = container.opt("map_shapes");
        if (mapShapes instanceof JSONArray) {
            count += ((JSONArray) mapShapes).length();
        } else if (mapShapes instanceof JSONObject) {
            count += ((JSONObject) mapShapes).length();
        } else if (mapShapes instanceof String) {
            String raw = ((String) mapShapes).trim();
            try {
                if (raw.startsWith("[")) {
                    count += new JSONArray(raw).length();
                } else if (raw.startsWith("{")) {
                    count += new JSONObject(raw).length();
                }
            } catch (JSONException ignored) {
            }
        }
        return count;
    }

    @Nullable
    private String toJsonPayload(@Nullable Object value) {
        if (value == null || value == JSONObject.NULL) {
            return null;
        }
        if (value instanceof JSONObject || value instanceof JSONArray) {
            return value.toString();
        }
        if (value instanceof String) {
            String raw = ((String) value).trim();
            return raw.isEmpty() ? null : raw;
        }
        return null;
    }

    @Nullable
    private String normalizeUrl(@Nullable String url) {
        if (TextUtils.isEmpty(url)) {
            return null;
        }
        if (url.startsWith("//")) {
            return "https:" + url;
        }
        if (url.startsWith("/")) {
            return "https://openpositioning.org" + url;
        }
        return url;
    }

    @Nullable
    private List<LatLng> parseOutline(@NonNull JSONObject obj) {
        JSONArray arr = null;
        Object outlineObj = obj.opt("outline");
        if (outlineObj instanceof JSONArray) {
            arr = (JSONArray) outlineObj;
        } else if (outlineObj instanceof JSONObject) {
            arr = extractCoordinateArray((JSONObject) outlineObj);
        } else if (outlineObj instanceof String) {
            arr = parseOutlineString((String) outlineObj);
        }
        if (arr == null) {
            arr = obj.optJSONArray("polygon");
        }
        if (arr == null) {
            arr = obj.optJSONArray("coordinates");
        }
        if (arr == null) {
            JSONObject geometry = obj.optJSONObject("geometry");
            if (geometry != null) {
                arr = extractCoordinateArray(geometry);
            }
        }
        if (arr == null) {
            return null;
        }
        return parsePointArray(arr);
    }

    @Nullable
    private JSONArray parseOutlineString(@Nullable String rawOutline) {
        if (TextUtils.isEmpty(rawOutline)) {
            return null;
        }
        String trimmed = rawOutline.trim();
        if (trimmed.isEmpty() || "null".equalsIgnoreCase(trimmed)) {
            return null;
        }
        try {
            if (trimmed.startsWith("{")) {
                return extractCoordinateArray(new JSONObject(trimmed));
            }
            if (trimmed.startsWith("[")) {
                return new JSONArray(trimmed);
            }
        } catch (JSONException e) {
            if (BuildConfig.DEBUG) {
                Log.d(TAG, "Failed to parse outline string", e);
            }
        }
        return null;
    }

    @Nullable
    private JSONArray extractCoordinateArray(@NonNull JSONObject obj) {
        JSONArray coordinates = obj.optJSONArray("coordinates");
        if (coordinates != null) {
            return coordinates;
        }

        JSONObject geometry = obj.optJSONObject("geometry");
        if (geometry != null) {
            JSONArray geometryCoordinates = geometry.optJSONArray("coordinates");
            if (geometryCoordinates != null) {
                return geometryCoordinates;
            }
        }

        JSONArray features = obj.optJSONArray("features");
        if (features != null) {
            for (int i = 0; i < features.length(); i++) {
                JSONObject feature = features.optJSONObject(i);
                if (feature == null) {
                    continue;
                }
                JSONArray featureCoordinates = extractCoordinateArray(feature);
                if (featureCoordinates != null) {
                    return featureCoordinates;
                }
            }
        }
        return null;
    }

    @NonNull
    private List<LatLng> parsePointArray(@NonNull JSONArray arr) {
        if (arr.length() == 0) {
            return Collections.emptyList();
        }
        while (arr.length() > 0) {
            Object first = arr.opt(0);
            if (!(first instanceof JSONArray)) {
                break;
            }
            JSONArray firstArray = (JSONArray) first;
            if (firstArray.length() == 0) {
                break;
            }
            Object nested = firstArray.opt(0);
            if (!(nested instanceof JSONArray) && !(nested instanceof JSONObject)) {
                break;
            }
            arr = firstArray;
        }

        List<LatLng> out = new ArrayList<>();
        for (int i = 0; i < arr.length(); i++) {
            Object p = arr.opt(i);
            LatLng point = null;
            if (p instanceof JSONArray) {
                point = parsePoint((JSONArray) p);
            } else if (p instanceof JSONObject) {
                point = parsePoint((JSONObject) p);
            }
            if (point != null) {
                out.add(point);
            }
        }
        return out;
    }

    @Nullable
    private LatLng parsePoint(@Nullable JSONObject obj) {
        if (obj == null) {
            return null;
        }
        double lat = optDouble(obj, Double.NaN, "lat", "latitude", "y");
        double lon = optDouble(obj, Double.NaN, "lon", "lng", "longitude", "x");
        if (!Double.isNaN(lat) && !Double.isNaN(lon) && isValidCoord(lat, lon)) {
            return new LatLng(lat, lon);
        }
        return null;
    }

    @Nullable
    private LatLng parsePoint(@NonNull JSONArray arr) {
        if (arr.length() < 2) {
            return null;
        }
        double a = arr.optDouble(0, Double.NaN);
        double b = arr.optDouble(1, Double.NaN);
        return parseCoordinatePair(a, b);
    }

    @Nullable
    private LatLng parseCoordinatePair(double a, double b) {
        if (!Double.isFinite(a) || !Double.isFinite(b)) {
            return null;
        }
        LatLng latLon = new LatLng(a, b);
        LatLng lonLat = new LatLng(b, a);
        boolean latLonOk = isValidCoord(latLon.latitude, latLon.longitude);
        boolean lonLatOk = isValidCoord(lonLat.latitude, lonLat.longitude);
        if (latLonOk && !lonLatOk) {
            return latLon;
        }
        if (!latLonOk && lonLatOk) {
            return lonLat;
        }
        if (!latLonOk) {
            return null;
        }
        if (currentLocation == null) {
            return latLon;
        }
        return distanceMeters(currentLocation, latLon) <= distanceMeters(currentLocation, lonLat)
                ? latLon : lonLat;
    }

    @Nullable
    private LatLngBounds parseBounds(@NonNull JSONObject obj) {
        JSONArray bbox = obj.optJSONArray("bbox");
        if (bbox != null && bbox.length() >= 4) {
            double b0 = bbox.optDouble(0, Double.NaN);
            double b1 = bbox.optDouble(1, Double.NaN);
            double b2 = bbox.optDouble(2, Double.NaN);
            double b3 = bbox.optDouble(3, Double.NaN);
            LatLng sw = parseCoordinatePair(b0, b1);
            LatLng ne = parseCoordinatePair(b2, b3);
            if (sw != null && ne != null) {
                return new LatLngBounds(
                        new LatLng(Math.min(sw.latitude, ne.latitude), Math.min(sw.longitude, ne.longitude)),
                        new LatLng(Math.max(sw.latitude, ne.latitude), Math.max(sw.longitude, ne.longitude)));
            }
        }

        JSONObject bounds = obj.optJSONObject("bounds");
        if (bounds == null) {
            return null;
        }
        LatLng sw = parsePoint(bounds.optJSONObject("sw"));
        if (sw == null) {
            sw = parsePoint(bounds.optJSONObject("southWest"));
        }
        if (sw == null) {
            sw = parsePoint(bounds.optJSONObject("south_west"));
        }
        LatLng ne = parsePoint(bounds.optJSONObject("ne"));
        if (ne == null) {
            ne = parsePoint(bounds.optJSONObject("northEast"));
        }
        if (ne == null) {
            ne = parsePoint(bounds.optJSONObject("north_east"));
        }
        if (sw != null && ne != null) {
            return new LatLngBounds(sw, ne);
        }
        return null;
    }

    @Nullable
    private LatLngBounds calculateBounds(@NonNull List<LatLng> points) {
        if (points.isEmpty()) {
            return null;
        }
        double south = Double.POSITIVE_INFINITY;
        double west = Double.POSITIVE_INFINITY;
        double north = Double.NEGATIVE_INFINITY;
        double east = Double.NEGATIVE_INFINITY;
        for (LatLng p : points) {
            south = Math.min(south, p.latitude);
            west = Math.min(west, p.longitude);
            north = Math.max(north, p.latitude);
            east = Math.max(east, p.longitude);
        }
        return new LatLngBounds(new LatLng(south, west), new LatLng(north, east));
    }

    @NonNull
    private List<LatLng> boundsToPolygon(@NonNull LatLngBounds bounds) {
        List<LatLng> points = new ArrayList<>(4);
        points.add(new LatLng(bounds.southwest.latitude, bounds.southwest.longitude));
        points.add(new LatLng(bounds.southwest.latitude, bounds.northeast.longitude));
        points.add(new LatLng(bounds.northeast.latitude, bounds.northeast.longitude));
        points.add(new LatLng(bounds.northeast.latitude, bounds.southwest.longitude));
        return points;
    }

    @Nullable
    private String firstNonEmpty(@NonNull JSONObject obj, @NonNull String... keys) {
        for (String key : keys) {
            String value = obj.optString(key, null);
            if (!TextUtils.isEmpty(value) && !"null".equalsIgnoreCase(value)) {
                return value;
            }
        }
        return null;
    }

    private double optDouble(@NonNull JSONObject obj, double fallback, @NonNull String... keys) {
        for (String key : keys) {
            if (!obj.has(key)) {
                continue;
            }
            Object raw = obj.opt(key);
            if (raw instanceof Number) {
                return ((Number) raw).doubleValue();
            }
            if (raw instanceof String) {
                try {
                    return Double.parseDouble((String) raw);
                } catch (NumberFormatException ignored) {
                }
            }
        }
        return fallback;
    }

    private boolean isValidCoord(double lat, double lon) {
        return lat >= -90 && lat <= 90 && lon >= -180 && lon <= 180;
    }

    @NonNull
    private VenueModel choosePreferredVenue(@NonNull List<VenueModel> venues) {
        if (venues.isEmpty()) {
            throw new IllegalArgumentException("venues must not be empty");
        }
        if (currentLocation == null) {
            return venues.get(0);
        }

        VenueModel nearestVenue = venues.get(0);
        float nearestDistance = Float.MAX_VALUE;
        for (VenueModel venue : venues) {
            if (venue.outline.size() >= 3 && isPointInsidePolygon(currentLocation, venue.outline)) {
                return venue;
            }
            LatLng centroid = computePolygonCentroid(venue.outline);
            if (centroid == null) {
                continue;
            }
            float distance = distanceMeters(currentLocation, centroid);
            if (distance < nearestDistance) {
                nearestDistance = distance;
                nearestVenue = venue;
            }
        }
        return nearestVenue;
    }

    @Nullable
    private LatLng computePolygonCentroid(@NonNull List<LatLng> polygon) {
        if (polygon.isEmpty()) {
            return null;
        }
        double latitudeSum = 0.0;
        double longitudeSum = 0.0;
        for (LatLng point : polygon) {
            latitudeSum += point.latitude;
            longitudeSum += point.longitude;
        }
        return new LatLng(latitudeSum / polygon.size(), longitudeSum / polygon.size());
    }

    private boolean isPointInsidePolygon(@NonNull LatLng point, @NonNull List<LatLng> polygon) {
        if (polygon.size() < 3) {
            return false;
        }

        int crossings = 0;
        for (int i = 0; i < polygon.size(); i++) {
            LatLng a = polygon.get(i);
            LatLng b = polygon.get((i + 1) % polygon.size());
            if (isRayCrossingSegment(point, a, b)) {
                crossings++;
            }
        }
        return (crossings % 2) == 1;
    }

    private boolean isRayCrossingSegment(
            @NonNull LatLng point,
            @NonNull LatLng a,
            @NonNull LatLng b
    ) {
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
        }
        if (pointLng < Math.min(aLng, bLng)) {
            return true;
        }
        double slope1 = (aLng != bLng) ? ((bLat - aLat) / (bLng - aLng)) : Double.POSITIVE_INFINITY;
        double slope2 = (aLng != pointLng) ? ((pointLat - aLat) / (pointLng - aLng)) : Double.POSITIVE_INFINITY;
        return slope2 >= slope1;
    }

    private float distanceMeters(@NonNull LatLng a, @NonNull LatLng b) {
        float[] result = new float[1];
        Location.distanceBetween(a.latitude, a.longitude, b.latitude, b.longitude, result);
        return result[0];
    }

    private static final class VenueModel {
        final String id;
        final String name;
        final List<LatLng> outline;
        final List<FloorModel> floors;
        final float floorHeight;
        @Nullable
        final String mapShapesPayload;

        VenueModel(String id, String name, List<LatLng> outline, List<FloorModel> floors,
                   float floorHeight, @Nullable String mapShapesPayload) {
            this.id = id;
            this.name = name;
            this.outline = outline;
            this.floors = floors;
            this.floorHeight = floorHeight;
            this.mapShapesPayload = mapShapesPayload;
        }
    }

    private static final class FloorModel {
        final int floorIndex;
        final String floorName;
        final String imageUrl;
        final LatLngBounds bounds;

        FloorModel(int floorIndex, String floorName, String imageUrl, LatLngBounds bounds) {
            this.floorIndex = floorIndex;
            this.floorName = floorName;
            this.imageUrl = imageUrl;
            this.bounds = bounds;
        }
    }
}
