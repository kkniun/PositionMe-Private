package com.openpositioning.PositionMe.presentation.fragment;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Spinner;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;

import com.google.android.gms.maps.OnMapReadyCallback;
import com.openpositioning.PositionMe.R;
import com.openpositioning.PositionMe.sensors.SensorFusion;
import com.openpositioning.PositionMe.utils.FloorDisplaySyncPolicy;
import com.openpositioning.PositionMe.utils.IndoorMapManager;
import com.openpositioning.PositionMe.utils.UtilFunctions;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.Circle;
import com.google.android.gms.maps.model.CircleOptions;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.model.Polygon;
import com.google.android.gms.maps.model.Polyline;
import com.google.android.gms.maps.model.PolylineOptions;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;


/**
 * A fragment responsible for displaying a trajectory map using Google Maps.
 * <p>
 * The TrajectoryMapFragment provides a map interface for visualizing movement trajectories,
 * GNSS tracking, and indoor mapping. It manages map settings, user interactions, and real-time
 * updates to user location and GNSS markers.
 * <p>
 * Key Features:
 * - Displays a Google Map with support for different map types (Hybrid, Normal, Satellite).
 * - Tracks and visualizes user movement using polylines.
 * - Supports GNSS position updates and visual representation.
 * - Includes indoor mapping with floor selection and auto-floor adjustments.
 * - Allows user interaction through map controls and UI elements.
 *
 * @see com.openpositioning.PositionMe.presentation.activity.RecordingActivity The activity hosting this fragment.
 * @see com.openpositioning.PositionMe.utils.IndoorMapManager Utility for managing indoor map overlays.
 * @see com.openpositioning.PositionMe.utils.UtilFunctions Utility functions for UI and graphics handling.
 *
 * @author Mate Stodulka
 */

public class TrajectoryMapFragment extends Fragment {

    private static final float DIRECTION_MARKER_SIZE_DP = 18f;
    private static final double MIN_DIRECTION_DISTANCE_METERS = 0.20;
    private static final double DISPLAY_SMOOTHING_ALPHA = 0.18;
    private static final double MAX_DISPLAY_STEP_METERS = 1.0;
    private static final double DISPLAY_SNAP_JUMP_METERS = 10.0;
    private static final double HEADING_MOVEMENT_MIN_DISTANCE_METERS = 0.80;
    private static final float HEADING_SENSOR_SMOOTH_ALPHA = 0.16f;
    private static final float HEADING_COURSE_BLEND_ALPHA = 0.18f;
    private static final double CAMERA_RECENTER_DISTANCE_METERS = 4.0;
    private static final long CAMERA_RECENTER_INTERVAL_MS = 1_500L;
    private static final int MAX_TRACK_HISTORY_POINTS = 600;
    private static final double TRACK_APPEND_DISTANCE_METERS = 0.06;
    private static final double TRACK_REPLACE_DISTANCE_METERS = 0.03;
    private static final double MAX_TRACK_APPEND_DISTANCE_METERS = 6.0;
    private static final double TRACK_SEGMENT_INTERPOLATION_METERS = 3.0;
    private static final double MAP_CORRECTION_MIN_METERS = 0.12;
    private static final double MAP_CORRECTION_MAX_METERS = 12.0;
    private static final double HISTORY_POINT_MATCH_TOLERANCE_METERS = 0.08;
    private GoogleMap gMap; // Google Maps instance
    private LatLng currentLocation; // Stores the user's current location
    private LatLng currentRawLocation;
    private LatLng smoothedDisplayLocation;
    private Marker directionMarker; // Current user direction arrow
    // Keep test point markers so they can be cleared when recording ends
    private final List<com.google.android.gms.maps.model.Marker> testPointMarkers = new ArrayList<>();
    private final List<Circle> gnssTailCircles = new ArrayList<>();
    private final List<Circle> wifiTailCircles = new ArrayList<>();
    private final List<Circle> pdrTailCircles = new ArrayList<>();

    private Polyline polyline; // Polyline representing user's movement path
    private boolean isGnssOn = true; // GNSS display stays enabled during recording

    private LatLng pendingCameraPosition = null; // Stores pending camera movement
    private boolean hasPendingCameraMove = false; // Tracks if camera needs to move

    private IndoorMapManager indoorMapManager; // Manages indoor mapping
    private SensorFusion sensorFusion;
    private float lastDirectionDegrees = 0f;
    private float filteredHeadingDegrees = Float.NaN;
    private LatLng lastHeadingLocation;
    private LatLng lastCameraLocation;
    private long lastCameraUpdateMs;
    private final Map<Integer, List<LatLng>> userTrackHistoryByFloor = new HashMap<>();
    private int activeTrackFloor = Integer.MIN_VALUE;

    // Auto-floor state
    private static final String TAG = "TrajectoryMapFragment";
    private static final long AUTO_FLOOR_CHECK_INTERVAL_MS = 400L;
    private static final long AUTO_FLOOR_STABLE_MS = 1_200L;
    private static final double AUTO_FLOOR_STAIRS_SYNC_PROXIMITY_METERS = 5.0;
    private static final double AUTO_FLOOR_LIFT_SYNC_PROXIMITY_METERS = 4.5;
    private static final double TAIL_RADIUS_METERS = 0.45;
    private static final int GNSS_TAIL_COLOR = Color.rgb(25, 118, 210);
    private static final int WIFI_TAIL_COLOR = Color.rgb(0, 137, 123);
    private static final int PDR_TAIL_COLOR = Color.rgb(255, 111, 0);
    private Handler autoFloorHandler;
    private Runnable autoFloorTask;
    @Nullable
    private Integer pendingDisplayFloorCandidate;
    private boolean pendingDisplayFloorCommitToFusion;
    private boolean pendingDisplayFloorCompletesInitialSync;
    private long pendingDisplayFloorSinceMs;
    private boolean hasCommittedDisplayFloorSync;

    // UI
    private View mapControlsCard;
    private Spinner switchMapSpinner;

    private com.google.android.material.floatingactionbutton.FloatingActionButton floorUpButton, floorDownButton;
    private TextView floorLabel;
    private boolean previewFloorOnlyMode;


    public TrajectoryMapFragment() {
        // Required empty public constructor
    }

    @Nullable
    @Override
    public View onCreateView(@NonNull LayoutInflater inflater,
                             @Nullable ViewGroup container,
                             @Nullable Bundle savedInstanceState) {
        // Inflate the separate layout containing map + map-related UI
        return inflater.inflate(R.layout.fragment_trajectory_map, container, false);
    }

    @Override
    public void onViewCreated(@NonNull View view,
                              @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);

        // Grab references to UI controls
        mapControlsCard  = view.findViewById(R.id.mapControlsCard);
        switchMapSpinner = view.findViewById(R.id.mapSwitchSpinner);
        floorUpButton   = view.findViewById(R.id.floorUpButton);
        floorDownButton = view.findViewById(R.id.floorDownButton);
        floorLabel      = view.findViewById(R.id.floorLabel);
        applyOverlayMode();

        // Setup floor up/down UI hidden initially until we know there's an indoor map
        setFloorControlsVisibility(View.GONE);

        // Initialize the map asynchronously
        SupportMapFragment mapFragment = (SupportMapFragment)
                getChildFragmentManager().findFragmentById(R.id.trajectoryMap);
        if (mapFragment != null) {
            mapFragment.getMapAsync(new OnMapReadyCallback() {
                @Override
                public void onMapReady(@NonNull GoogleMap googleMap) {
                    // Assign the provided googleMap to your field variable
                    gMap = googleMap;
                    // Initialize map settings with the now non-null gMap
                    initMapSettings(gMap);

                    // If we had a pending camera move, apply it now
                    if (hasPendingCameraMove && pendingCameraPosition != null) {
                        gMap.moveCamera(CameraUpdateFactory.newLatLngZoom(pendingCameraPosition, 19f));
                        hasPendingCameraMove = false;
                        pendingCameraPosition = null;
                    }

                    drawBuildingPolygon();

                    Log.d("TrajectoryMapFragment", "onMapReady: Map is ready!");


                }
            });
        }

        // Map type spinner setup
        initMapTypeSpinner();

        sensorFusion = SensorFusion.getInstance();
        configureAlwaysOnControls();
        startAutoFloor();

        floorUpButton.setOnClickListener(v -> {
            if (indoorMapManager != null) {
                indoorMapManager.increaseFloor();
                sensorFusion.setCurrentLogicalFloor(indoorMapManager.getCurrentLogicalFloor());
                syncActiveTrackFloorWithDisplayedFloor();
                updateFloorLabel();
            }
        });

        floorDownButton.setOnClickListener(v -> {
            if (indoorMapManager != null) {
                indoorMapManager.decreaseFloor();
                sensorFusion.setCurrentLogicalFloor(indoorMapManager.getCurrentLogicalFloor());
                syncActiveTrackFloorWithDisplayedFloor();
                updateFloorLabel();
            }
        });
    }

    public void setPreviewFloorOnlyMode(boolean enabled) {
        previewFloorOnlyMode = enabled;
        applyOverlayMode();
    }

    private void configureAlwaysOnControls() {
        isGnssOn = true;
    }

    /**
     * Initialize the map settings with the provided GoogleMap instance.
     * <p>
     *     The method sets basic map settings, initializes the indoor map manager,
     *     and creates an empty polyline for fused movement tracking.
     *     The method sets the map type to Hybrid and initializes the map with these settings.
     *
     * @param map
     */

    private void initMapSettings(GoogleMap map) {
        // Basic map settings
        map.getUiSettings().setCompassEnabled(true);
        map.getUiSettings().setTiltGesturesEnabled(true);
        map.getUiSettings().setRotateGesturesEnabled(true);
        map.getUiSettings().setScrollGesturesEnabled(true);
        map.setMapType(GoogleMap.MAP_TYPE_HYBRID);

        // Initialize indoor manager
        indoorMapManager = new IndoorMapManager(map);

        // Initialize an empty polyline
        polyline = map.addPolyline(new PolylineOptions()
                .color(Color.RED)
                .width(5f)
                .add() // start empty
        );
    }


    /**
     * Initialize the map type spinner with the available map types.
     * <p>
     *     The spinner allows the user to switch between different map types
     *     (e.g. Hybrid, Normal, Satellite) to customize their map view.
     *     The spinner is populated with the available map types and listens
     *     for user selection to update the map accordingly.
     *     The map type is updated directly on the GoogleMap instance.
     *     <p>
     *         Note: The spinner is initialized with the default map type (Hybrid).
     *         The map type is updated on user selection.
     *     </p>
     * </p>
     *     @see com.google.android.gms.maps.GoogleMap The GoogleMap instance to update map type.
     */
    private void initMapTypeSpinner() {
        if (switchMapSpinner == null) return;
        String[] maps = new String[]{
                getString(R.string.hybrid),
                getString(R.string.normal),
                getString(R.string.satellite)
        };
        ArrayAdapter<String> adapter = new ArrayAdapter<>(
                requireContext(),
                android.R.layout.simple_spinner_dropdown_item,
                maps
        );
        switchMapSpinner.setAdapter(adapter);

        switchMapSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view,
                                       int position, long id) {
                if (gMap == null) return;
                switch (position){
                    case 0:
                        gMap.setMapType(GoogleMap.MAP_TYPE_HYBRID);
                        break;
                    case 1:
                        gMap.setMapType(GoogleMap.MAP_TYPE_NORMAL);
                        break;
                    case 2:
                        gMap.setMapType(GoogleMap.MAP_TYPE_SATELLITE);
                        break;
                }
            }
            @Override
            public void onNothingSelected(AdapterView<?> parent) {}
        });
    }

    /**
     * Update the user's current location on the map, create or move orientation marker,
     * and append to polyline if the user actually moved.
     *
     * @param newLocation The new location to plot.
     * @param orientation The user’s heading (e.g. from sensor fusion).
     */
    public LatLng updateUserLocation(@NonNull LatLng newLocation, float orientation) {
        if (gMap == null) return newLocation;

        LatLng previousRawLocation = currentRawLocation != null ? currentRawLocation : currentLocation;
        LatLng displaySourceLocation = newLocation;
        if (indoorMapManager != null) {
            indoorMapManager.setCurrentLocation(newLocation);
            syncDisplayedFloor(newLocation);
            syncActiveTrackFloorWithDisplayedFloor();
            setFloorControlsVisibility(indoorMapManager.getIsIndoorMapSet() ? View.VISIBLE : View.GONE);
            displaySourceLocation = indoorMapManager.constrainPositionToLegalSpace(
                    previousRawLocation,
                    newLocation
            );
            maybeApplyMapCorrection(newLocation, displaySourceLocation);
            indoorMapManager.setCurrentLocation(displaySourceLocation);
        }

        currentRawLocation = displaySourceLocation;
        LatLng previousDisplayLocation = currentLocation;
        LatLng displayLocation = resolveDisplayLocation(displaySourceLocation);
        float resolvedDirection = resolveDisplayHeading(displayLocation, orientation);
        boolean insignificantMove = previousDisplayLocation != null
                && UtilFunctions.distanceBetweenPoints(previousDisplayLocation, displayLocation) < 0.18;
        boolean insignificantRotation = directionMarker != null
                && absoluteBearingDelta(lastDirectionDegrees, resolvedDirection) < 2.5f;
        this.currentLocation = displayLocation;

        if (insignificantMove && insignificantRotation) {
            updateTrackHistory(displaySourceLocation);
            return displayLocation;
        }

        if (directionMarker == null) {
            updateDirectionMarker(displayLocation, resolvedDirection);
            gMap.moveCamera(CameraUpdateFactory.newLatLngZoom(displayLocation, 19f));
            lastCameraLocation = displayLocation;
            lastCameraUpdateMs = SystemClock.elapsedRealtime();
        } else {
            updateDirectionMarker(displayLocation, resolvedDirection);
            long now = SystemClock.elapsedRealtime();
            boolean movedEnough = lastCameraLocation == null
                    || UtilFunctions.distanceBetweenPoints(lastCameraLocation, displayLocation)
                    >= CAMERA_RECENTER_DISTANCE_METERS;
            if (movedEnough && now - lastCameraUpdateMs >= CAMERA_RECENTER_INTERVAL_MS) {
                gMap.moveCamera(CameraUpdateFactory.newLatLng(displayLocation));
                lastCameraLocation = displayLocation;
                lastCameraUpdateMs = now;
            }
        }

        updateTrackHistory(displaySourceLocation);
        return displayLocation;
    }

    public boolean isRenderSurfaceReady() {
        return gMap != null && polyline != null;
    }



    /**
     * Set the initial camera position for the map.
     * <p>
     *     The method sets the initial camera position for the map when it is first loaded.
     *     If the map is already ready, the camera is moved immediately.
     *     If the map is not ready, the camera position is stored until the map is ready.
     *     The method also tracks if there is a pending camera move.
     * </p>
     * @param startLocation The initial camera position to set.
     */
    public void setInitialCameraPosition(@NonNull LatLng startLocation) {
        // If the map is already ready, move camera immediately
        if (gMap != null) {
            gMap.moveCamera(CameraUpdateFactory.newLatLngZoom(startLocation, 19f));
        } else {
            // Otherwise, store it until onMapReady
            pendingCameraPosition = startLocation;
            hasPendingCameraMove = true;
        }
    }


    /**
     * Get the current user location on the map.
     * @return The current user location as a LatLng object.
     */
    public LatLng getCurrentLocation() {
        return currentLocation;
    }

    /**
     * Add a numbered test point marker on the map.
     * Called by RecordingFragment when user presses the "Test Point" button.
     */
    public void addTestPointMarker(int index, long timestampMs, @NonNull LatLng position) {
        if (gMap == null) return;

        com.google.android.gms.maps.model.Marker m = gMap.addMarker(new com.google.android.gms.maps.model.MarkerOptions()
                .position(position)
                .title("TP " + index)
                .snippet("t=" + timestampMs));

        if (m != null) {
            m.showInfoWindow(); // Show TP index immediately
            testPointMarkers.add(m);
        }
    }


    public void updateGNSS(@NonNull LatLng gnssLocation) {
        // GNSS is rendered as recent observation dots only.
    }

    public void clearGNSS() {
        // GNSS is rendered as recent observation dots only.
    }

    /**
     * Whether user is currently showing GNSS or not
     */
    public boolean isGnssEnabled() {
        return isGnssOn;
    }

    public void renderFusedHistory(@Nullable List<LatLng> fusedHistory) {
        if (polyline == null) return;
        if (fusedHistory == null) {
            clearTrackHistory();
            return;
        }
        if (fusedHistory.isEmpty()) {
            clearTrackHistory();
            return;
        }

        List<LatLng> normalizedHistory = normalizeHistory(fusedHistory);
        if (normalizedHistory.isEmpty()) {
            clearTrackHistory();
            return;
        }

        rebuildTrackHistory(normalizedHistory);
        if (currentLocation == null) {
            LatLng lastPoint = normalizedHistory.get(normalizedHistory.size() - 1);
            currentRawLocation = lastPoint;
            smoothedDisplayLocation = lastPoint;
            currentLocation = lastPoint;
        }
        if (directionMarker != null && Float.isNaN(filteredHeadingDegrees)) {
            updateDirectionFromHistory(normalizedHistory);
        }
        refreshDisplayedTrackPolyline();
    }

    public void renderObservationTails(@Nullable List<LatLng> gnssTrail,
                                       @Nullable List<LatLng> wifiTrail,
                                       @Nullable List<LatLng> pdrTrail) {
        if (gMap == null) return;
        redrawTailCircles(gnssTailCircles, gnssTrail, GNSS_TAIL_COLOR);
        redrawTailCircles(wifiTailCircles, wifiTrail, WIFI_TAIL_COLOR);
        redrawTailCircles(pdrTailCircles, pdrTrail, PDR_TAIL_COLOR);
    }

    private void setFloorControlsVisibility(int visibility) {
        floorLabel.setVisibility(visibility);
        if (previewFloorOnlyMode) {
            floorUpButton.setVisibility(View.GONE);
            floorDownButton.setVisibility(View.GONE);
        } else {
            floorUpButton.setVisibility(visibility);
            floorDownButton.setVisibility(visibility);
        }
        if (visibility == View.VISIBLE) {
            updateFloorLabel();
        }
    }

    /**
     * Updates the floor label text to reflect the current floor display name.
     */
    private void updateFloorLabel() {
        if (floorLabel == null) {
            return;
        }
        if (indoorMapManager != null && indoorMapManager.getIsIndoorMapSet()) {
            floorLabel.setText(indoorMapManager.getCurrentFloorDisplayName());
            return;
        }
        if (sensorFusion != null && sensorFusion.isIndoorContextActive()) {
            floorLabel.setText(sensorFusion.getCurrentFloorDisplayName());
        }
    }

    public void clearMapAndReset() {
        stopAutoFloor();
        if (polyline != null) {
            polyline.remove();
            polyline = null;
        }
        if (directionMarker != null) {
            directionMarker.remove();
            directionMarker = null;
        }
        currentLocation  = null;
        currentRawLocation = null;
        smoothedDisplayLocation = null;
        lastDirectionDegrees = 0f;
        filteredHeadingDegrees = Float.NaN;
        lastHeadingLocation = null;
        lastCameraLocation = null;
        lastCameraUpdateMs = 0L;
        userTrackHistoryByFloor.clear();
        activeTrackFloor = Integer.MIN_VALUE;
        hasCommittedDisplayFloorSync = false;
        clearPendingDisplayFloorDecision();

        // Clear test point markers
        for (com.google.android.gms.maps.model.Marker m : testPointMarkers) {
            m.remove();
        }
        testPointMarkers.clear();
        clearCircles(gnssTailCircles);
        clearCircles(wifiTailCircles);
        clearCircles(pdrTailCircles);


        // Re-create empty polylines with your chosen colors
        if (gMap != null) {
            polyline = gMap.addPolyline(new PolylineOptions()
                    .color(Color.RED)
                    .width(5f)
                    .add());
        }
    }

    /**
     * Draw the building polygon on the map
     * <p>
     *     The method draws a polygon representing the building on the map.
     *     The polygon is drawn with specific vertices and colors to represent
     *     different buildings or areas on the map.
     *     The method removes the old polygon if it exists and adds the new polygon
     *     to the map with the specified options.
     *     The method logs the number of vertices in the polygon for debugging.
     *     <p>
     *
     *    Note: The method uses hard-coded vertices for the building polygon.
     *
     *    </p>
     *
     *    See: {@link com.google.android.gms.maps.model.PolygonOptions} The options for the new polygon.
     */
    private void drawBuildingPolygon() {
        if (gMap == null) {
            Log.e("TrajectoryMapFragment", "GoogleMap is not ready");
            return;
        }
        indoorMapManager.setIndicationOfIndoorMap();
    }

    //region Auto-floor logic

    /**
     * Starts the periodic auto-floor evaluation task.
     * Strict floor evaluation runs every few hundred milliseconds; display-floor
     * promotion still waits for a short stability window to avoid jitter.
     */
    private void startAutoFloor() {
        if (autoFloorHandler == null) {
            autoFloorHandler = new Handler(Looper.getMainLooper());
        }
        if (autoFloorTask != null) {
            autoFloorHandler.removeCallbacks(autoFloorTask);
        }

        applyImmediateFloor();

        autoFloorTask = new Runnable() {
            @Override
            public void run() {
                evaluateAutoFloor();
                autoFloorHandler.postDelayed(this, AUTO_FLOOR_CHECK_INTERVAL_MS);
            }
        };
        autoFloorHandler.post(autoFloorTask);
        Log.d(TAG, "Auto-floor started");
    }

    /**
     * Applies the best-guess floor immediately without debounce.
     * Called once when auto-floor is first toggled on, so the user
     * sees an instant correction after manually browsing wrong floors.
     */
    private void applyImmediateFloor() {
        if (sensorFusion == null || indoorMapManager == null) return;
        if (!indoorMapManager.getIsIndoorMapSet()) return;

        FloorDisplaySyncPolicy.Decision initialDecision = resolveDisplayFloorDecision(
                getFloorProbeLocation()
        );
        if (initialDecision != null) {
            applyDisplayedFloor(initialDecision.getLogicalFloor(), false, false);
            return;
        }
        applyDisplayedFloor(sensorFusion.getPreferredDisplayLogicalFloor(), false, false);
    }

    /**
     * Stops the periodic auto-floor evaluation and resets debounce state.
     */
    private void stopAutoFloor() {
        if (autoFloorHandler != null && autoFloorTask != null) {
            autoFloorHandler.removeCallbacks(autoFloorTask);
        }
        clearPendingDisplayFloorDecision();
        Log.d(TAG, "Auto-floor stopped");
    }

    /**
     * Evaluates the current floor using barometric/semantic evidence first,
     * then lets the display-floor sync policy handle stable WiFi-led alignment.
     */
    private void evaluateAutoFloor() {
        if (sensorFusion == null || indoorMapManager == null) return;
        if (!indoorMapManager.getIsIndoorMapSet()) return;

        LatLng floorProbeLocation = getFloorProbeLocation();
        if (floorProbeLocation == null) return;
        Integer resolvedFloor = sensorFusion.evaluateIndoorFloorChange(
                floorProbeLocation,
                SystemClock.elapsedRealtime()
        );
        if (resolvedFloor != null) {
            clearPendingDisplayFloorDecision();
            applyDisplayedFloor(resolvedFloor, false, true);
            return;
        }
        syncDisplayedFloor(getFloorProbeLocation());
    }

    //endregion

    private void redrawTailCircles(List<Circle> targetCircles,
                                   @Nullable List<LatLng> points,
                                   int baseColor) {
        clearCircles(targetCircles);
        if (points == null || points.isEmpty() || gMap == null) {
            return;
        }

        int count = points.size();
        for (int i = 0; i < count; i++) {
            LatLng point = points.get(i);
            float ratio = (i + 1f) / count;
            int strokeColor = withAlpha(baseColor, (int) (80 + ratio * 140));
            int fillColor = withAlpha(baseColor, (int) (25 + ratio * 90));
            targetCircles.add(gMap.addCircle(new CircleOptions()
                    .center(point)
                    .radius(TAIL_RADIUS_METERS)
                    .strokeWidth(1.4f)
                    .strokeColor(strokeColor)
                    .fillColor(fillColor)
                    .zIndex(3f)));
        }
    }

    private void syncDisplayedFloor() {
        syncDisplayedFloor(getFloorProbeLocation());
    }

    private void syncDisplayedFloor(@Nullable LatLng probeLocation) {
        if (sensorFusion == null || indoorMapManager == null) {
            return;
        }
        if (!indoorMapManager.getIsIndoorMapSet()) {
            return;
        }

        alignMapFloorToPreferredDisplayFloor();

        FloorDisplaySyncPolicy.Decision decision = resolveDisplayFloorDecision(probeLocation);
        if (decision == null) {
            return;
        }
        if (!hasCommittedDisplayFloorSync) {
            alignDisplayedFloorImmediately(decision);
        }
        maybeApplyDisplayFloorDecision(decision, SystemClock.elapsedRealtime());
    }

    private void alignMapFloorToPreferredDisplayFloor() {
        if (sensorFusion == null || indoorMapManager == null) {
            return;
        }
        int preferredDisplayFloor = sensorFusion.getPreferredDisplayLogicalFloor();
        if (indoorMapManager.getCurrentLogicalFloor() == preferredDisplayFloor) {
            return;
        }

        // 地图显示层要跟随融合层选出的 display floor，避免 UI 长时间卡在 GF。
        indoorMapManager.setCurrentFloor(preferredDisplayFloor, true);
        syncActiveTrackFloorWithDisplayedFloor();
        updateFloorLabel();
    }

    private void alignDisplayedFloorImmediately(@NonNull FloorDisplaySyncPolicy.Decision decision) {
        if (indoorMapManager == null || !indoorMapManager.getIsIndoorMapSet()) {
            return;
        }
        if (indoorMapManager.getCurrentLogicalFloor() == decision.getLogicalFloor()) {
            return;
        }

        // 首次同步时先修正地图显示层，避免右下角长时间停留在默认 GF。
        indoorMapManager.setCurrentFloor(decision.getLogicalFloor(), true);
        syncActiveTrackFloorWithDisplayedFloor();
        updateFloorLabel();
    }

    @Nullable
    private FloorDisplaySyncPolicy.Decision resolveDisplayFloorDecision(@Nullable LatLng probeLocation) {
        if (sensorFusion == null) {
            return null;
        }

        boolean hasWifiFloor = sensorFusion.getLatLngWifiPositioning() != null;
        int fusedFloor = sensorFusion.getCurrentLogicalFloor();
        int wifiFloor = hasWifiFloor ? sensorFusion.getWifiFloor() : fusedFloor;
        return FloorDisplaySyncPolicy.resolve(
                hasCommittedDisplayFloorSync,
                hasWifiFloor,
                wifiFloor,
                fusedFloor,
                isNearTransitionFeature(probeLocation)
        );
    }

    private void maybeApplyDisplayFloorDecision(@NonNull FloorDisplaySyncPolicy.Decision decision,
                                                long now) {
        if (sensorFusion == null || indoorMapManager == null) {
            return;
        }

        int displayedFloor = indoorMapManager.getCurrentLogicalFloor();
        boolean needsFusionCommit = decision.shouldCommitToFusion()
                && sensorFusion.getCurrentLogicalFloor() != decision.getLogicalFloor();
        boolean needsInitialSyncCompletion = decision.shouldCompleteInitialSync()
                && !hasCommittedDisplayFloorSync;

        if (!decision.requiresStability()) {
            clearPendingDisplayFloorDecision();
            applyDisplayedFloor(
                    decision.getLogicalFloor(),
                    needsFusionCommit,
                    needsInitialSyncCompletion
            );
            return;
        }

        if (displayedFloor == decision.getLogicalFloor()
                && !needsFusionCommit
                && !needsInitialSyncCompletion) {
            clearPendingDisplayFloorDecision();
            updateFloorLabel();
            return;
        }

        boolean samePending = pendingDisplayFloorCandidate != null
                && pendingDisplayFloorCandidate == decision.getLogicalFloor()
                && pendingDisplayFloorCommitToFusion == needsFusionCommit
                && pendingDisplayFloorCompletesInitialSync == needsInitialSyncCompletion;
        if (!samePending) {
            pendingDisplayFloorCandidate = decision.getLogicalFloor();
            pendingDisplayFloorCommitToFusion = needsFusionCommit;
            pendingDisplayFloorCompletesInitialSync = needsInitialSyncCompletion;
            pendingDisplayFloorSinceMs = now;
            return;
        }

        if (now - pendingDisplayFloorSinceMs < AUTO_FLOOR_STABLE_MS) {
            return;
        }

        applyDisplayedFloor(
                decision.getLogicalFloor(),
                needsFusionCommit,
                needsInitialSyncCompletion
        );
        clearPendingDisplayFloorDecision();
    }

    private void applyDisplayedFloor(int logicalFloor,
                                     boolean commitToFusion,
                                     boolean completeInitialSync) {
        if (sensorFusion != null && commitToFusion
                && sensorFusion.getCurrentLogicalFloor() != logicalFloor) {
            sensorFusion.setCurrentLogicalFloor(logicalFloor);
        }
        if (indoorMapManager != null) {
            indoorMapManager.setCurrentFloor(logicalFloor, true);
        }
        syncActiveTrackFloorWithDisplayedFloor();
        updateFloorLabel();
        if (commitToFusion || completeInitialSync) {
            hasCommittedDisplayFloorSync = true;
        }
    }

    private void clearPendingDisplayFloorDecision() {
        pendingDisplayFloorCandidate = null;
        pendingDisplayFloorCommitToFusion = false;
        pendingDisplayFloorCompletesInitialSync = false;
        pendingDisplayFloorSinceMs = 0L;
    }

    @Nullable
    private LatLng getFloorProbeLocation() {
        return currentRawLocation != null ? currentRawLocation : currentLocation;
    }

    private boolean isNearTransitionFeature(@Nullable LatLng position) {
        if (position == null || sensorFusion == null) {
            return false;
        }
        return sensorFusion.isNearIndoorFeature(
                position,
                "stairs",
                AUTO_FLOOR_STAIRS_SYNC_PROXIMITY_METERS
        ) || sensorFusion.isNearIndoorFeature(
                position,
                "lift",
                AUTO_FLOOR_LIFT_SYNC_PROXIMITY_METERS
        );
    }

    private void maybeApplyMapCorrection(@NonNull LatLng fusedLocation,
                                         @NonNull LatLng constrainedLocation) {
        if (sensorFusion == null) {
            return;
        }
        double correctionMeters = UtilFunctions.distanceBetweenPoints(fusedLocation, constrainedLocation);
        if (correctionMeters < MAP_CORRECTION_MIN_METERS || correctionMeters > MAP_CORRECTION_MAX_METERS) {
            return;
        }
        // 将地图上的合法化修正回灌到融合状态，避免显示位置与滤波状态持续漂移。
        sensorFusion.applyMapConstrainedPosition(constrainedLocation);
    }

    private void clearCircles(List<Circle> circles) {
        for (Circle circle : circles) {
            circle.remove();
        }
        circles.clear();
    }

    private int withAlpha(int color, int alpha) {
        return (color & 0x00FFFFFF) | (Math.max(0, Math.min(255, alpha)) << 24);
    }

    private void updateDirectionMarker(@NonNull LatLng position, float directionDegrees) {
        if (gMap == null || getContext() == null) {
            return;
        }

        if (directionMarker == null) {
            directionMarker = gMap.addMarker(new MarkerOptions()
                    .position(position)
                    .flat(true)
                    .anchor(0.5f, 0.5f)
                    .icon(BitmapDescriptorFactory.fromBitmap(getDirectionBitmap()))
                    .zIndex(6f));
        } else {
            directionMarker.setPosition(position);
        }
        directionMarker.setRotation(directionDegrees);
    }

    private void updateDirectionFromHistory(@Nullable List<LatLng> fusedHistory) {
        if (fusedHistory == null || fusedHistory.size() < 2 || directionMarker == null) {
            return;
        }

        if (!Float.isNaN(filteredHeadingDegrees)) {
            lastDirectionDegrees = normalizeDegrees(filteredHeadingDegrees);
            directionMarker.setRotation(lastDirectionDegrees);
            return;
        }

        LatLng last = fusedHistory.get(fusedHistory.size() - 1);
        for (int i = fusedHistory.size() - 2; i >= 0; i--) {
            LatLng candidate = fusedHistory.get(i);
            if (UtilFunctions.distanceBetweenPoints(candidate, last) >= MIN_DIRECTION_DISTANCE_METERS) {
                float trackDirection = computeHeadingDegrees(candidate, last);
                lastDirectionDegrees = trackDirection;
                directionMarker.setRotation(trackDirection);
                return;
            }
        }
    }

    private Bitmap getDirectionBitmap() {
        Bitmap base = UtilFunctions.getBitmapFromVector(requireContext(), R.drawable.ic_baseline_navigation_24);
        int sizePx = Math.max(18, Math.round(
                DIRECTION_MARKER_SIZE_DP * requireContext().getResources().getDisplayMetrics().density
        ));
        return Bitmap.createScaledBitmap(base, sizePx, sizePx, true);
    }

    private float resolveDisplayHeading(@NonNull LatLng currentDisplayLocation,
                                          float sensorOrientationDegrees) {
        if (isValidOrientation(sensorOrientationDegrees)) {
            float normalizedSensorDegrees = normalizeDegrees(sensorOrientationDegrees);
            if (Float.isNaN(filteredHeadingDegrees)) {
                filteredHeadingDegrees = normalizedSensorDegrees;
            } else {
                filteredHeadingDegrees = blendDegrees(
                        filteredHeadingDegrees,
                        normalizedSensorDegrees,
                        HEADING_SENSOR_SMOOTH_ALPHA
                );
            }
        }

        if (lastHeadingLocation != null) {
            double movementMeters = UtilFunctions.distanceBetweenPoints(
                    lastHeadingLocation,
                    currentDisplayLocation
            );
            if (movementMeters >= HEADING_MOVEMENT_MIN_DISTANCE_METERS) {
                float courseHeading = computeHeadingDegrees(lastHeadingLocation, currentDisplayLocation);
                if (Float.isNaN(filteredHeadingDegrees)) {
                    filteredHeadingDegrees = courseHeading;
                } else {
                    filteredHeadingDegrees = blendDegrees(
                            filteredHeadingDegrees,
                            courseHeading,
                            HEADING_COURSE_BLEND_ALPHA
                    );
                }
                lastHeadingLocation = currentDisplayLocation;
            }
        } else {
            lastHeadingLocation = currentDisplayLocation;
        }

        if (Float.isNaN(filteredHeadingDegrees)) {
            return lastDirectionDegrees;
        }

        lastDirectionDegrees = normalizeDegrees(filteredHeadingDegrees);
        return lastDirectionDegrees;
    }

    private boolean isValidOrientation(float orientationDegrees) {
        return !Float.isNaN(orientationDegrees) && !Float.isInfinite(orientationDegrees);
    }

    private LatLng resolveDisplayLocation(@NonNull LatLng rawLocation) {
        if (smoothedDisplayLocation == null) {
            smoothedDisplayLocation = rawLocation;
            return rawLocation;
        }

        double distanceMeters = UtilFunctions.distanceBetweenPoints(smoothedDisplayLocation, rawLocation);
        if (distanceMeters >= DISPLAY_SNAP_JUMP_METERS) {
            smoothedDisplayLocation = rawLocation;
            return rawLocation;
        }

        double alpha = DISPLAY_SMOOTHING_ALPHA;
        if (distanceMeters > MAX_DISPLAY_STEP_METERS && distanceMeters > 1e-6) {
            alpha = Math.min(alpha, MAX_DISPLAY_STEP_METERS / distanceMeters);
        }

        smoothedDisplayLocation = new LatLng(
                smoothedDisplayLocation.latitude
                        + (rawLocation.latitude - smoothedDisplayLocation.latitude) * alpha,
                smoothedDisplayLocation.longitude
                        + (rawLocation.longitude - smoothedDisplayLocation.longitude) * alpha
        );
        return smoothedDisplayLocation;
    }

    private float blendDegrees(float fromDegrees, float toDegrees, float alpha) {
        float from = normalizeDegrees(fromDegrees);
        float to = normalizeDegrees(toDegrees);
        float delta = to - from;
        if (delta > 180f) {
            delta -= 360f;
        } else if (delta < -180f) {
            delta += 360f;
        }
        return normalizeDegrees(from + alpha * delta);
    }

    private float computeHeadingDegrees(@NonNull LatLng from, @NonNull LatLng to) {
        double deltaNorth = (to.latitude - from.latitude) * 111_111d;
        double deltaEast = (to.longitude - from.longitude)
                * 111_111d
                * Math.cos(Math.toRadians((from.latitude + to.latitude) * 0.5d));
        return normalizeDegrees((float) Math.toDegrees(Math.atan2(deltaEast, deltaNorth)));
    }

    private float normalizeDegrees(float degrees) {
        float value = degrees % 360f;
        if (value < 0f) {
            value += 360f;
        }
        return value;
    }

    private void updateTrackHistory(@NonNull LatLng location) {
        if (polyline == null) {
            return;
        }

        List<LatLng> userTrackHistory = getActiveTrackHistory();

        if (userTrackHistory.isEmpty()) {
            userTrackHistory.add(location);
            refreshDisplayedTrackPolyline();
            return;
        }

        int lastIndex = userTrackHistory.size() - 1;
        LatLng lastPoint = userTrackHistory.get(lastIndex);
        double distanceMeters = UtilFunctions.distanceBetweenPoints(lastPoint, location);

        if (distanceMeters <= TRACK_REPLACE_DISTANCE_METERS) {
            userTrackHistory.set(lastIndex, location);
            refreshDisplayedTrackPolyline();
            return;
        }

        if (distanceMeters <= MAX_TRACK_APPEND_DISTANCE_METERS
                && distanceMeters >= TRACK_APPEND_DISTANCE_METERS) {
            userTrackHistory.add(location);
            while (userTrackHistory.size() > MAX_TRACK_HISTORY_POINTS) {
                userTrackHistory.remove(0);
            }
            refreshDisplayedTrackPolyline();
            return;
        }

        appendInterpolatedTrackPoints(userTrackHistory, lastPoint, location, distanceMeters);
        refreshDisplayedTrackPolyline();
    }

    private void appendInterpolatedTrackPoints(@NonNull List<LatLng> userTrackHistory,
                                               @NonNull LatLng start,
                                               @NonNull LatLng end,
                                               double distanceMeters) {
        int segments = Math.max(1, (int) Math.ceil(distanceMeters / TRACK_SEGMENT_INTERPOLATION_METERS));
        for (int i = 1; i <= segments; i++) {
            double ratio = (double) i / segments;
            LatLng point = new LatLng(
                    start.latitude + (end.latitude - start.latitude) * ratio,
                    start.longitude + (end.longitude - start.longitude) * ratio
            );
            userTrackHistory.add(point);
        }
        while (userTrackHistory.size() > MAX_TRACK_HISTORY_POINTS) {
            userTrackHistory.remove(0);
        }
    }

    private void rebuildTrackHistory(@NonNull List<LatLng> fusedHistory) {
        int historyFloor = resolveHistoryFloor();
        List<LatLng> existingHistory = userTrackHistoryByFloor.get(historyFloor);
        if (userTrackHistoryByFloor.size() == 1
                && activeTrackFloor == historyFloor
                && historiesMatch(existingHistory, fusedHistory)) {
            return;
        }

        userTrackHistoryByFloor.clear();
        userTrackHistoryByFloor.put(historyFloor, new ArrayList<>(fusedHistory));
        activeTrackFloor = historyFloor;
    }

    private int resolveHistoryFloor() {
        if (indoorMapManager != null && indoorMapManager.getIsIndoorMapSet()) {
            return indoorMapManager.getCurrentLogicalFloor();
        }
        if (sensorFusion != null && sensorFusion.isIndoorContextActive()) {
            return sensorFusion.getCurrentLogicalFloor();
        }
        return 0;
    }

    private boolean historiesMatch(@Nullable List<LatLng> first,
                                   @NonNull List<LatLng> second) {
        if (first == null || first.size() != second.size()) {
            return false;
        }
        for (int i = 0; i < second.size(); i++) {
            if (UtilFunctions.distanceBetweenPoints(first.get(i), second.get(i))
                    > HISTORY_POINT_MATCH_TOLERANCE_METERS) {
                return false;
            }
        }
        return true;
    }

    @NonNull
    private List<LatLng> normalizeHistory(@NonNull List<LatLng> fusedHistory) {
        int startIndex = Math.max(0, fusedHistory.size() - MAX_TRACK_HISTORY_POINTS);
        ArrayList<LatLng> normalized = new ArrayList<>(fusedHistory.size() - startIndex);
        for (int i = startIndex; i < fusedHistory.size(); i++) {
            LatLng point = fusedHistory.get(i);
            if (point != null) {
                normalized.add(point);
            }
        }
        return normalized;
    }

    private void clearTrackHistory() {
        userTrackHistoryByFloor.clear();
        activeTrackFloor = Integer.MIN_VALUE;
        if (polyline != null) {
            polyline.setPoints(Collections.emptyList());
        }
    }

    private void refreshDisplayedTrackPolyline() {
        if (polyline == null) {
            return;
        }
        List<LatLng> activeHistory = getActiveTrackHistory();
        List<LatLng> displayPath = indoorMapManager == null
                ? new ArrayList<>(activeHistory)
                : indoorMapManager.buildLegalDisplayPath(activeHistory);
        polyline.setPoints(displayPath);
    }

    private List<LatLng> getActiveTrackHistory() {
        int floorKey = activeTrackFloor != Integer.MIN_VALUE ? activeTrackFloor : 0;
        return userTrackHistoryByFloor.computeIfAbsent(floorKey, ignored -> new ArrayList<>());
    }

    private void syncActiveTrackFloorWithDisplayedFloor() {
        if (indoorMapManager == null || !indoorMapManager.getIsIndoorMapSet()) {
            return;
        }
        int displayedFloor = indoorMapManager.getCurrentLogicalFloor();
        if (displayedFloor == activeTrackFloor) {
            return;
        }
        activeTrackFloor = displayedFloor;
        refreshDisplayedTrackPolyline();
        clearCircles(gnssTailCircles);
        clearCircles(wifiTailCircles);
        clearCircles(pdrTailCircles);
    }

    private float absoluteBearingDelta(float first, float second) {
        float delta = Math.abs(normalizeDegrees(first) - normalizeDegrees(second));
        return delta > 180f ? 360f - delta : delta;
    }

    private void applyOverlayMode() {
        if (mapControlsCard != null) {
            mapControlsCard.setVisibility(previewFloorOnlyMode ? View.GONE : View.VISIBLE);
        }
        if (previewFloorOnlyMode) {
            if (floorUpButton != null) {
                floorUpButton.setVisibility(View.GONE);
            }
            if (floorDownButton != null) {
                floorDownButton.setVisibility(View.GONE);
            }
        }
    }
}
