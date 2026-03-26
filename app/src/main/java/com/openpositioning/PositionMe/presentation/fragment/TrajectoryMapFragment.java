package com.openpositioning.PositionMe.presentation.fragment;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.text.TextUtils;
import android.util.Log;
import android.util.TypedValue;
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
import androidx.lifecycle.ViewModelProvider;

import com.bumptech.glide.Glide;
import com.bumptech.glide.request.target.CustomTarget;
import com.bumptech.glide.request.transition.Transition;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.BitmapDescriptor;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.GroundOverlay;
import com.google.android.gms.maps.model.GroundOverlayOptions;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.LatLngBounds;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.model.Polygon;
import com.google.android.gms.maps.model.PolygonOptions;
import com.google.android.gms.maps.model.Polyline;
import com.google.android.gms.maps.model.PolylineOptions;
import com.google.android.material.floatingactionbutton.FloatingActionButton;
import com.google.android.material.switchmaterial.SwitchMaterial;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.openpositioning.PositionMe.R;
import com.openpositioning.PositionMe.presentation.activity.RecordingActivity;
import com.openpositioning.PositionMe.sensors.FusedPose;
import com.openpositioning.PositionMe.sensors.SensorFusion;
import com.openpositioning.PositionMe.utils.BuildingPolygon;
import com.openpositioning.PositionMe.utils.IndoorMapManager;
import com.openpositioning.PositionMe.utils.UtilFunctions;
import com.openpositioning.PositionMe.viewmodels.MapViewModel;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class TrajectoryMapFragment extends Fragment {

    private static final int MAX_RAW_OBSERVATIONS = 12;
    private static final long FUSED_PATH_UPDATE_INTERVAL_MS = 1000L;
    private static final double FUSED_PATH_MOVEMENT_THRESHOLD_M = 0.75;
    // Suppress visually duplicate trajectory vertices when the displayed position is effectively stationary.
    private static final double TRAJECTORY_DUPLICATE_SUPPRESS_THRESHOLD_M = 0.15;
    private static final double RAW_POINT_DUPLICATE_THRESHOLD_M = 0.25;
    private static final double DISPLAY_SMOOTHING_ALPHA = 0.35;
    private static final double HEADING_MOVEMENT_MIN_DISTANCE_M = 0.8;
    private static final float HEADING_SENSOR_SMOOTH_ALPHA = 0.15f;
    private static final float HEADING_COURSE_BLEND_ALPHA = 0.35f;
    private static final float FUSED_ICON_HEADING_OFFSET_DEG = 0f;
    private static final float DISABLED_ACTION_ALPHA = 0.4f;
    private static final float ENABLED_ACTION_ALPHA = 1f;
    private static final float DEFAULT_WAITING_CAMERA_ZOOM = 17.2f;
    private static final int FUSED_COLOR = Color.parseColor("#D32F2F");
    private static final int GNSS_COLOR = Color.parseColor("#1976D2");
    private static final int WIFI_COLOR = Color.parseColor("#F57C00");
    private static final int PDR_COLOR = Color.parseColor("#00897B");

    private GoogleMap gMap;
    private LatLng currentLocation;
    private LatLng currentRawLocation;
    private LatLng smoothedFusedLocation;
    private Marker fusedMarker;
    private Polyline fusedTrajectoryPolyline;
    private LatLng pendingCameraPosition;
    private boolean hasPendingCameraMove;
    private boolean hasAutoCenteredOnFirstFix;
    private boolean isFusedOn = true;
    private boolean isGnssOn = true;
    private boolean isWifiOn = true;
    private boolean isPdrOn = true;
    private boolean displaySmoothingEnabled = true;
    private boolean venueSelectionEnabled;

    private final List<LatLng> fusedTrajectoryRawPoints = new ArrayList<>();
    private final ArrayDeque<Marker> gnssObservationMarkers = new ArrayDeque<>();
    private final ArrayDeque<Marker> wifiObservationMarkers = new ArrayDeque<>();
    private final ArrayDeque<Marker> pdrObservationMarkers = new ArrayDeque<>();
    private final List<Marker> testPointMarkers = new ArrayList<>();
    private final Map<Integer, BitmapDescriptor> testPointIconCache = new HashMap<>();

    private LatLng lastFusedTrajectoryRawLocation;
    private LatLng lastGnssObservation;
    private LatLng lastWifiObservation;
    private LatLng lastPdrObservation;
    private LatLng lastHeadingLocation;
    private float filteredHeadingDeg = Float.NaN;
    private long lastFusedTrajectoryUpdateTimestampMs = -1L;
    private boolean fusedTrajectoryDrawingEnabled = true;

    private BitmapDescriptor fusedMarkerIcon;
    private BitmapDescriptor gnssObservationIcon;
    private BitmapDescriptor wifiObservationIcon;
    private BitmapDescriptor pdrObservationIcon;

    private SensorFusion sensorFusion;
    private IndoorMapManager indoorMapManager;
    private Spinner switchMapSpinner;
    private SwitchMaterial fusedSwitch;
    private SwitchMaterial gnssSwitch;
    private SwitchMaterial wifiSwitch;
    private SwitchMaterial pdrSwitch;
    private SwitchMaterial displaySmoothingSwitch;
    private SwitchMaterial autoFloorSwitch;
    private FloatingActionButton floorUpButton;
    private FloatingActionButton floorDownButton;
    private FloatingActionButton recenterButton;
    private View mapEmptyStateCard;
    private TextView selectedVenueText;
    private TextView mapEmptyStateText;
    private MapViewModel mapViewModel;
    private final List<Polygon> venuePolygons = new ArrayList<>();
    private GroundOverlay floorplanOverlay;
    private OnMapReadyCallback mapReadyCallback;

    public TrajectoryMapFragment() {
    }

    @Nullable
    @Override
    public View onCreateView(@NonNull LayoutInflater inflater,
                             @Nullable ViewGroup container,
                             @Nullable Bundle savedInstanceState) {
        return inflater.inflate(R.layout.fragment_trajectory_map, container, false);
    }

    @Override
    public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);
        sensorFusion = SensorFusion.getInstance();
        venueSelectionEnabled = requireActivity() instanceof RecordingActivity;

        mapViewModel = new ViewModelProvider(requireParentFragment()).get(MapViewModel.class);
        mapViewModel.getFloorplanResponse().observe(getViewLifecycleOwner(), response -> {
            if (response != null && gMap != null) {
                drawVenueOutlines(response);
            } else {
                Log.d("TrajectoryMapFragment", "Failed to fetch floorplans or map not ready.");
            }
        });

        switchMapSpinner = view.findViewById(R.id.mapSwitchSpinner);
        fusedSwitch = view.findViewById(R.id.fusedSwitch);
        gnssSwitch = view.findViewById(R.id.gnssSwitch);
        wifiSwitch = view.findViewById(R.id.wifiSwitch);
        pdrSwitch = view.findViewById(R.id.pdrSwitch);
        displaySmoothingSwitch = view.findViewById(R.id.displaySmoothingSwitch);
        autoFloorSwitch = view.findViewById(R.id.autoFloor);
        floorUpButton = view.findViewById(R.id.floorUpButton);
        floorDownButton = view.findViewById(R.id.floorDownButton);
        recenterButton = view.findViewById(R.id.recenterButton);
        mapEmptyStateCard = view.findViewById(R.id.mapEmptyStateCard);
        selectedVenueText = view.findViewById(R.id.selectedVenueText);
        mapEmptyStateText = view.findViewById(R.id.mapEmptyStateText);

        isFusedOn = fusedSwitch.isChecked();
        isGnssOn = gnssSwitch.isChecked();
        isWifiOn = wifiSwitch.isChecked();
        isPdrOn = pdrSwitch.isChecked();
        displaySmoothingEnabled = displaySmoothingSwitch.isChecked();

        setFloorControlsVisibility(View.GONE);
        if (!venueSelectionEnabled && selectedVenueText != null) {
            selectedVenueText.setVisibility(View.GONE);
        }

        SupportMapFragment mapFragment =
                (SupportMapFragment) getChildFragmentManager().findFragmentById(R.id.trajectoryMap);
        if (mapFragment != null) {
            mapFragment.getMapAsync(googleMap -> {
                gMap = googleMap;
                initMapSettings(gMap);
                if (hasPendingCameraMove && pendingCameraPosition != null) {
                    gMap.moveCamera(CameraUpdateFactory.newLatLngZoom(pendingCameraPosition, 19f));
                    hasPendingCameraMove = false;
                    pendingCameraPosition = null;
                } else {
                    moveCameraToDefaultWaitingAreaIfNeeded();
                }
                if (venueSelectionEnabled) {
                    gMap.setOnPolygonClickListener(polygon -> {
                        if (indoorMapManager != null && indoorMapManager.onVenuePolygonClicked(polygon)) {
                            updateVenueLabel();
                            setFloorControlsVisibility(
                                    indoorMapManager.getIsIndoorMapSet() ? View.VISIBLE : View.GONE
                            );
                        }
                    });
                }
                if (mapReadyCallback != null) {
                    mapReadyCallback.onMapReady(gMap);
                }
                updateWaitingStateUi();
            });
        }

        initMapTypeSpinner();

        fusedSwitch.setOnCheckedChangeListener((buttonView, isChecked) -> {
            isFusedOn = isChecked;
            updateLayerVisibility();
        });
        gnssSwitch.setOnCheckedChangeListener((buttonView, isChecked) -> {
            isGnssOn = isChecked;
            updateLayerVisibility();
        });
        wifiSwitch.setOnCheckedChangeListener((buttonView, isChecked) -> {
            isWifiOn = isChecked;
            updateLayerVisibility();
        });
        pdrSwitch.setOnCheckedChangeListener((buttonView, isChecked) -> {
            isPdrOn = isChecked;
            updateLayerVisibility();
        });
        displaySmoothingSwitch.setOnCheckedChangeListener((buttonView, isChecked) -> {
            displaySmoothingEnabled = isChecked;
            smoothedFusedLocation = currentRawLocation;
            refreshFusedTrajectoryPolyline();
            if (currentRawLocation != null) {
                LatLng displayedLocation = resolveDisplayLocation(currentRawLocation);
                currentLocation = displayedLocation;
                updateFusedMarker(
                        displayedLocation,
                        fusedMarker != null ? fusedMarker.getRotation() : 0f
                );
            }
        });

        autoFloorSwitch.setOnCheckedChangeListener((compoundButton, isChecked) -> {
            if (!isChecked || indoorMapManager == null || !indoorMapManager.getIsIndoorMapSet()) {
                return;
            }
            float venueFloorHeight = indoorMapManager.getFloorHeight();
            if (venueFloorHeight > 0f) {
                sensorFusion.setPdrFloorHeightMeters(venueFloorHeight);
            }
            // Floor switch is handled by RecordingFragment with 2s stability debounce.
        });

        floorUpButton.setOnClickListener(v -> {
            autoFloorSwitch.setChecked(false);
            if (indoorMapManager != null) {
                indoorMapManager.increaseFloor();
                updateVenueLabel();
            }
        });

        floorDownButton.setOnClickListener(v -> {
            autoFloorSwitch.setChecked(false);
            if (indoorMapManager != null) {
                indoorMapManager.decreaseFloor();
                updateVenueLabel();
            }
        });

        recenterButton.setOnClickListener(v -> recenterOnCurrentLocation());
        updateWaitingStateUi();
    }

    private void initMapSettings(@NonNull GoogleMap map) {
        map.getUiSettings().setCompassEnabled(true);
        map.getUiSettings().setTiltGesturesEnabled(true);
        map.getUiSettings().setRotateGesturesEnabled(true);
        map.getUiSettings().setScrollGesturesEnabled(true);
        map.setMapType(GoogleMap.MAP_TYPE_HYBRID);

        indoorMapManager = new IndoorMapManager(requireContext(), map);
        indoorMapManager.setAutoSelectFirstVenue(!venueSelectionEnabled);
        indoorMapManager.setVenueSelectionListener((venueId, venueName) -> {
            if (venueSelectionEnabled) {
                sensorFusion.setCollectionVenue(venueId);
                sensorFusion.setVenueIdForTrajectory(venueId);
                if (mapViewModel != null) {
                    mapViewModel.setSelectedVenueId(venueId);
                }
            }
            updateVenueLabel();
            setFloorControlsVisibility(indoorMapManager.getIsIndoorMapSet() ? View.VISIBLE : View.GONE);
        });

        fusedTrajectoryPolyline = map.addPolyline(new PolylineOptions()
                .color(FUSED_COLOR)
                .width(7f)
                .zIndex(2f));
        updateVenueLabel();
        updateLayerVisibility();
    }

    private void initMapTypeSpinner() {
        if (switchMapSpinner == null) {
            return;
        }
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
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                if (gMap == null) {
                    return;
                }
                if (position == 0) {
                    gMap.setMapType(GoogleMap.MAP_TYPE_HYBRID);
                } else if (position == 1) {
                    gMap.setMapType(GoogleMap.MAP_TYPE_NORMAL);
                } else {
                    gMap.setMapType(GoogleMap.MAP_TYPE_SATELLITE);
                }
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
            }
        });
    }

    public void updateUserLocation(@NonNull LatLng newLocation, float orientation) {
        updateUserLocation(newLocation, orientation, System.currentTimeMillis());
    }

    public void updateUserLocation(@NonNull LatLng newLocation, float orientation, long timestampMs) {
        if (gMap == null) {
            return;
        }

        currentRawLocation = newLocation;
        LatLng displayedLocation = resolveDisplayLocation(newLocation);
        currentLocation = displayedLocation;

        float resolvedHeadingDeg = resolveDisplayHeading(displayedLocation, orientation);
        updateFusedMarker(displayedLocation, resolvedHeadingDeg);
        maybeAppendFusedTrajectory(newLocation, timestampMs);

        if (!hasAutoCenteredOnFirstFix) {
            gMap.moveCamera(CameraUpdateFactory.newLatLngZoom(displayedLocation, 19f));
            hasAutoCenteredOnFirstFix = true;
        }
        updateWaitingStateUi();

        if (indoorMapManager != null) {
            indoorMapManager.setCurrentLocation(displayedLocation);
            indoorMapManager.refreshNearbyVenues(displayedLocation, sensorFusion.getWifiList());
            setFloorControlsVisibility(indoorMapManager.getIsIndoorMapSet() ? View.VISIBLE : View.GONE);
            updateVenueLabel();
        }
    }

    public void updateGNSS(@NonNull LatLng gnssLocation) {
        upsertLatestObservationMarker(
                gnssObservationMarkers,
                gnssLocation,
                "GNSS Position",
                null,
                getGnssObservationIcon(),
                isGnssOn,
                1f
        );
        lastGnssObservation = gnssLocation;
    }

    public void updateWifiFix(@NonNull LatLng wifiLocation, int floor) {
        upsertLatestObservationMarker(
                wifiObservationMarkers,
                wifiLocation,
                "WiFi Position",
                "Floor " + floor,
                getWifiObservationIcon(),
                isWifiOn,
                1.2f
        );
        lastWifiObservation = wifiLocation;
    }

    public void updatePdrObservation(@NonNull LatLng pdrLocation) {
        addObservationMarker(
                pdrObservationMarkers,
                lastPdrObservation,
                pdrLocation,
                "PDR Position",
                null,
                getPdrObservationIcon(),
                isPdrOn,
                1.15f
        );
        lastPdrObservation = pdrLocation;
    }

    public void clearGNSS() {
        clearObservationMarkers(gnssObservationMarkers);
        lastGnssObservation = null;
    }

    public void clearWifiFix() {
        clearObservationMarkers(wifiObservationMarkers);
        lastWifiObservation = null;
    }

    public void clearPdrObservations() {
        clearObservationMarkers(pdrObservationMarkers);
        lastPdrObservation = null;
    }

    public boolean isGnssEnabled() {
        return isGnssOn;
    }

    public boolean isAutoFloorEnabled() {
        return autoFloorSwitch != null && autoFloorSwitch.isChecked();
    }

    public boolean isMappedVenueActive() {
        return indoorMapManager != null
                && indoorMapManager.getIsIndoorMapSet()
                && !TextUtils.isEmpty(indoorMapManager.getSelectedVenueName());
    }

    public float getVenueFloorHeightMeters() {
        if (indoorMapManager == null || !indoorMapManager.getIsIndoorMapSet()) {
            return 0f;
        }
        return indoorMapManager.getFloorHeight();
    }

    @Nullable
    public String getCurrentDisplayedFloorLabel() {
        if (indoorMapManager == null || !indoorMapManager.getIsIndoorMapSet()) {
            return null;
        }
        return indoorMapManager.getCurrentFloorLabel();
    }

    @Nullable
    public Integer getCurrentDisplayedFloorSemanticLevel() {
        if (indoorMapManager == null || !indoorMapManager.getIsIndoorMapSet()) {
            return null;
        }
        return indoorMapManager.getCurrentFloorSemanticLevel();
    }

    public void syncDisplayedFloor(int floor) {
        if (indoorMapManager == null || !indoorMapManager.getIsIndoorMapSet()) {
            return;
        }
        // RecordingFragment already applies debounce; avoid a second debounce layer here.
        indoorMapManager.setCurrentFloorFromSemanticLevel(floor, false);
        updateVenueLabel();
    }

    @Nullable
    public String getFloorDisplayLabelFor(int floorIndex) {
        if (indoorMapManager == null || !indoorMapManager.getIsIndoorMapSet()) {
            return null;
        }
        return indoorMapManager.getFloorLabelForIndex(floorIndex);
    }

    public void setInitialCameraPosition(@NonNull LatLng startLocation) {
        if (gMap != null) {
            gMap.moveCamera(CameraUpdateFactory.newLatLngZoom(startLocation, 19f));
            hasAutoCenteredOnFirstFix = true;
        } else {
            pendingCameraPosition = startLocation;
            hasPendingCameraMove = true;
        }
    }

    public LatLng getCurrentLocation() {
        return currentLocation;
    }

    public void clearMapAndReset() {
        fusedTrajectoryRawPoints.clear();
        lastFusedTrajectoryRawLocation = null;
        lastFusedTrajectoryUpdateTimestampMs = -1L;
        currentLocation = null;
        currentRawLocation = null;
        smoothedFusedLocation = null;
        lastHeadingLocation = null;
        filteredHeadingDeg = Float.NaN;
        hasAutoCenteredOnFirstFix = false;

        clearGNSS();
        clearWifiFix();
        clearPdrObservations();

        if (fusedMarker != null) {
            fusedMarker.remove();
            fusedMarker = null;
        }
        if (fusedTrajectoryPolyline != null) {
            fusedTrajectoryPolyline.remove();
            fusedTrajectoryPolyline = null;
        }
        if (gMap != null) {
            fusedTrajectoryPolyline = gMap.addPolyline(new PolylineOptions()
                    .color(FUSED_COLOR)
                    .width(7f)
                    .zIndex(2f));
        }
        updateLayerVisibility();
        updateWaitingStateUi();
    }

    public void setFusedTrajectoryDrawingEnabled(boolean enabled, boolean resetExistingPath) {
        fusedTrajectoryDrawingEnabled = enabled;
        if (resetExistingPath) {
            fusedTrajectoryRawPoints.clear();
            lastFusedTrajectoryRawLocation = null;
            lastFusedTrajectoryUpdateTimestampMs = -1L;
            refreshFusedTrajectoryPolyline();
        }
    }

    public void getMapAsync(OnMapReadyCallback callback) {
        if (gMap != null) {
            callback.onMapReady(gMap);
        } else {
            this.mapReadyCallback = callback;
        }
    }

    public GoogleMap getMap() {
        return gMap;
    }

    public void addTestPointMarker(@NonNull LatLng pos, int idx) {
        if (gMap == null) {
            return;
        }
        Marker marker = gMap.addMarker(new MarkerOptions()
                .position(pos)
                .title("TP " + idx)
                .anchor(0.5f, 0.5f)
                .icon(getNumberedTestPointIcon(idx)));
        if (marker != null) {
            testPointMarkers.add(marker);
        }
    }

    public void clearTestPointMarkers() {
        for (Marker marker : testPointMarkers) {
            if (marker != null) {
                marker.remove();
            }
        }
        testPointMarkers.clear();
    }

    private void updateFusedMarker(@NonNull LatLng displayedLocation, float orientation) {
        if (gMap == null) {
            return;
        }
        if (fusedMarker == null) {
            fusedMarker = gMap.addMarker(new MarkerOptions()
                    .position(displayedLocation)
                    .anchor(0.5f, 0.5f)
                    .flat(true)
                    .title("Fused Position")
                    .zIndex(4f)
                    .icon(getFusedMarkerIcon()));
        } else {
            fusedMarker.setPosition(displayedLocation);
        }
        if (fusedMarker != null) {
            fusedMarker.setRotation(normalizeDegrees(orientation + FUSED_ICON_HEADING_OFFSET_DEG));
            fusedMarker.setVisible(isFusedOn);
        }
    }

    private float resolveDisplayHeading(@NonNull LatLng current, float sensorHeadingDeg) {
        float normalizedSensorDeg = normalizeDegrees(sensorHeadingDeg);
        if (Float.isNaN(filteredHeadingDeg)) {
            filteredHeadingDeg = normalizedSensorDeg;
            lastHeadingLocation = current;
            return filteredHeadingDeg;
        }

        // Smooth raw sensor heading first to reduce jitter.
        filteredHeadingDeg = blendDegrees(filteredHeadingDeg, normalizedSensorDeg, HEADING_SENSOR_SMOOTH_ALPHA);

        if (lastHeadingLocation != null) {
            double movementMeters = UtilFunctions.distanceBetweenPoints(lastHeadingLocation, current);
            if (movementMeters >= HEADING_MOVEMENT_MIN_DISTANCE_M) {
                float courseHeadingDeg = bearingDegrees(lastHeadingLocation, current);
                // When moving, blend in course heading so arrow follows actual travel direction.
                filteredHeadingDeg = blendDegrees(filteredHeadingDeg, courseHeadingDeg, HEADING_COURSE_BLEND_ALPHA);
                lastHeadingLocation = current;
            }
        } else {
            lastHeadingLocation = current;
        }
        return normalizeDegrees(filteredHeadingDeg);
    }

    private float bearingDegrees(@NonNull LatLng from, @NonNull LatLng to) {
        double fromLat = Math.toRadians(from.latitude);
        double toLat = Math.toRadians(to.latitude);
        double deltaLon = Math.toRadians(to.longitude - from.longitude);
        double y = Math.sin(deltaLon) * Math.cos(toLat);
        double x = Math.cos(fromLat) * Math.sin(toLat)
                - Math.sin(fromLat) * Math.cos(toLat) * Math.cos(deltaLon);
        return normalizeDegrees((float) Math.toDegrees(Math.atan2(y, x)));
    }

    private float blendDegrees(float baseDeg, float targetDeg, float alpha) {
        float delta = shortestSignedDeltaDeg(baseDeg, targetDeg);
        return normalizeDegrees(baseDeg + alpha * delta);
    }

    private float shortestSignedDeltaDeg(float fromDeg, float toDeg) {
        float delta = normalizeDegrees(toDeg) - normalizeDegrees(fromDeg);
        if (delta > 180f) delta -= 360f;
        if (delta < -180f) delta += 360f;
        return delta;
    }

    private float normalizeDegrees(float deg) {
        float normalized = deg % 360f;
        if (normalized < 0f) {
            normalized += 360f;
        }
        return normalized;
    }

    private void maybeAppendFusedTrajectory(@NonNull LatLng rawLocation, long timestampMs) {
        if (!fusedTrajectoryDrawingEnabled) {
            return;
        }
        if (fusedTrajectoryRawPoints.isEmpty()) {
            fusedTrajectoryRawPoints.add(rawLocation);
            lastFusedTrajectoryRawLocation = rawLocation;
            lastFusedTrajectoryUpdateTimestampMs = timestampMs;
            refreshFusedTrajectoryPolyline();
            return;
        }

        double distanceMeters = lastFusedTrajectoryRawLocation == null
                ? 0.0
                : UtilFunctions.distanceBetweenPoints(lastFusedTrajectoryRawLocation, rawLocation);
        long elapsedMs = lastFusedTrajectoryUpdateTimestampMs <= 0L
                ? Long.MAX_VALUE
                : Math.max(0L, timestampMs - lastFusedTrajectoryUpdateTimestampMs);

        if (distanceMeters < FUSED_PATH_MOVEMENT_THRESHOLD_M
                && elapsedMs < FUSED_PATH_UPDATE_INTERVAL_MS) {
            return;
        }
        if (distanceMeters < TRAJECTORY_DUPLICATE_SUPPRESS_THRESHOLD_M) {
            return;
        }

        fusedTrajectoryRawPoints.add(rawLocation);
        lastFusedTrajectoryRawLocation = rawLocation;
        lastFusedTrajectoryUpdateTimestampMs = timestampMs;
        refreshFusedTrajectoryPolyline();
    }

    private void refreshFusedTrajectoryPolyline() {
        if (fusedTrajectoryPolyline == null) {
            return;
        }
        List<LatLng> displayedPoints = new ArrayList<>(fusedTrajectoryRawPoints.size());
        if (displaySmoothingEnabled) {
            LatLng smoothedPoint = null;
            for (LatLng rawPoint : fusedTrajectoryRawPoints) {
                smoothedPoint = smoothedPoint == null
                        ? rawPoint
                        : interpolate(smoothedPoint, rawPoint, DISPLAY_SMOOTHING_ALPHA);
                displayedPoints.add(smoothedPoint);
            }
        } else {
            displayedPoints.addAll(fusedTrajectoryRawPoints);
        }
        fusedTrajectoryPolyline.setPoints(displayedPoints);
        fusedTrajectoryPolyline.setVisible(isFusedOn);
    }

    @NonNull
    private LatLng resolveDisplayLocation(@NonNull LatLng rawLocation) {
        if (!displaySmoothingEnabled) {
            smoothedFusedLocation = rawLocation;
            return rawLocation;
        }
        if (smoothedFusedLocation == null) {
            smoothedFusedLocation = rawLocation;
            return rawLocation;
        }
        smoothedFusedLocation = interpolate(smoothedFusedLocation, rawLocation, DISPLAY_SMOOTHING_ALPHA);
        return smoothedFusedLocation;
    }

    @NonNull
    private LatLng interpolate(@NonNull LatLng from, @NonNull LatLng to, double alpha) {
        double clampedAlpha = Math.max(0.0, Math.min(1.0, alpha));
        return new LatLng(
                from.latitude + ((to.latitude - from.latitude) * clampedAlpha),
                from.longitude + ((to.longitude - from.longitude) * clampedAlpha)
        );
    }

    private void addObservationMarker(
            @NonNull ArrayDeque<Marker> markers,
            @Nullable LatLng previousLocation,
            @NonNull LatLng newLocation,
            @NonNull String title,
            @Nullable String snippet,
            @NonNull BitmapDescriptor icon,
            boolean visible,
            float zIndex
    ) {
        if (gMap == null) {
            return;
        }
        if (previousLocation != null
                && UtilFunctions.distanceBetweenPoints(previousLocation, newLocation) < RAW_POINT_DUPLICATE_THRESHOLD_M) {
            return;
        }
        Marker marker = gMap.addMarker(new MarkerOptions()
                .position(newLocation)
                .title(title)
                .snippet(snippet)
                .anchor(0.5f, 0.5f)
                .zIndex(zIndex)
                .icon(icon)
                .visible(visible));
        if (marker == null) {
            return;
        }
        markers.addLast(marker);
        while (markers.size() > MAX_RAW_OBSERVATIONS) {
            Marker oldest = markers.removeFirst();
            oldest.remove();
        }
    }

    private void upsertLatestObservationMarker(
            @NonNull ArrayDeque<Marker> markers,
            @NonNull LatLng newLocation,
            @NonNull String title,
            @Nullable String snippet,
            @NonNull BitmapDescriptor icon,
            boolean visible,
            float zIndex
    ) {
        if (gMap == null) {
            return;
        }
        Marker marker = markers.peekLast();
        if (marker != null) {
            marker.setPosition(newLocation);
            marker.setTitle(title);
            marker.setSnippet(snippet);
            marker.setVisible(visible);
            marker.setZIndex(zIndex);
            return;
        }
        Marker created = gMap.addMarker(new MarkerOptions()
                .position(newLocation)
                .title(title)
                .snippet(snippet)
                .anchor(0.5f, 0.5f)
                .zIndex(zIndex)
                .icon(icon)
                .visible(visible));
        if (created != null) {
            markers.addLast(created);
        }
    }

    private void clearObservationMarkers(@NonNull ArrayDeque<Marker> markers) {
        while (!markers.isEmpty()) {
            Marker marker = markers.removeFirst();
            marker.remove();
        }
    }

    private void recenterOnCurrentLocation() {
        if (gMap == null || currentLocation == null) {
            return;
        }
        gMap.animateCamera(CameraUpdateFactory.newLatLngZoom(currentLocation, 19f));
    }

    private void updateWaitingStateUi() {
        boolean hasDisplayPose = currentLocation != null;
        if (mapEmptyStateCard != null) {
            boolean showEmptyState = venueSelectionEnabled && !hasDisplayPose;
            mapEmptyStateCard.setVisibility(showEmptyState ? View.VISIBLE : View.GONE);
        }
        if (mapEmptyStateText != null) {
            mapEmptyStateText.setText(getString(R.string.map_waiting_state));
        }
        updateRecenterState(hasDisplayPose);
    }

    private void updateRecenterState(boolean enabled) {
        if (recenterButton == null) {
            return;
        }
        recenterButton.setEnabled(enabled);
        recenterButton.setClickable(enabled);
        recenterButton.setAlpha(enabled ? ENABLED_ACTION_ALPHA : DISABLED_ACTION_ALPHA);
    }

    private void moveCameraToDefaultWaitingAreaIfNeeded() {
        if (gMap == null || !venueSelectionEnabled || currentLocation != null || hasAutoCenteredOnFirstFix) {
            return;
        }

        LatLng anchor = sensorFusion.getCurrentGnssLatLng();
        if (anchor == null) {
            float[] startLocation = sensorFusion.getGNSSLatitude(true);
            if (startLocation != null
                    && startLocation.length >= 2
                    && !(startLocation[0] == 0f && startLocation[1] == 0f)) {
                anchor = new LatLng(startLocation[0], startLocation[1]);
            }
        }
        if (anchor == null) {
            anchor = getDefaultWaitingCameraCenter();
        }
        gMap.moveCamera(CameraUpdateFactory.newLatLngZoom(anchor, DEFAULT_WAITING_CAMERA_ZOOM));
    }

    @NonNull
    private LatLng getDefaultWaitingCameraCenter() {
        double north = Math.max(BuildingPolygon.NUCLEUS_NE.latitude, BuildingPolygon.LIBRARY_NE.latitude);
        double south = Math.min(BuildingPolygon.NUCLEUS_SW.latitude, BuildingPolygon.LIBRARY_SW.latitude);
        double east = Math.max(BuildingPolygon.NUCLEUS_NE.longitude, BuildingPolygon.LIBRARY_NE.longitude);
        double west = Math.min(BuildingPolygon.NUCLEUS_SW.longitude, BuildingPolygon.LIBRARY_SW.longitude);
        return new LatLng((north + south) * 0.5, (east + west) * 0.5);
    }

    private void updateLayerVisibility() {
        if (fusedMarker != null) {
            fusedMarker.setVisible(isFusedOn);
        }
        if (fusedTrajectoryPolyline != null) {
            fusedTrajectoryPolyline.setVisible(isFusedOn);
        }
        setObservationVisibility(gnssObservationMarkers, isGnssOn);
        setObservationVisibility(wifiObservationMarkers, isWifiOn);
        setObservationVisibility(pdrObservationMarkers, isPdrOn);
    }

    private void setObservationVisibility(@NonNull ArrayDeque<Marker> markers, boolean visible) {
        for (Marker marker : markers) {
            marker.setVisible(visible);
        }
    }

    private void updateVenueLabel() {
        if (selectedVenueText == null || !venueSelectionEnabled) {
            return;
        }
        if (indoorMapManager == null || TextUtils.isEmpty(indoorMapManager.getSelectedVenueName())) {
            selectedVenueText.setText(getString(R.string.venue_not_selected));
            return;
        }
        String venueName = indoorMapManager.getSelectedVenueName();
        int floorCount = indoorMapManager.getFloorCount();
        if (floorCount <= 0) {
            selectedVenueText.setText(getString(R.string.venue_selected_no_floor, venueName));
            return;
        }
        int floorNumber = indoorMapManager.getCurrentFloor() + 1;
        selectedVenueText.setText(getString(R.string.venue_selected_format, venueName, floorNumber, floorCount));
    }

    private void setFloorControlsVisibility(int visibility) {
        int finalVisibility = venueSelectionEnabled ? visibility : View.GONE;
        floorUpButton.setVisibility(finalVisibility);
        floorDownButton.setVisibility(finalVisibility);
        autoFloorSwitch.setVisibility(finalVisibility);
    }

    private BitmapDescriptor getFusedMarkerIcon() {
        if (fusedMarkerIcon != null) {
            return fusedMarkerIcon;
        }
        int sizePx = dpToPx(54);
        int arrowSizePx = dpToPx(24);
        Bitmap bitmap = Bitmap.createBitmap(sizePx, sizePx, Bitmap.Config.ARGB_8888);
        Canvas canvas = new Canvas(bitmap);

        Paint fillPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        fillPaint.setColor(Color.WHITE);
        canvas.drawCircle(sizePx / 2f, sizePx / 2f, (sizePx / 2f) - dpToPx(2), fillPaint);

        Paint strokePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        strokePaint.setColor(FUSED_COLOR);
        strokePaint.setStyle(Paint.Style.STROKE);
        strokePaint.setStrokeWidth(dpToPx(3));
        canvas.drawCircle(sizePx / 2f, sizePx / 2f, (sizePx / 2f) - dpToPx(2), strokePaint);

        Bitmap arrowBitmap = UtilFunctions.getBitmapFromVector(requireContext(), R.drawable.ic_baseline_navigation_24);
        Bitmap scaledArrowBitmap = Bitmap.createScaledBitmap(arrowBitmap, arrowSizePx, arrowSizePx, true);
        float left = (sizePx - arrowSizePx) / 2f;
        float top = (sizePx - arrowSizePx) / 2f;
        canvas.drawBitmap(scaledArrowBitmap, left, top, null);

        fusedMarkerIcon = BitmapDescriptorFactory.fromBitmap(bitmap);
        return fusedMarkerIcon;
    }

    private BitmapDescriptor getGnssObservationIcon() {
        if (gnssObservationIcon == null) {
            gnssObservationIcon = createObservationIcon(GNSS_COLOR, 12, 1);
        }
        return gnssObservationIcon;
    }

    private BitmapDescriptor getWifiObservationIcon() {
        if (wifiObservationIcon == null) {
            wifiObservationIcon = createObservationIcon(WIFI_COLOR, 15, 2);
        }
        return wifiObservationIcon;
    }

    private BitmapDescriptor getPdrObservationIcon() {
        if (pdrObservationIcon == null) {
            pdrObservationIcon = createObservationIcon(PDR_COLOR, 15, 2);
        }
        return pdrObservationIcon;
    }

    private BitmapDescriptor createObservationIcon(int color, int sizeDp, int strokeWidthDp) {
        int sizePx = dpToPx(sizeDp);
        Bitmap bitmap = Bitmap.createBitmap(sizePx, sizePx, Bitmap.Config.ARGB_8888);
        Canvas canvas = new Canvas(bitmap);

        Paint fillPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        fillPaint.setColor(color);
        canvas.drawCircle(sizePx / 2f, sizePx / 2f, (sizePx / 2f) - 1f, fillPaint);

        Paint strokePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        strokePaint.setColor(Color.WHITE);
        strokePaint.setStyle(Paint.Style.STROKE);
        strokePaint.setStrokeWidth(dpToPx(strokeWidthDp));
        canvas.drawCircle(sizePx / 2f, sizePx / 2f, (sizePx / 2f) - 1f, strokePaint);

        return BitmapDescriptorFactory.fromBitmap(bitmap);
    }

    private int dpToPx(int dp) {
        return (int) TypedValue.applyDimension(
                TypedValue.COMPLEX_UNIT_DIP,
                dp,
                getResources().getDisplayMetrics()
        );
    }

    private BitmapDescriptor getNumberedTestPointIcon(int idx) {
        BitmapDescriptor cached = testPointIconCache.get(idx);
        if (cached != null) {
            return cached;
        }

        int sizePx = dpToPx(36);
        Bitmap bitmap = Bitmap.createBitmap(sizePx, sizePx, Bitmap.Config.ARGB_8888);
        Canvas canvas = new Canvas(bitmap);

        Paint fillPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        fillPaint.setColor(Color.parseColor("#D9485F"));
        canvas.drawCircle(sizePx / 2f, sizePx / 2f, (sizePx / 2f) - 2f, fillPaint);

        Paint strokePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        strokePaint.setColor(Color.WHITE);
        strokePaint.setStyle(Paint.Style.STROKE);
        strokePaint.setStrokeWidth(3f);
        canvas.drawCircle(sizePx / 2f, sizePx / 2f, (sizePx / 2f) - 2f, strokePaint);

        Paint textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        textPaint.setColor(Color.WHITE);
        textPaint.setTextAlign(Paint.Align.CENTER);
        textPaint.setFakeBoldText(true);
        textPaint.setTextSize(idx >= 10 ? sizePx * 0.34f : sizePx * 0.42f);
        Paint.FontMetrics fontMetrics = textPaint.getFontMetrics();
        float textY = (sizePx / 2f) - ((fontMetrics.ascent + fontMetrics.descent) / 2f);
        canvas.drawText(String.valueOf(idx), sizePx / 2f, textY, textPaint);

        BitmapDescriptor icon = BitmapDescriptorFactory.fromBitmap(bitmap);
        testPointIconCache.put(idx, icon);
        return icon;
    }

    private void drawVenueOutlines(JsonObject apiResponse) {
        for (Polygon polygon : venuePolygons) {
            polygon.remove();
        }
        venuePolygons.clear();

        JsonArray venues = apiResponse.getAsJsonArray("venues");
        if (venues == null || gMap == null) {
            return;
        }

        for (JsonElement venueElement : venues) {
            JsonObject venue = venueElement.getAsJsonObject();
            JsonArray outlineCoords = venue.getAsJsonObject("outline")
                    .getAsJsonArray("coordinates")
                    .get(0)
                    .getAsJsonArray();

            PolygonOptions polygonOptions = new PolygonOptions()
                    .strokeColor(Color.BLUE)
                    .strokeWidth(5)
                    .fillColor(Color.argb(50, 0, 0, 255))
                    .clickable(true);

            for (JsonElement coordElement : outlineCoords) {
                JsonArray lngLat = coordElement.getAsJsonArray();
                polygonOptions.add(new LatLng(lngLat.get(1).getAsDouble(), lngLat.get(0).getAsDouble()));
            }

            Polygon polygon = gMap.addPolygon(polygonOptions);
            polygon.setTag(venue);
            venuePolygons.add(polygon);
        }
    }

    private void selectVenue(JsonObject venueData) {
        String venueId = venueData.get("id").getAsString();
        Log.d("TrajectoryMapFragment", "Venue selected: " + venueId);
        mapViewModel.setSelectedVenueId(venueId);

        JsonArray floorplans = venueData.getAsJsonArray("floorplans");
        if (floorplans != null && floorplans.size() > 0) {
            JsonObject firstFloor = floorplans.get(0).getAsJsonObject();
            displayFloorplan(firstFloor);
        }
    }

    private void displayFloorplan(JsonObject floorplan) {
        if (floorplanOverlay != null) {
            floorplanOverlay.remove();
        }

        String imageUrl = floorplan.get("url").getAsString();
        JsonArray bbox = floorplan.getAsJsonArray("bbox");
        LatLngBounds bounds = new LatLngBounds(
                new LatLng(bbox.get(1).getAsDouble(), bbox.get(0).getAsDouble()),
                new LatLng(bbox.get(3).getAsDouble(), bbox.get(2).getAsDouble())
        );

        Glide.with(this)
                .asBitmap()
                .load(imageUrl)
                .into(new CustomTarget<Bitmap>() {
                    @Override
                    public void onResourceReady(@NonNull Bitmap resource, @Nullable Transition<? super Bitmap> transition) {
                        if (gMap != null) {
                            floorplanOverlay = gMap.addGroundOverlay(new GroundOverlayOptions()
                                    .image(BitmapDescriptorFactory.fromBitmap(resource))
                                    .positionFromBounds(bounds));
                        }
                    }

                    @Override
                    public void onLoadCleared(@Nullable Drawable placeholder) {
                    }
                });
    }

    public void handlePolygonClick(Polygon polygon) {
        JsonObject venueData = (JsonObject) polygon.getTag();
        if (venueData != null) {
            selectVenue(venueData);
        } else {
            Log.w("TrajectoryMapFragment", "Clicked a polygon with no venue data attached.");
        }
    }
}
