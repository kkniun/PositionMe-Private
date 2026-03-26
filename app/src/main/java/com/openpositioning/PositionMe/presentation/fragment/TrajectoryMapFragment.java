package com.openpositioning.PositionMe.presentation.fragment;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
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
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;
import com.openpositioning.PositionMe.BuildConfig;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.CameraPosition;
import com.google.android.gms.maps.model.BitmapDescriptor;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.model.Polyline;
import com.google.android.gms.maps.model.PolylineOptions;
import com.google.android.material.floatingactionbutton.FloatingActionButton;
import com.google.android.material.switchmaterial.SwitchMaterial;
import com.openpositioning.PositionMe.R;
import com.openpositioning.PositionMe.presentation.activity.RecordingActivity;
import com.openpositioning.PositionMe.sensors.FusedPose;
import com.openpositioning.PositionMe.sensors.SensorFusion;
import com.openpositioning.PositionMe.utils.BuildingPolygon;
import com.openpositioning.PositionMe.utils.IndoorMapManager;
import com.openpositioning.PositionMe.utils.UtilFunctions;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

public class TrajectoryMapFragment extends Fragment {
    private static final String ARROW_DBG_TAG = "ARROW_DBG";
    private static final long ARROW_DBG_INTERVAL_MS = 500L;

    private static final int MAX_RAW_OBSERVATIONS = 12;
    private static final long FUSED_PATH_UPDATE_INTERVAL_MS = 1000L;
    private static final double FUSED_PATH_MOVEMENT_THRESHOLD_M = 0.75;
    // Suppress visually duplicate trajectory vertices when the displayed position is effectively stationary.
    private static final double TRAJECTORY_DUPLICATE_SUPPRESS_THRESHOLD_M = 0.15;
    private static final double RAW_POINT_DUPLICATE_THRESHOLD_M = 0.25;
    private static final float HEADING_SMOOTHING_FACTOR = 0.15f;
    // 地图 marker 的视觉基准当前比期望“尖头朝上”右偏约 90 度，只在显示层做常量修正。
    private static final float FUSED_MARKER_VISUAL_OFFSET_DEG = 0f;
    private static final double DISPLAY_SMOOTHING_ALPHA = 0.35;
    private static final float DISABLED_ACTION_ALPHA = 0.4f;
    private static final float ENABLED_ACTION_ALPHA = 1f;
    private static final float DEFAULT_WAITING_CAMERA_ZOOM = 17.2f;
    private static final int FUSED_COLOR = Color.parseColor("#D32F2F");
    private static final int GNSS_COLOR = Color.parseColor("#1976D2");
    private static final int WIFI_COLOR = Color.parseColor("#F57C00");
    private static final int PDR_COLOR = Color.parseColor("#00897B");

    public enum MapMatchingUiState {
        WAITING_FOR_ABSOLUTE_FIX,
        PENDING,
        ACTIVE,
        UNAVAILABLE
    }

    private GoogleMap gMap;
    private LatLng currentLocation;
    private LatLng currentRawLocation;
    private LatLng smoothedFusedLocation;
    private float smoothedFusedMarkerRotationDeg = Float.NaN;
    private Marker fusedMarker;
    private Polyline fusedTrajectoryPolyline;
    private LatLng pendingCameraPosition;
    private LatLng pendingIndoorMapAnchor;
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
    private long lastFusedTrajectoryUpdateTimestampMs = -1L;
    // 地图箭头显示日志节流。
    private long arrowDbgMarkerLastLogMs = 0L;

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
            smoothedFusedMarkerRotationDeg = fusedMarker != null
                    ? fusedMarker.getRotation()
                    : Float.NaN;
            refreshFusedTrajectoryPolyline();
            if (currentRawLocation != null) {
                LatLng displayedLocation = resolveDisplayLocation(currentRawLocation);
                currentLocation = displayedLocation;
                updateFusedMarker(
                        displayedLocation,
                        fusedMarker != null
                                ? (float) normalizeDegrees(
                                        fusedMarker.getRotation() - FUSED_MARKER_VISUAL_OFFSET_DEG
                                )
                                : 0f
                );
            }
        });

        autoFloorSwitch.setOnCheckedChangeListener((compoundButton, isChecked) -> {
            if (!isChecked || indoorMapManager == null || !indoorMapManager.getIsIndoorMapSet()) {
                return;
            }
            float floorHeight = indoorMapManager.getFloorHeight();
            if (floorHeight <= 0f) {
                return;
            }
            int estimatedFloor = (int) (sensorFusion.getElevation() / floorHeight);
            indoorMapManager.setCurrentFloor(estimatedFloor, true);
            updateVenueLabel();
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
        indoorMapManager.setAutoSelectFirstVenue(true);
        indoorMapManager.setVenueSelectionListener((venueId, venueName) -> {
            if (venueSelectionEnabled) {
                sensorFusion.setCollectionVenue(venueId);
                sensorFusion.setVenueIdForTrajectory(venueId);
            }
            if (venueId != null) {
                sensorFusion.setVenueFloorHeightMeters(indoorMapManager.getFloorHeight());
            } else {
                sensorFusion.clearVenueFloorHeightOverride();
            }
            updateVenueLabel();
            setFloorControlsVisibility(indoorMapManager.getIsIndoorMapSet() ? View.VISIBLE : View.GONE);
            if (venueId != null && !indoorMapManager.hasVectorMapShapes()) {
                Toast.makeText(
                        requireContext(),
                        "Warning: Map Physics Disabled (Image-Only Venue)",
                        Toast.LENGTH_LONG
                ).show();
            }
        });

        fusedTrajectoryPolyline = map.addPolyline(new PolylineOptions()
                .color(FUSED_COLOR)
                .width(7f)
                .zIndex(2f));
        maybeRefreshIndoorMapContext(getBestAvailableMapAnchor());
        updateVenueLabel();
        updateLayerVisibility();
        updateWaitingStateUi();
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
        sensorFusion.noteDisplayedFusedMarker(newLocation, displayedLocation, timestampMs);
        logUiFusedTrace(newLocation, displayedLocation, timestampMs);

        updateFusedMarker(displayedLocation, orientation);
        maybeAppendFusedTrajectory(newLocation, timestampMs);

        if (!hasAutoCenteredOnFirstFix) {
            gMap.moveCamera(CameraUpdateFactory.newLatLngZoom(displayedLocation, 19f));
            hasAutoCenteredOnFirstFix = true;
        }
        maybeRefreshIndoorMapContext(newLocation);
        updateWaitingStateUi();
    }

    public void updateGNSS(@NonNull LatLng gnssLocation) {
        addObservationMarker(
                gnssObservationMarkers,
                lastGnssObservation,
                gnssLocation,
                "GNSS Position",
                null,
                getGnssObservationIcon(),
                isGnssOn,
                1f
        );
        logUiObservationTrace("GNSS", gnssLocation, null);
        lastGnssObservation = gnssLocation;
        maybeRefreshIndoorMapContext(gnssLocation);
        updateWaitingStateUi();
    }

    public void updateWifiFix(@NonNull LatLng wifiLocation, int floor) {
        addObservationMarker(
                wifiObservationMarkers,
                lastWifiObservation,
                wifiLocation,
                "WiFi Position",
                "Floor " + floor,
                getWifiObservationIcon(),
                isWifiOn,
                1.2f
        );
        logUiObservationTrace("WIFI", wifiLocation, floor);
        lastWifiObservation = wifiLocation;
        maybeRefreshIndoorMapContext(wifiLocation);
        updateWaitingStateUi();
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
        logUiObservationTrace("PDR", pdrLocation, null);
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
                && indoorMapManager.hasActiveMapMatchingConstraints();
    }

    @NonNull
    public MapMatchingUiState getMapMatchingUiState() {
        boolean hasAbsoluteAnchor = getBestAvailableMapAnchor() != null;
        if (!hasAbsoluteAnchor) {
            return MapMatchingUiState.WAITING_FOR_ABSOLUTE_FIX;
        }
        if (indoorMapManager == null) {
            return MapMatchingUiState.PENDING;
        }
        if (indoorMapManager.hasActiveMapMatchingConstraints()) {
            return MapMatchingUiState.ACTIVE;
        }
        if (!indoorMapManager.hasRequestedNearbyVenues() || indoorMapManager.isVenueRequestInFlight()) {
            return MapMatchingUiState.PENDING;
        }
        return MapMatchingUiState.UNAVAILABLE;
    }

    public boolean isDisplaySmoothingEnabled() {
        return displaySmoothingEnabled;
    }

    public void primeIndoorMapContext(@Nullable LatLng location) {
        if (location != null) {
            pendingIndoorMapAnchor = location;
        }
        maybeRefreshIndoorMapContext(location);
        updateWaitingStateUi();
    }

    public void syncDisplayedFloor(int floor) {
        if (indoorMapManager == null || !indoorMapManager.getIsIndoorMapSet()) {
            return;
        }
        indoorMapManager.setCurrentFloor(floor, true);
        updateVenueLabel();
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
        smoothedFusedMarkerRotationDeg = Float.NaN;
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
        boolean markerCreated = false;
        if (fusedMarker == null) {
            fusedMarker = gMap.addMarker(new MarkerOptions()
                    .position(displayedLocation)
                    .anchor(0.5f, 0.5f)
                    .flat(true)
                    .title("Fused Position")
                    .zIndex(4f)
                    .icon(getFusedMarkerIcon()));
            markerCreated = true;
        } else {
            fusedMarker.setPosition(displayedLocation);
        }
        if (fusedMarker != null) {
            float targetRotation = (float) normalizeDegrees(orientation + FUSED_MARKER_VISUAL_OFFSET_DEG);
            float markerRotation = resolveDisplayedMarkerRotation(targetRotation, markerCreated);
            fusedMarker.setRotation(markerRotation);
            fusedMarker.setVisible(isFusedOn);
            logArrowMarkerTrace(markerRotation, markerCreated);

            float trueHeading = (float) normalizeDegrees(
                    markerRotation - FUSED_MARKER_VISUAL_OFFSET_DEG
            );
            if (gMap != null) {
                CameraPosition currentCameraPosition = gMap.getCameraPosition();
                CameraPosition newCameraPosition = new CameraPosition.Builder(currentCameraPosition)
                        .bearing(trueHeading)
                        .build();
                gMap.animateCamera(
                        CameraUpdateFactory.newCameraPosition(newCameraPosition),
                        200,
                        null
                );
            }
        }
    }

    private float resolveDisplayedMarkerRotation(float targetRotation, boolean markerCreated) {
        if (!displaySmoothingEnabled) {
            smoothedFusedMarkerRotationDeg = targetRotation;
            return targetRotation;
        }
        if (markerCreated || !Float.isFinite(smoothedFusedMarkerRotationDeg)) {
            smoothedFusedMarkerRotationDeg = targetRotation;
            return targetRotation;
        }

        float currentRotation = smoothedFusedMarkerRotationDeg;
        float diff = (targetRotation - currentRotation + 180f + 360f) % 360f - 180f;
        float smoothedRotation = currentRotation + diff * HEADING_SMOOTHING_FACTOR;
        smoothedFusedMarkerRotationDeg = (float) normalizeDegrees(smoothedRotation);
        return smoothedFusedMarkerRotationDeg;
    }

    // 记录最终 setRotation 的角度，以及地图当前 camera bearing。
    private void logArrowMarkerTrace(float angleDeg, boolean markerCreated) {
        if (!BuildConfig.DEBUG) {
            return;
        }
        long now = android.os.SystemClock.elapsedRealtime();
        if (now - arrowDbgMarkerLastLogMs < ARROW_DBG_INTERVAL_MS) {
            return;
        }
        float cameraBearing = gMap == null || gMap.getCameraPosition() == null
                ? Float.NaN
                : gMap.getCameraPosition().bearing;
        Log.d(
                ARROW_DBG_TAG,
                "stage=TrajectoryMapFragment.setRotation"
                        + " setRotationRad=" + formatDebugDouble(Math.toRadians(angleDeg))
                        + " setRotationDeg=" + formatDebugDouble(angleDeg)
                        + " setRotationDeg360=" + formatDebugDouble(normalizeDegrees(angleDeg))
                        + " markerCreated=" + markerCreated
                        + " markerExists=" + (fusedMarker != null)
                        + " cameraBearingRad=" + formatDebugDouble(Math.toRadians(cameraBearing))
                        + " cameraBearingDeg=" + formatDebugDouble(cameraBearing)
                        + " cameraBearingDeg360=" + formatDebugDouble(normalizeDegrees(cameraBearing))
        );
        arrowDbgMarkerLastLogMs = now;
    }

    private double normalizeDegrees(double degrees) {
        if (!Double.isFinite(degrees)) {
            return Double.NaN;
        }
        double normalized = degrees % 360.0;
        return normalized < 0.0 ? normalized + 360.0 : normalized;
    }

    private void maybeAppendFusedTrajectory(@NonNull LatLng rawLocation, long timestampMs) {
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
        MapMatchingUiState mapState = getMapMatchingUiState();
        if (mapEmptyStateCard != null) {
            boolean showEmptyState = venueSelectionEnabled && !hasDisplayPose;
            mapEmptyStateCard.setVisibility(showEmptyState ? View.VISIBLE : View.GONE);
        }
        if (mapEmptyStateText != null) {
            int textRes = R.string.map_waiting_state;
            if (mapState == MapMatchingUiState.PENDING) {
                textRes = R.string.map_constraints_pending_state;
            } else if (mapState == MapMatchingUiState.UNAVAILABLE) {
                textRes = R.string.map_constraints_unavailable_state;
            }
            mapEmptyStateText.setText(getString(textRes));
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
        MapMatchingUiState mapState = getMapMatchingUiState();
        if (mapState == MapMatchingUiState.WAITING_FOR_ABSOLUTE_FIX) {
            selectedVenueText.setText(getString(R.string.venue_waiting_fix));
            return;
        }
        if (mapState == MapMatchingUiState.PENDING) {
            selectedVenueText.setText(getString(R.string.venue_lookup_loading));
            return;
        }
        if (indoorMapManager == null || TextUtils.isEmpty(indoorMapManager.getSelectedVenueName())) {
            selectedVenueText.setText(getString(R.string.venue_not_selected));
            return;
        }
        if (!indoorMapManager.hasVectorMapShapes()) {
            selectedVenueText.setText(getString(
                    R.string.venue_selected_display_only,
                    indoorMapManager.getSelectedVenueName()
            ));
            return;
        }
        String venueName = indoorMapManager.getSelectedVenueName();
        int floorCount = indoorMapManager.getFloorCount();
        if (floorCount <= 0) {
            selectedVenueText.setText(getString(R.string.venue_selected_no_floor, venueName));
            return;
        }
        int floorNumber = indoorMapManager.getCurrentFloor();
        selectedVenueText.setText(getString(R.string.venue_selected_format, venueName, floorNumber, floorCount));
    }

    private void setFloorControlsVisibility(int visibility) {
        int finalVisibility = venueSelectionEnabled ? visibility : View.GONE;
        floorUpButton.setVisibility(finalVisibility);
        floorDownButton.setVisibility(finalVisibility);
        autoFloorSwitch.setVisibility(finalVisibility);
    }

    private void maybeRefreshIndoorMapContext(@Nullable LatLng seedLocation) {
        if (indoorMapManager == null) {
            return;
        }
        LatLng requestLocation = seedLocation != null ? seedLocation : getBestAvailableMapAnchor();
        if (requestLocation == null) {
            return;
        }
        indoorMapManager.setCurrentLocation(requestLocation);
        indoorMapManager.refreshNearbyVenues(requestLocation, sensorFusion.getWifiList());
        setFloorControlsVisibility(indoorMapManager.getIsIndoorMapSet() ? View.VISIBLE : View.GONE);
        updateVenueLabel();
    }

    @Nullable
    private LatLng getBestAvailableMapAnchor() {
        if (currentRawLocation != null) {
            return currentRawLocation;
        }
        LatLng gnssLocation = sensorFusion.getCurrentGnssLatLng();
        if (gnssLocation != null) {
            return gnssLocation;
        }
        LatLng wifiLocation = sensorFusion.getLatLngWifiPositioning();
        if (wifiLocation != null) {
            return wifiLocation;
        }
        FusedPose fusedPose = sensorFusion.getLatestFusedPose();
        if (fusedPose != null) {
            return sensorFusion.getLatLngForFusedPose(fusedPose);
        }
        float[] startLocation = sensorFusion.getGNSSLatitude(true);
        if (startLocation != null
                && startLocation.length >= 2
                && !(startLocation[0] == 0f && startLocation[1] == 0f)) {
            return new LatLng(startLocation[0], startLocation[1]);
        }
        return pendingIndoorMapAnchor;
    }

    private void logUiFusedTrace(
            @NonNull LatLng rawLocation,
            @NonNull LatLng displayedLocation,
            long timestampMs
    ) {
        if (!SensorFusion.DEBUG_FUSION_TRACE) {
            return;
        }
        double[] rawLocal = sensorFusion.getLocalMetersForLatLng(rawLocation);
        double[] displayedLocal = sensorFusion.getLocalMetersForLatLng(displayedLocation);
        FusedPose fusedPose = sensorFusion.getLatestFusedPose();
        Log.d(
                "TrajectoryMapFragment",
                "UI_DBG ts=" + timestampMs
                        + " source=FUSED"
                        + " rawLatLon=" + formatLatLng(rawLocation)
                        + " rawEN=" + formatLocal(rawLocal)
                        + " displayedLatLon=" + formatLatLng(displayedLocation)
                        + " displayedEN=" + formatLocal(displayedLocal)
                        + " fused=" + formatFusedPose(fusedPose)
                        + " displayedFloor=" + getDisplayedFloorForDebug()
                        + " smoothing=" + displaySmoothingEnabled
                        + " displayAgeMs=" + Math.max(0L, System.currentTimeMillis() - timestampMs)
        );
    }

    private void logUiObservationTrace(
            @NonNull String source,
            @NonNull LatLng location,
            @Nullable Integer observationFloor
    ) {
        if (!SensorFusion.DEBUG_FUSION_TRACE) {
            return;
        }
        double[] local = sensorFusion.getLocalMetersForLatLng(location);
        FusedPose fusedPose = sensorFusion.getLatestFusedPose();
        Log.d(
                "TrajectoryMapFragment",
                "UI_DBG ts=" + System.currentTimeMillis()
                        + " source=" + source
                        + " markerLatLon=" + formatLatLng(location)
                        + " markerEN=" + formatLocal(local)
                        + " markerFloor=" + (observationFloor == null ? "n/a" : observationFloor)
                        + " displayedFloor=" + getDisplayedFloorForDebug()
                        + " fused=" + formatFusedPose(fusedPose)
                        + " smoothing=" + displaySmoothingEnabled
        );
    }

    private int getDisplayedFloorForDebug() {
        return indoorMapManager == null ? -1 : indoorMapManager.getCurrentFloor();
    }

    @NonNull
    private String formatFusedPose(@Nullable FusedPose fusedPose) {
        if (fusedPose == null) {
            return "n/a";
        }
        return "{e=" + formatDebugDouble(fusedPose.getX())
                + ",n=" + formatDebugDouble(fusedPose.getY())
                + ",floor=" + fusedPose.getFloor()
                + ",conf=" + formatDebugDouble(fusedPose.getConfidence())
                + "}";
    }

    @NonNull
    private String formatLatLng(@Nullable LatLng latLng) {
        if (latLng == null) {
            return "n/a";
        }
        return "{lat=" + formatDebugDouble(latLng.latitude)
                + ",lon=" + formatDebugDouble(latLng.longitude)
                + "}";
    }

    @NonNull
    private String formatLocal(@Nullable double[] localMeters) {
        if (localMeters == null || localMeters.length < 2) {
            return "n/a";
        }
        return "{e=" + formatDebugDouble(localMeters[0])
                + ",n=" + formatDebugDouble(localMeters[1])
                + "}";
    }

    @NonNull
    private String formatDebugDouble(double value) {
        if (!Double.isFinite(value)) {
            return "n/a";
        }
        return String.format(Locale.US, "%.3f", value);
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

}
