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
import com.openpositioning.PositionMe.presentation.map.DisplayPoseFilter;
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
    private static final String DISPLAY_DIAG_TAG = "DISPLAY_DIAG";
    private static final String FLOOR_DIAG_TAG = "FloorDiag";
    private static final long ARROW_DBG_INTERVAL_MS = 500L;

    private static final int MAX_RAW_OBSERVATIONS = 12;
    private static final long FUSED_PATH_UPDATE_INTERVAL_MS = 1000L;
    private static final double FUSED_PATH_MOVEMENT_THRESHOLD_M = 0.75;
    // Suppress visually duplicate trajectory vertices when the displayed position is effectively stationary.
    private static final double TRAJECTORY_DUPLICATE_SUPPRESS_THRESHOLD_M = 0.15;
    private static final double RAW_POINT_DUPLICATE_THRESHOLD_M = 0.25;
    private static final double TRAJECTORY_STATIONARY_HINT_THRESHOLD_M = 0.45;
    private static final long MARKER_FAST_FOLLOW_WINDOW_MS = 1_500L;
    private static final double HEADING_FREEZE_LOW_CONFIDENCE_THRESHOLD = 0.40;
    // 地图 marker 的视觉基准当前比期望“尖头朝上”右偏约 90 度，只在显示层做常量修正。
    private static final float FUSED_MARKER_VISUAL_OFFSET_DEG = 0f;
    private static final float DISABLED_ACTION_ALPHA = 0.4f;
    private static final float ENABLED_ACTION_ALPHA = 1f;
    private static final float DEFAULT_WAITING_CAMERA_ZOOM = 17.2f;
    private static final int FUSED_COLOR = Color.parseColor("#D32F2F");
    private static final int GNSS_COLOR = Color.parseColor("#1976D2");
    private static final int WIFI_COLOR = Color.parseColor("#F57C00");
    private static final int PDR_COLOR = Color.parseColor("#00897B");

    public static final class DisplayDebugSnapshot {
        public final long poseTimestampMs;
        public final long lastMarkerMoveTimestampMs;
        public final boolean markerMoved;
        public final double markerMoveDistanceMeters;
        @NonNull public final String lastDecision;
        @NonNull public final String lastRejectReason;
        @NonNull public final String headingSource;
        public final float headingMotionDeg;
        public final float headingDeviceDeg;
        public final float headingRenderedDeg;
        public final boolean markerHoldActive;
        @NonNull public final String markerHoldReason;
        public final long markerHoldAgeMs;

        DisplayDebugSnapshot(
                long poseTimestampMs,
                long lastMarkerMoveTimestampMs,
                boolean markerMoved,
                double markerMoveDistanceMeters,
                @NonNull String lastDecision,
                @NonNull String lastRejectReason,
                @NonNull String headingSource,
                float headingMotionDeg,
                float headingDeviceDeg,
                float headingRenderedDeg,
                boolean markerHoldActive,
                @NonNull String markerHoldReason,
                long markerHoldAgeMs
        ) {
            this.poseTimestampMs = poseTimestampMs;
            this.lastMarkerMoveTimestampMs = lastMarkerMoveTimestampMs;
            this.markerMoved = markerMoved;
            this.markerMoveDistanceMeters = markerMoveDistanceMeters;
            this.lastDecision = lastDecision;
            this.lastRejectReason = lastRejectReason;
            this.headingSource = headingSource;
            this.headingMotionDeg = headingMotionDeg;
            this.headingDeviceDeg = headingDeviceDeg;
            this.headingRenderedDeg = headingRenderedDeg;
            this.markerHoldActive = markerHoldActive;
            this.markerHoldReason = markerHoldReason;
            this.markerHoldAgeMs = markerHoldAgeMs;
        }
    }

    private static final class MarkerHoldState {
        final boolean active;
        @NonNull final String reason;
        final long ageMs;

        MarkerHoldState(boolean active, @NonNull String reason, long ageMs) {
            this.active = active;
            this.reason = reason;
            this.ageMs = ageMs;
        }
    }

    private GoogleMap gMap;
    private LatLng currentLocation;
    private LatLng currentRawLocation;
    private float lastDisplayedHeadingDeg = Float.NaN;
    private Marker fusedMarker;
    private Polyline fusedTrajectoryPolyline;
    private LatLng pendingCameraPosition;
    private LatLng pendingIndoorMapAnchor;
    private boolean hasPendingCameraMove;
    private boolean hasAutoCenteredOnFirstFix;
    // Tracks an explicit user floor button action. This must never be reused by auto-floor paths
    // unless the current SensorFusion floor is already trusted.
    private boolean hasPendingManualFloorRecalibration;
    private boolean isFusedOn = true;
    private boolean isGnssOn = true;
    private boolean isWifiOn = true;
    private boolean isPdrOn = true;
    private boolean displaySmoothingEnabled = true;
    private boolean venueSelectionEnabled;

    private final List<LatLng> fusedTrajectoryRawPoints = new ArrayList<>();
    private final List<Integer> fusedTrajectoryFloors = new ArrayList<>();
    private final ArrayDeque<Marker> gnssObservationMarkers = new ArrayDeque<>();
    private final ArrayDeque<Marker> wifiObservationMarkers = new ArrayDeque<>();
    private final ArrayDeque<Marker> pdrObservationMarkers = new ArrayDeque<>();
    private final List<Marker> testPointMarkers = new ArrayList<>();
    private final Map<Integer, BitmapDescriptor> testPointIconCache = new HashMap<>();
    private final DisplayPoseFilter markerDisplayFilter =
            new DisplayPoseFilter(DisplayPoseFilter.defaultMarkerConfig());

    private LatLng lastFusedTrajectoryRawLocation;
    private int lastFusedTrajectoryFloor = Integer.MIN_VALUE;
    private LatLng lastGnssObservation;
    private LatLng lastWifiObservation;
    private LatLng lastPdrObservation;
    private long lastFusedTrajectoryUpdateTimestampMs = -1L;
    private long lastRenderedFusedPoseTimestampMs = Long.MIN_VALUE;
    private int lastRenderedFusedFloor = Integer.MIN_VALUE;
    private long fusedTrajectorySessionStartTimestampMs = Long.MIN_VALUE;
    private boolean fusedTrajectoryRenderingArmed;
    // 地图箭头显示日志节流。
    private long arrowDbgMarkerLastLogMs = 0L;
    private long lastDisplayPoseTimestampMs = Long.MIN_VALUE;
    private long lastMarkerMoveTimestampMs = Long.MIN_VALUE;
    private boolean lastDisplayMarkerMoved;
    private double lastDisplayMarkerMoveDistanceMeters = Double.NaN;
    @NonNull
    private String lastDisplayDecision = "init";
    @NonNull
    private String lastDisplayRejectReason = "none";
    @NonNull
    private String lastHeadingUpdateLogState = "init";

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
            markerDisplayFilter.reset();
            lastDisplayedHeadingDeg = fusedMarker != null
                    ? (float) normalizeDegrees(fusedMarker.getRotation())
                    : Float.NaN;
            refreshFusedTrajectoryPolyline();
            if (currentRawLocation != null) {
                Integer displayFloor = resolveCurrentDisplayFloor();
                if (displayFloor == null) {
                    return;
                }
                LatLng displayedLocation = resolveDisplayLocation(
                        currentRawLocation,
                        displayFloor,
                        System.currentTimeMillis()
                );
                currentLocation = displayedLocation;
                updateFusedMarker(
                        displayedLocation,
                        fusedMarker != null
                                ? (float) normalizeDegrees(
                                        fusedMarker.getRotation() - FUSED_MARKER_VISUAL_OFFSET_DEG
                                )
                                : 0f,
                        false
                );
            }
        });

        autoFloorSwitch.setOnCheckedChangeListener((compoundButton, isChecked) -> {
            if (!isChecked || indoorMapManager == null) {
                return;
            }
            if (hasPendingManualFloorRecalibration) {
                maybeApplyPendingManualFloorRecalibration();
                hasPendingManualFloorRecalibration = false;
            }
            // Modified: bootstrap auto-floor from the best known absolute floor, not relative height.
            Integer trustedFloor = resolveBestKnownFloorForDisplay();
            if (trustedFloor != null) {
                logFloorDiag(
                        "event=map_auto_floor_sync source=auto_floor_toggle"
                                + " floor=" + trustedFloor
                                + " trustedSensorFloor=true"
                );
                indoorMapManager.setCurrentFloor(trustedFloor, false);
            } else {
                logFloorDiag(
                        "event=map_auto_floor_sync source=auto_floor_toggle"
                                + " floor=null trustedSensorFloor=false"
                );
            }
            updateVenueLabel();
        });

        floorUpButton.setOnClickListener(v -> {
            autoFloorSwitch.setChecked(false);
            if (indoorMapManager != null) {
                indoorMapManager.increaseFloor();
                hasPendingManualFloorRecalibration = true;
                updateVenueLabel();
            }
        });

        floorDownButton.setOnClickListener(v -> {
            autoFloorSwitch.setChecked(false);
            if (indoorMapManager != null) {
                indoorMapManager.decreaseFloor();
                hasPendingManualFloorRecalibration = true;
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
            hasPendingManualFloorRecalibration = false;
            if (venueSelectionEnabled) {
                sensorFusion.setCollectionVenue(venueId);
                sensorFusion.setVenueIdForTrajectory(venueId);
            }
            if (venueId != null) {
                sensorFusion.setVenueFloorHeightMeters(indoorMapManager.getFloorHeight());
                if (autoFloorSwitch != null && autoFloorSwitch.isChecked()) {
                    // Modified: when a venue becomes active, immediately snap the map to the
                    // currently known floor instead of waiting for a later refresh.
                    Integer trustedFloor = resolveBestKnownFloorForDisplay();
                    if (trustedFloor != null) {
                        logFloorDiag(
                                "event=map_auto_floor_sync source=venue_selected"
                                        + " floor=" + trustedFloor
                                        + " trustedSensorFloor=true"
                        );
                        indoorMapManager.setCurrentFloor(trustedFloor, false);
                    } else {
                        logFloorDiag(
                                "event=map_auto_floor_sync source=venue_selected"
                                        + " floor=null trustedSensorFloor=false"
                        );
                    }
                }
            } else {
                sensorFusion.clearVenueFloorHeightOverride();
            }
            updateVenueLabel();
            setFloorControlsVisibility(indoorMapManager.getIsIndoorMapSet() ? View.VISIBLE : View.GONE);
            if (venueId != null && !indoorMapManager.hasVectorMapShapes()) {
                Toast.makeText(
                        requireContext(),
                        getString(R.string.map_display_only_toast),
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
        Integer displayFloor = resolveCurrentDisplayFloor();
        if (displayFloor == null) {
            return;
        }
        updateUserLocation(newLocation, orientation, displayFloor, System.currentTimeMillis());
    }

    public void updateUserLocation(
            @NonNull LatLng newLocation,
            float orientation,
            @Nullable Integer trustedFloor,
            long timestampMs
    ) {
        Integer displayFloor = resolveIncomingDisplayFloor(
                trustedFloor,
                indoorMapManager == null ? null : indoorMapManager.getCurrentFloor(),
                lastRenderedFusedFloor == Integer.MIN_VALUE ? null : lastRenderedFusedFloor
        );
        if (displayFloor == null) {
            return;
        }
        updateUserLocation(newLocation, orientation, displayFloor.intValue(), timestampMs);
    }

    public void updateUserLocation(@NonNull LatLng newLocation, float orientation, int floor, long timestampMs) {
        if (gMap == null) {
            return;
        }
        boolean internalPoseUpdated = timestampMs != lastRenderedFusedPoseTimestampMs;
        if (timestampMs == lastRenderedFusedPoseTimestampMs) {
            recordDisplayDecision(
                    false,
                    false,
                    timestampMs,
                    false,
                    0.0,
                    "duplicate_fused_timestamp_ignored",
                    "none"
            );
            return;
        }
        boolean crossFloorDisplayUpdate = DisplayFloorResetGate.requiresResetToken(lastRenderedFusedFloor, floor);
        boolean authorizedFloorReset = crossFloorDisplayUpdate
                && sensorFusion != null
                && sensorFusion.isDisplayFloorChangeReady(floor, timestampMs);
        boolean hasPendingFloorResetToken = crossFloorDisplayUpdate
                && sensorFusion != null
                && sensorFusion.hasPendingDisplayFloorReset(floor, timestampMs);
        LatLng crossFloorLandingLocation = hasPendingFloorResetToken && sensorFusion != null
                ? sensorFusion.peekPendingDisplayFloorResetLanding(floor, timestampMs)
                : null;
        if (DisplayFloorResetGate.shouldHoldCrossFloorUpdate(
                lastRenderedFusedFloor,
                floor,
                authorizedFloorReset
        )) {
            recordDisplayDecision(
                    internalPoseUpdated,
                    false,
                    timestampMs,
                    false,
                    0.0,
                    "held_last_valid_pose",
                    "cross_floor_not_authorized"
            );
            if (currentLocation != null) {
                updateFusedMarker(currentLocation, orientation, true);
            }
            lastRenderedFusedPoseTimestampMs = timestampMs;
            return;
        }
        LatLng previousRawLocation = currentRawLocation;
        LatLng previousDisplayedLocation = currentLocation;
        boolean rawPoseConstrained = false;
        boolean usedCrossFloorLandingCorrection = false;
        if (!MapDisplayConstraintFilter.isRenderablePoint(newLocation, floor)) {
            LatLng constrainedRawLocation = crossFloorDisplayUpdate
                    ? MapDisplayConstraintFilter.resolveBestRenderableDisplayLocation(
                    currentLocation,
                    newLocation,
                    crossFloorLandingLocation,
                    lastRenderedFusedFloor,
                    floor,
                    crossFloorLandingLocation
            )
                    : MapDisplayConstraintFilter.constrainToRenderableSegmentPrefix(
                    currentRawLocation,
                    newLocation,
                    lastRenderedFusedFloor,
                    floor
            );
            if (constrainedRawLocation == null) {
                recordDisplayDecision(
                        internalPoseUpdated,
                        false,
                        timestampMs,
                        false,
                        0.0,
                        "raw_pose_rejected_keep_previous",
                        "raw_pose_not_renderable"
                );
                if (currentLocation != null) {
                    updateFusedMarker(currentLocation, orientation, true);
                }
                lastRenderedFusedPoseTimestampMs = timestampMs;
                return;
            }
            newLocation = constrainedRawLocation;
            rawPoseConstrained = true;
            usedCrossFloorLandingCorrection = crossFloorDisplayUpdate
                    && crossFloorLandingLocation != null
                    && crossFloorLandingLocation.equals(constrainedRawLocation);
        }

        LatLng renderedRawLocation = !crossFloorDisplayUpdate && shouldHoldStationaryDisplayPosition(newLocation)
                ? currentRawLocation
                : newLocation;
        boolean usedConstrainedCorrection = rawPoseConstrained;
        boolean usedRawDisplayFallback = false;
        if (!crossFloorDisplayUpdate && MapDisplayConstraintFilter.shouldHoldPositionForIllegalTransition(
                currentRawLocation,
                renderedRawLocation,
                lastRenderedFusedFloor,
                floor
        )) {
            LatLng constrainedRawLocation = MapDisplayConstraintFilter.constrainToRenderableSegmentPrefix(
                    currentRawLocation,
                    renderedRawLocation,
                    lastRenderedFusedFloor,
                    floor
            );
            if (constrainedRawLocation == null) {
                recordDisplayDecision(
                        internalPoseUpdated,
                        false,
                        timestampMs,
                        false,
                        0.0,
                        "raw_transition_rejected_keep_previous",
                        "raw_transition_illegal"
                );
                Log.i("TrajectoryMapFragment", "DISPLAY:hold_illegal_transition reason=raw_transition_illegal");
                renderedRawLocation = currentRawLocation;
            } else {
                renderedRawLocation = constrainedRawLocation;
                usedConstrainedCorrection = true;
                Log.i(
                        "TrajectoryMapFragment",
                        "DISPLAY:accept_constrained_correction stage=raw"
                                + " distanceM="
                                + String.format(Locale.US, "%.3f",
                                UtilFunctions.distanceBetweenPoints(currentRawLocation, constrainedRawLocation))
                );
            }
        }
        if (renderedRawLocation == null) {
            recordDisplayDecision(
                    internalPoseUpdated,
                    false,
                    timestampMs,
                    false,
                    0.0,
                    "raw_pose_missing_keep_previous",
                    "raw_pose_null_after_rejection"
            );
            if (currentLocation != null) {
                updateFusedMarker(currentLocation, orientation, true);
            }
            lastRenderedFusedPoseTimestampMs = timestampMs;
            return;
        }
        currentRawLocation = renderedRawLocation;
        if ((crossFloorDisplayUpdate && authorizedFloorReset)
                || (sensorFusion != null && sensorFusion.shouldFastFollowDisplayedMarker(timestampMs))) {
            markerDisplayFilter.armFastFollowWindow(timestampMs, MARKER_FAST_FOLLOW_WINDOW_MS);
        }
        LatLng displayedLocation = resolveDisplayLocation(renderedRawLocation, floor, timestampMs);
        LatLng smoothedDisplayedLocation = displayedLocation;
        boolean displayPointRenderable =
                MapDisplayConstraintFilter.isRenderablePoint(displayedLocation, floor);
        boolean displayTransitionIllegal = !crossFloorDisplayUpdate
                && MapDisplayConstraintFilter.shouldHoldPositionForIllegalTransition(
                currentLocation,
                displayedLocation,
                lastRenderedFusedFloor,
                floor
        );
        if (!displayPointRenderable || displayTransitionIllegal) {
            String rejectReason = !displayPointRenderable
                    ? "display_pose_not_renderable"
                    : "display_transition_illegal";
            LatLng resolvedDisplayLocation = MapDisplayConstraintFilter.resolveBestRenderableDisplayLocation(
                    currentLocation,
                    renderedRawLocation,
                    displayedLocation,
                    lastRenderedFusedFloor,
                    floor,
                    crossFloorLandingLocation
            );
            if (resolvedDisplayLocation == null) {
                recordDisplayDecision(
                        internalPoseUpdated,
                        false,
                        timestampMs,
                    false,
                    0.0,
                    "display_pose_rejected_keep_previous",
                    rejectReason
                );
                Log.i("TrajectoryMapFragment", "DISPLAY:hold_illegal_transition reason=" + rejectReason);
                displayedLocation = currentLocation != null ? currentLocation : renderedRawLocation;
                markerDisplayFilter.updatePosition(
                        displayedLocation,
                        sensorFusion != null && sensorFusion.isStationary(),
                        true
                );
            } else {
                displayedLocation = resolvedDisplayLocation;
                markerDisplayFilter.updatePosition(
                        displayedLocation,
                        sensorFusion != null && sensorFusion.isStationary(),
                        true
                );
                if (displayedLocation.equals(renderedRawLocation)
                        && !displayedLocation.equals(smoothedDisplayedLocation)) {
                    usedRawDisplayFallback = true;
                    Log.i(
                            "TrajectoryMapFragment",
                            "DISPLAY:accept_raw_pose_bypass stage=display"
                                    + " reason=" + rejectReason
                    );
                } else {
                    usedConstrainedCorrection = true;
                    Log.i(
                            "TrajectoryMapFragment",
                            "DISPLAY:accept_constrained_correction stage=display"
                                    + " distanceM="
                                    + String.format(Locale.US, "%.3f",
                                    currentLocation == null
                                            ? 0.0
                                            : UtilFunctions.distanceBetweenPoints(currentLocation, displayedLocation))
                    );
                }
            }
        }
        boolean floorSwitchCommitted = false;
        if (crossFloorDisplayUpdate && authorizedFloorReset && sensorFusion != null) {
            floorSwitchCommitted = sensorFusion.commitDisplayFloorIfReady(
                    floor,
                    timestampMs,
                    displayedLocation
            );
            if (!floorSwitchCommitted) {
                currentRawLocation = previousRawLocation;
                markerDisplayFilter.reset();
                if (currentLocation != null) {
                    markerDisplayFilter.updatePosition(
                            currentLocation,
                            sensorFusion != null && sensorFusion.isStationary(),
                            true
                    );
                }
                recordDisplayDecision(
                        internalPoseUpdated,
                        false,
                        timestampMs,
                        false,
                        0.0,
                        "held_last_valid_pose",
                        "cross_floor_commit_failed"
                );
                if (currentLocation != null) {
                    updateFusedMarker(currentLocation, orientation, true);
                }
                lastRenderedFusedPoseTimestampMs = timestampMs;
                return;
            }
            commitDisplayedFloorImmediately(floor);
            lastRenderedFusedFloor = floor;
        }
        currentLocation = displayedLocation;
        sensorFusion.noteDisplayedFusedMarker(renderedRawLocation, displayedLocation, timestampMs, true, floor);
        logUiFusedTrace(renderedRawLocation, displayedLocation, timestampMs);

        updateFusedMarker(displayedLocation, orientation, false);
        double moveDistanceMeters = previousDisplayedLocation == null
                ? 0.0
                : UtilFunctions.distanceBetweenPoints(previousDisplayedLocation, displayedLocation);
        boolean markerMoved = previousDisplayedLocation == null
                || moveDistanceMeters > 0.01;
        recordDisplayDecision(
                internalPoseUpdated,
                true,
                timestampMs,
                markerMoved,
                moveDistanceMeters,
                usedConstrainedCorrection
                        ? "accepted_constrained_correction"
                        : usedCrossFloorLandingCorrection
                        ? "accepted_cross_floor_landing_correction"
                        : usedRawDisplayFallback
                        ? "accepted_raw_display_fallback"
                        : crossFloorDisplayUpdate
                        ? floorSwitchCommitted
                        ? "accepted_cross_floor_commit"
                        : "accepted_cross_floor_pending"
                        : "accepted_same_floor",
                "none"
        );
        maybeAppendFusedTrajectory(renderedRawLocation, floor, timestampMs);

        if (!hasAutoCenteredOnFirstFix) {
            gMap.moveCamera(CameraUpdateFactory.newLatLngZoom(displayedLocation, 19f));
            hasAutoCenteredOnFirstFix = true;
        }
        maybeRefreshIndoorMapContext(renderedRawLocation);
        updateWaitingStateUi();
        lastRenderedFusedPoseTimestampMs = timestampMs;
        lastRenderedFusedFloor = floor;
    }

    public void updateUserHeading(float orientation) {
        if (gMap == null) {
            return;
        }
        LatLng displayedLocation = currentLocation != null ? currentLocation : currentRawLocation;
        if (displayedLocation == null) {
            return;
        }
        updateFusedMarker(displayedLocation, orientation, true);
    }

    @NonNull
    public DisplayDebugSnapshot getDisplayDebugSnapshot() {
        DisplayPoseFilter.HeadingDebugSnapshot headingDebug = markerDisplayFilter.getHeadingDebugSnapshot();
        MarkerHoldState markerHoldState = resolveMarkerHoldState(System.currentTimeMillis());
        return new DisplayDebugSnapshot(
                lastDisplayPoseTimestampMs,
                lastMarkerMoveTimestampMs,
                lastDisplayMarkerMoved,
                lastDisplayMarkerMoveDistanceMeters,
                lastDisplayDecision,
                lastDisplayRejectReason,
                headingDebug.headingSource,
                headingDebug.motionHeadingDegrees,
                headingDebug.deviceHeadingDegrees,
                headingDebug.renderedHeadingDegrees,
                markerHoldState.active,
                markerHoldState.reason,
                markerHoldState.ageMs
        );
    }

    private void recordDisplayDecision(
            boolean internalPoseUpdated,
            boolean displayPoseApplied,
            long poseTimestampMs,
            boolean markerMoved,
            double moveDistanceMeters,
            @NonNull String decision,
            @NonNull String rejectReason
    ) {
        lastDisplayPoseTimestampMs = poseTimestampMs;
        lastDisplayMarkerMoved = markerMoved;
        lastDisplayMarkerMoveDistanceMeters = moveDistanceMeters;
        lastDisplayDecision = decision;
        if (!"none".equals(rejectReason)) {
            lastDisplayRejectReason = rejectReason;
        } else if (markerMoved) {
            lastDisplayRejectReason = "none";
        }
        if (markerMoved) {
            lastMarkerMoveTimestampMs = poseTimestampMs;
        }
        if (!BuildConfig.DEBUG) {
            return;
        }
        Log.d(
                DISPLAY_DIAG_TAG,
                "display_pose ts=" + poseTimestampMs
                        + " INTERNAL_POSE_UPDATED=" + internalPoseUpdated
                        + " DISPLAY_POSE_APPLIED=" + displayPoseApplied
                        + " decision=" + decision
                        + " markerMoved=" + markerMoved
                        + " moveDistanceM=" + String.format(Locale.US, "%.3f", moveDistanceMeters)
                        + " DISPLAY_REJECT_REASON=" + rejectReason
                        + " currentFloor=" + lastRenderedFusedFloor
                        + " ABSOLUTE_FLOOR=" + (sensorFusion == null ? Integer.MIN_VALUE : sensorFusion.getCurrentFloor())
                        + " FUSED_POSE_FLOOR="
                        + (sensorFusion == null || sensorFusion.getLatestFusedPose() == null
                        ? Integer.MIN_VALUE
                        : sensorFusion.getLatestFusedPose().getFloor())
                        + " TRUSTED_FLOOR=" + (sensorFusion == null ? Integer.MIN_VALUE : sensorFusion.getUserVisibleFloor())
                        + " DISPLAY_FLOOR=" + (indoorMapManager == null ? Integer.MIN_VALUE : indoorMapManager.getCurrentFloor())
        );
    }

    @NonNull
    private MarkerHoldState resolveMarkerHoldState(long nowMs) {
        boolean hasKnownPose = currentLocation != null || currentRawLocation != null;
        long poseAgeMs = lastRenderedFusedPoseTimestampMs == Long.MIN_VALUE
                ? Long.MAX_VALUE
                : Math.max(0L, nowMs - lastRenderedFusedPoseTimestampMs);
        String reason = MapPointerDisplayFilter.resolveMarkerHoldReason(
                hasKnownPose,
                sensorFusion != null && sensorFusion.getElevator(),
                sensorFusion != null && sensorFusion.isFloorOnlyResyncActive(),
                getMapMatchingUiState(),
                poseAgeMs
        );
        return new MarkerHoldState(
                !"none".equals(reason),
                reason,
                poseAgeMs == Long.MAX_VALUE ? -1L : poseAgeMs
        );
    }

    private boolean shouldHoldStationaryDisplayPosition(@NonNull LatLng newLocation) {
        return MapPointerDisplayFilter.shouldHoldStationaryDisplayPosition(
                currentRawLocation,
                newLocation,
                sensorFusion != null && sensorFusion.isStationary()
        );
    }

    public void updateGNSS(@NonNull LatLng gnssLocation) {
        addObservationMarker(
                gnssObservationMarkers,
                lastGnssObservation,
                gnssLocation,
                "Device Location",
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

    public void updateWifiFix(@NonNull LatLng wifiLocation, @Nullable Integer floor) {
        addObservationMarker(
                wifiObservationMarkers,
                lastWifiObservation,
                wifiLocation,
                "WiFi Position",
                floor == null ? null : "Floor " + floor,
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
    public MapMatchingStateResolver.MapMatchingUiState getMapMatchingUiState() {
        return MapMatchingStateResolver.resolve(
                getBestAvailableMapAnchor() != null,
                indoorMapManager != null,
                indoorMapManager != null && indoorMapManager.hasActiveMapMatchingConstraints(),
                indoorMapManager != null && indoorMapManager.hasDisplayOnlyIndoorMap(),
                indoorMapManager != null && indoorMapManager.hasRequestedNearbyVenues(),
                indoorMapManager != null && indoorMapManager.isVenueRequestInFlight()
        );
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
        if (indoorMapManager == null) {
            return;
        }
        indoorMapManager.setCurrentFloor(floor, true);
        updateVenueLabel();
    }

    private void commitDisplayedFloorImmediately(int floor) {
        if (indoorMapManager == null) {
            return;
        }
        if (isAutoFloorEnabled()) {
            indoorMapManager.setCurrentFloor(floor, false);
        }
        updateVenueLabel();
    }

    public void syncReplayDisplayedFloor(int floor) {
        if (indoorMapManager == null) {
            return;
        }
        Integer replayFloor = FloorDisplayGate.resolveReplayFloorForDisplay(floor);
        if (replayFloor == null) {
            return;
        }
        indoorMapManager.setCurrentFloor(replayFloor, false);
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
        hasPendingManualFloorRecalibration = false;
        fusedTrajectoryRawPoints.clear();
        fusedTrajectoryFloors.clear();
        lastFusedTrajectoryRawLocation = null;
        lastFusedTrajectoryFloor = Integer.MIN_VALUE;
        lastFusedTrajectoryUpdateTimestampMs = -1L;
        fusedTrajectorySessionStartTimestampMs = System.currentTimeMillis();
        fusedTrajectoryRenderingArmed = false;
        lastRenderedFusedPoseTimestampMs = Long.MIN_VALUE;
        lastRenderedFusedFloor = Integer.MIN_VALUE;
        lastDisplayPoseTimestampMs = Long.MIN_VALUE;
        lastMarkerMoveTimestampMs = Long.MIN_VALUE;
        lastDisplayMarkerMoved = false;
        lastDisplayMarkerMoveDistanceMeters = Double.NaN;
        lastDisplayDecision = "reset";
        lastDisplayRejectReason = "none";
        currentLocation = null;
        currentRawLocation = null;
        markerDisplayFilter.reset();
        lastDisplayedHeadingDeg = Float.NaN;
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

    private void updateFusedMarker(@NonNull LatLng displayedLocation, float orientation, boolean headingOnlyUpdate) {
        if (gMap == null) {
            return;
        }
        double displayMovementMeters = fusedMarker == null || fusedMarker.getPosition() == null
                ? Double.NaN
                : UtilFunctions.distanceBetweenPoints(fusedMarker.getPosition(), displayedLocation);
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
            MarkerHoldState markerHoldState = resolveMarkerHoldState(System.currentTimeMillis());
            String headingHoldReason = MapPointerDisplayFilter.resolveHeadingHoldReason(
                    markerCreated,
                    headingOnlyUpdate,
                    sensorFusion != null && sensorFusion.isStationary(),
                    isHeadingLowConfidence(),
                    markerHoldState.reason,
                    displayMovementMeters
            );
            float markerRotation = resolveDisplayedMarkerRotation(
                    targetRotation,
                    markerCreated,
                    headingHoldReason
            );
            DisplayPoseFilter.HeadingDebugSnapshot headingDebug = markerDisplayFilter.getHeadingDebugSnapshot();
            boolean headingBlocked = "hold_last_stable".equals(headingDebug.headingSource)
                    || "low_confidence_freeze".equals(headingDebug.headingSource);
            if (shouldUpdateDisplayedHeading(markerRotation, markerCreated)) {
                fusedMarker.setRotation(markerRotation);
            }
            lastDisplayedHeadingDeg = markerRotation;
            fusedMarker.setVisible(isFusedOn);
            float displayedRotation = Float.isFinite(lastDisplayedHeadingDeg)
                    ? lastDisplayedHeadingDeg
                    : markerRotation;
            logHeadingUpdateTrace(
                    headingOnlyUpdate,
                    headingBlocked,
                    headingHoldReason,
                    displayMovementMeters,
                    targetRotation,
                    displayedRotation
            );
            logArrowMarkerTrace(displayedRotation, markerCreated);
        }
    }

    private float resolveDisplayedMarkerRotation(
            float targetRotation,
            boolean markerCreated,
            @NonNull String headingHoldReason
    ) {
        boolean fastResponseHint = sensorFusion == null || !sensorFusion.isStationary();
        return markerDisplayFilter.updateHeading(
                targetRotation,
                fastResponseHint,
                !displaySmoothingEnabled || markerCreated,
                "none".equals(headingHoldReason) ? null : headingHoldReason
        );
    }

    private boolean isHeadingLowConfidence() {
        if (sensorFusion == null) {
            return false;
        }
        FusedPose latestFusedPose = sensorFusion.getLatestFusedPose();
        return latestFusedPose != null
                && latestFusedPose.getConfidence() < HEADING_FREEZE_LOW_CONFIDENCE_THRESHOLD;
    }

    private boolean shouldUpdateDisplayedHeading(float candidateRotation, boolean markerCreated) {
        if (markerCreated || !Float.isFinite(lastDisplayedHeadingDeg)) {
            return true;
        }
        return absoluteShortestAngularDifferenceDeg(candidateRotation, lastDisplayedHeadingDeg) > 0.01f;
    }

    private float absoluteShortestAngularDifferenceDeg(float fromDeg, float toDeg) {
        return Math.abs(shortestAngularDifferenceDeg(fromDeg, toDeg));
    }

    private float shortestAngularDifferenceDeg(float fromDeg, float toDeg) {
        return (fromDeg - toDeg + 180f + 360f) % 360f - 180f;
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

    private void logHeadingUpdateTrace(
            boolean headingOnlyUpdate,
            boolean headingBlocked,
            @NonNull String holdReason,
            double displayMovementMeters,
            float targetRotation,
            float renderedRotation
    ) {
        if (!BuildConfig.DEBUG) {
            return;
        }
        DisplayPoseFilter.HeadingDebugSnapshot headingDebug = markerDisplayFilter.getHeadingDebugSnapshot();
        String headingSourceLabel = headingBlocked
                ? "HELD"
                : "motion".equals(headingDebug.headingSource)
                ? "MOTION"
                : "DEVICE";
        String logState = headingDebug.headingSource
                + "|" + holdReason
                + "|" + headingOnlyUpdate
                + "|" + headingBlocked;
        if (logState.equals(lastHeadingUpdateLogState)) {
            return;
        }
        lastHeadingUpdateLogState = logState;
        Log.d(
                DISPLAY_DIAG_TAG,
                "heading_update"
                        + " HEADING_SOURCE=" + headingSourceLabel
                        + " HEADING_REASON=" + holdReason
                        + " headingOnlyUpdate=" + headingOnlyUpdate
                        + " allowed=" + (!headingBlocked)
                        + " blocked=" + headingBlocked
                        + " markerHoldReason=" + resolveMarkerHoldState(System.currentTimeMillis()).reason
                        + " motionDistanceM=" + (
                        Double.isFinite(displayMovementMeters)
                                ? String.format(Locale.US, "%.3f", displayMovementMeters)
                                : "n/a"
                )
                        + " headingDeltaDeg=" + String.format(
                        Locale.US,
                        "%.1f",
                        absoluteShortestAngularDifferenceDeg(targetRotation, renderedRotation)
                )
        );
    }

    private double normalizeDegrees(double degrees) {
        if (!Double.isFinite(degrees)) {
            return Double.NaN;
        }
        double normalized = degrees % 360.0;
        return normalized < 0.0 ? normalized + 360.0 : normalized;
    }

    private void maybeAppendFusedTrajectory(@NonNull LatLng rawLocation, int floor, long timestampMs) {
        if (!isFusedTrajectoryRenderingArmed()) {
            lastFusedTrajectoryRawLocation = rawLocation;
            lastFusedTrajectoryFloor = floor;
            lastFusedTrajectoryUpdateTimestampMs = timestampMs;
            return;
        }
        if (fusedTrajectoryRawPoints.isEmpty()) {
            fusedTrajectoryRawPoints.add(rawLocation);
            fusedTrajectoryFloors.add(floor);
            lastFusedTrajectoryRawLocation = rawLocation;
            lastFusedTrajectoryFloor = floor;
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
        if (lastFusedTrajectoryRawLocation != null
                && lastFusedTrajectoryFloor == floor
                && !MapDisplayConstraintFilter.isRenderableSegment(
                lastFusedTrajectoryRawLocation,
                rawLocation,
                floor,
                floor
        )) {
            return;
        }

        fusedTrajectoryRawPoints.add(rawLocation);
        fusedTrajectoryFloors.add(floor);
        lastFusedTrajectoryRawLocation = rawLocation;
        lastFusedTrajectoryFloor = floor;
        lastFusedTrajectoryUpdateTimestampMs = timestampMs;
        refreshFusedTrajectoryPolyline();
    }

    private boolean isFusedTrajectoryRenderingArmed() {
        if (sensorFusion == null) {
            return fusedTrajectoryRenderingArmed;
        }
        SensorFusion.MotionDebugSnapshot motionDebug = sensorFusion.getMotionDebugSnapshot();
        fusedTrajectoryRenderingArmed = MapPointerDisplayFilter.shouldArmTrajectoryRendering(
                fusedTrajectoryRenderingArmed,
                fusedTrajectorySessionStartTimestampMs,
                motionDebug.lastAcceptedStepTimestampMs
        );
        return fusedTrajectoryRenderingArmed;
    }

    private void refreshFusedTrajectoryPolyline() {
        if (fusedTrajectoryPolyline == null) {
            return;
        }
        List<LatLng> displayedPoints = new ArrayList<>(fusedTrajectoryRawPoints.size());
        if (displaySmoothingEnabled) {
            DisplayPoseFilter trajectoryDisplayFilter =
                    new DisplayPoseFilter(DisplayPoseFilter.defaultTrajectoryConfig());
            LatLng previousRawPoint = null;
            int smoothedFloor = Integer.MIN_VALUE;
            for (int i = 0; i < fusedTrajectoryRawPoints.size(); i++) {
                LatLng rawPoint = fusedTrajectoryRawPoints.get(i);
                int rawFloor = i < fusedTrajectoryFloors.size()
                        ? fusedTrajectoryFloors.get(i)
                        : smoothedFloor;
                LatLng currentDisplayedPoint = trajectoryDisplayFilter.getFilteredPosition();
                boolean bypassFilter = currentDisplayedPoint != null
                        && MapDisplayConstraintFilter.shouldBypassSmoothing(
                        currentDisplayedPoint,
                        rawPoint,
                        smoothedFloor,
                        rawFloor
                );
                boolean stationaryHint = previousRawPoint != null
                        && UtilFunctions.distanceBetweenPoints(previousRawPoint, rawPoint)
                        <= TRAJECTORY_STATIONARY_HINT_THRESHOLD_M;
                LatLng displayedPoint = trajectoryDisplayFilter.updatePosition(
                        rawPoint,
                        stationaryHint,
                        bypassFilter
                );
                if (currentDisplayedPoint != null
                        && (!MapDisplayConstraintFilter.isRenderablePoint(displayedPoint, rawFloor)
                        || !MapDisplayConstraintFilter.isRenderableSegment(
                        currentDisplayedPoint,
                        displayedPoint,
                        smoothedFloor,
                        rawFloor
                ))) {
                    displayedPoint = trajectoryDisplayFilter.updatePosition(
                            rawPoint,
                            stationaryHint,
                            true
                    );
                }
                displayedPoints.add(displayedPoint);
                previousRawPoint = rawPoint;
                smoothedFloor = rawFloor;
            }
        } else {
            displayedPoints.addAll(fusedTrajectoryRawPoints);
        }
        fusedTrajectoryPolyline.setPoints(displayedPoints);
        fusedTrajectoryPolyline.setVisible(isFusedOn);
    }

    @NonNull
    private LatLng resolveDisplayLocation(@NonNull LatLng rawLocation, int floor, long timestampMs) {
        LatLng currentDisplayedLocation = markerDisplayFilter.getFilteredPosition();
        boolean bypassFilter = currentDisplayedLocation != null
                && MapDisplayConstraintFilter.shouldBypassSmoothing(
                currentDisplayedLocation,
                rawLocation,
                lastRenderedFusedFloor,
                floor
        );
        return markerDisplayFilter.updatePosition(
                rawLocation,
                sensorFusion != null && sensorFusion.isStationary(),
                !displaySmoothingEnabled || bypassFilter,
                timestampMs
        );
    }

    @Nullable
    private Integer resolveCurrentDisplayFloor() {
        Integer trustedSensorFloor = sensorFusion == null
                ? null
                : FloorDisplayGate.resolveTrustedFloorForDisplay(
                        sensorFusion.isFloorCalibrated(),
                        sensorFusion.getUserVisibleFloor()
                );
        Integer currentMapFloor = indoorMapManager == null ? null : indoorMapManager.getCurrentFloor();
        Integer lastRenderedFloor = lastRenderedFusedFloor == Integer.MIN_VALUE ? null : lastRenderedFusedFloor;
        Integer resolvedFloor = resolveIncomingDisplayFloor(
                trustedSensorFloor,
                currentMapFloor,
                lastRenderedFloor
        );
        String source = trustedSensorFloor != null
                ? "trusted_sensor_floor"
                : currentMapFloor != null
                ? "current_map_floor"
                : lastRenderedFloor != null
                ? "last_rendered_floor"
                : "null";
        logFloorDiag(
                "event=map_display_floor_selected"
                        + " floor=" + resolvedFloor
                        + " source=" + source
                        + " trustedSensorFloor=" + trustedSensorFloor
                        + " currentMapFloor=" + currentMapFloor
                        + " lastRenderedFloor=" + lastRenderedFloor
        );
        return resolvedFloor;
    }

    @Nullable
    private Integer resolveBestKnownFloorForDisplay() {
        if (sensorFusion == null) {
            return null;
        }
        Integer resolvedFloor = FloorDisplayGate.resolveTrustedFloorForDisplay(
                sensorFusion.isFloorCalibrated(),
                sensorFusion.getUserVisibleFloor()
        );
        logFloorDiag(
                "event=resolve_best_known_floor_for_display"
                        + " floor=" + resolvedFloor
                        + " rawFloor=" + sensorFusion.getUserVisibleFloor()
                        + " floorCalibrated=" + sensorFusion.isFloorCalibrated()
                        + " source=" + (resolvedFloor == null ? "null" : "trusted_sensor_floor")
        );
        return resolvedFloor;
    }

    @Nullable
    static Integer resolveIncomingDisplayFloor(
            @Nullable Integer trustedFloor,
            @Nullable Integer currentMapFloor,
            @Nullable Integer lastRenderedFloor
    ) {
        Integer resolvedFloor = FloorDisplayGate.resolveDisplayFloor(
                trustedFloor,
                currentMapFloor,
                lastRenderedFloor
        );
        String source = trustedFloor != null
                ? "trusted_sensor_floor"
                : currentMapFloor != null
                ? "current_map_floor"
                : lastRenderedFloor != null
                ? "last_rendered_floor"
                : "null";
        logFloorDiag(
                "event=resolve_incoming_display_floor"
                        + " floor=" + resolvedFloor
                        + " source=" + source
                        + " trustedFloor=" + trustedFloor
                        + " currentMapFloor=" + currentMapFloor
                        + " lastRenderedFloor=" + lastRenderedFloor
        );
        return resolvedFloor;
    }

    private static void logFloorDiag(@NonNull String message) {
        try {
            Log.d(FLOOR_DIAG_TAG, message);
        } catch (RuntimeException ignored) {
        }
    }

    private void maybeApplyPendingManualFloorRecalibration() {
        if (sensorFusion == null || indoorMapManager == null) {
            return;
        }
        Integer candidateFloor = indoorMapManager.getCurrentFloor();
        boolean floorCalibrated = sensorFusion.isFloorCalibrated();
        boolean shouldRecalibrate = FloorDisplayGate.shouldAllowBaselineRecalibration(
                floorCalibrated,
                true,
                candidateFloor
        );
        sensorFusion.recalibrateAbsoluteFloorBaselineIfTrusted(
                candidateFloor,
                shouldRecalibrate,
                System.currentTimeMillis()
        );
    }

    @Nullable
    private Integer resolveTrustedVenueLabelFloor() {
        if (sensorFusion == null) {
            return null;
        }
        return FloorDisplayGate.resolveTrustedFloorForVenueLabel(
                sensorFusion.isFloorCalibrated(),
                sensorFusion.getUserVisibleFloor()
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
        MapMatchingStateResolver.MapMatchingUiState mapState = getMapMatchingUiState();
        if (mapEmptyStateCard != null) {
            boolean showEmptyState = venueSelectionEnabled && !hasDisplayPose;
            mapEmptyStateCard.setVisibility(showEmptyState ? View.VISIBLE : View.GONE);
        }
        if (mapEmptyStateText != null) {
            int textRes = R.string.map_waiting_state;
            if (mapState == MapMatchingStateResolver.MapMatchingUiState.PENDING) {
                textRes = R.string.map_constraints_pending_state;
            } else if (mapState == MapMatchingStateResolver.MapMatchingUiState.DISPLAY_ONLY) {
                textRes = R.string.map_constraints_display_only_state;
            } else if (mapState == MapMatchingStateResolver.MapMatchingUiState.UNAVAILABLE) {
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
        MapMatchingStateResolver.MapMatchingUiState mapState = getMapMatchingUiState();
        if (mapState == MapMatchingStateResolver.MapMatchingUiState.WAITING_FOR_ABSOLUTE_FIX) {
            selectedVenueText.setText(getString(R.string.venue_waiting_fix));
            return;
        }
        if (mapState == MapMatchingStateResolver.MapMatchingUiState.PENDING) {
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
        Integer floorNumber = resolveTrustedVenueLabelFloor();
        if (floorNumber == null) {
            selectedVenueText.setText(getString(R.string.venue_selected_no_floor, venueName));
            return;
        }
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
        if (sensorFusion == null) {
            return pendingIndoorMapAnchor;
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
        int iconSizePx = dpToPx(38);
        Bitmap arrowBitmap = UtilFunctions.getBitmapFromVector(
                requireContext(),
                R.drawable.ic_fused_pointer_arrow
        );
        Bitmap scaledArrowBitmap = Bitmap.createScaledBitmap(
                arrowBitmap,
                iconSizePx,
                iconSizePx,
                true
        );
        fusedMarkerIcon = BitmapDescriptorFactory.fromBitmap(scaledArrowBitmap);
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

final class DisplayFloorResetGate {

    private DisplayFloorResetGate() {
    }

    static boolean requiresResetToken(int lastRenderedFloor, int candidateFloor) {
        return lastRenderedFloor != Integer.MIN_VALUE && candidateFloor != lastRenderedFloor;
    }

    static boolean shouldHoldCrossFloorUpdate(
            int lastRenderedFloor,
            int candidateFloor,
            boolean legalFloorReset
    ) {
        return requiresResetToken(lastRenderedFloor, candidateFloor) && !legalFloorReset;
    }
}
