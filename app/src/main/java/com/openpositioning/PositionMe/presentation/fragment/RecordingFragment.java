package com.openpositioning.PositionMe.presentation.fragment;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.content.SharedPreferences;
import android.graphics.Color;
import android.os.Bundle;
import android.os.CountDownTimer;
import android.os.Handler;
import android.os.SystemClock;
import android.text.TextUtils;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;

import android.view.animation.AlphaAnimation;
import android.view.animation.Animation;
import android.view.animation.LinearInterpolator;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.ProgressBar;
import android.widget.TextView;
import android.widget.Toast;
import com.google.android.material.button.MaterialButton;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.core.content.ContextCompat;
import androidx.fragment.app.Fragment;
import androidx.preference.PreferenceManager;

import com.openpositioning.PositionMe.BuildConfig;
import com.openpositioning.PositionMe.R;
import com.openpositioning.PositionMe.presentation.activity.RecordingActivity;
import com.openpositioning.PositionMe.sensors.FusedPose;
import com.openpositioning.PositionMe.sensors.SensorFusion;
import com.openpositioning.PositionMe.sensors.SensorTypes;
import com.openpositioning.PositionMe.utils.IndoorMapManager;
import com.openpositioning.PositionMe.utils.UtilFunctions;
import com.google.android.gms.maps.model.LatLng;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * Fragment responsible for managing the recording process of trajectory data.
 * <p>
 * The RecordingFragment serves as the interface for users to initiate, monitor, and
 * complete trajectory recording. It integrates sensor fusion data to track user movement
 * and updates a map view in real time. Additionally, it provides UI controls to cancel,
 * stop, and monitor recording progress.
 * <p>
 * Features:
 * - Starts and stops trajectory recording.
 * - Displays real-time sensor data such as elevation and distance traveled.
 * - Provides UI controls to cancel or complete recording.
 * - Uses {@link TrajectoryMapFragment} to visualize recorded paths.
 * - Manages GNSS tracking and error display.
 *
 * @see TrajectoryMapFragment The map fragment displaying the recorded trajectory.
 * @see RecordingActivity The activity managing the recording workflow.
 * @see SensorFusion Handles sensor data collection.
 * @see SensorTypes Enumeration of available sensor types.
 *
 * @author Shu Gu
 */

public class RecordingFragment extends Fragment {
    private static final String ARROW_DBG_TAG = "ARROW_DBG";
    private static final String FLOOR_DIAG_TAG = "FloorDiag";
    private static final String POSE_DIAG_TAG = "POSE_DIAG";
    private static final long ARROW_DBG_INTERVAL_MS = 500L;
    private static final long UI_REFRESH_INTERVAL_MS = 50L;
    private static final float ORIENTATION_UPDATE_THRESHOLD_DEG = 1.0f;

    // UI elements
    private MaterialButton completeButton, cancelButton, addMarkerButton;
    private ImageView recIcon;
    private ProgressBar timeRemaining;
    private TextView elevation, distanceTravelled, gnssError, floorStatus,
            systemStatus, lastUpdateTime, trackingConfidence, trackingContextHint;

    // Marker data  elements
    private final List<MarkerPoint> markerPoints = new ArrayList<>();
    private int markerIndex = 0;
    private long recordingStartElapsedMs;
    private static class MarkerPoint {
        final int index;
        final long tMs;
        final LatLng pos;

        MarkerPoint(int index, long tMs, LatLng pos) {
            this.index = index;
            this.tMs = tMs;
            this.pos = pos;
        }
    }



    // App settings
    private SharedPreferences settings;

    // Sensor & data logic
    private SensorFusion sensorFusion;
    private Handler refreshDataHandler;
    private CountDownTimer autoStop;
    private long headingDbgUiLastLogMs = 0;
    // 地图箭头链路最小日志节流。
    private long arrowDbgUiLastLogMs = 0;
    private long lastRenderedFusedPoseTimestampMs = Long.MIN_VALUE;
    private float lastForwardedOrientationDeg = Float.NaN;
    private long lastObservedHeadingSampleTimestampMs = Long.MIN_VALUE;
    private String lastUiPoseDiagnosticState = "";

    static int resolveMapUpdateFloor(
            @Nullable Integer preferredDisplayFloor,
            @NonNull FusedPose fusedPose
    ) {
        return preferredDisplayFloor != null ? preferredDisplayFloor : fusedPose.getFloor();
    }

    // Distance tracking
    private float distance = 0f;
    private double previousLocalX = 0.0;
    private double previousLocalY = 0.0;
    private final SimpleDateFormat updateTimeFormat = new SimpleDateFormat("HH:mm:ss", Locale.getDefault());

    // References to the child map fragment
    private TrajectoryMapFragment trajectoryMapFragment;

    private final Runnable refreshDataTask = new Runnable() {
        @Override
        public void run() {
            updateUIandPosition();
            // Loop again
            refreshDataHandler.postDelayed(refreshDataTask, UI_REFRESH_INTERVAL_MS);
        }
    };

    public RecordingFragment() {
        // Required empty public constructor
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        this.sensorFusion = SensorFusion.getInstance();
        Context context = requireActivity();
        this.settings = PreferenceManager.getDefaultSharedPreferences(context);
        this.refreshDataHandler = new Handler();

    }

    @Nullable
    @Override
    public View onCreateView(@NonNull LayoutInflater inflater,
                             @Nullable ViewGroup container,
                             @Nullable Bundle savedInstanceState) {
        // Inflate only the "recording" UI parts (no map)
        return inflater.inflate(R.layout.fragment_recording, container, false);
    }

    @Override
    public void onViewCreated(@NonNull View view,
                              @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);

        // 录制界面保持屏幕常亮，防止录制被系统息屏打断
        view.setKeepScreenOn(true);

        // Child Fragment: the container in fragment_recording.xml
        // where TrajectoryMapFragment is placed
        trajectoryMapFragment = (TrajectoryMapFragment)
                getChildFragmentManager().findFragmentById(R.id.trajectoryMapFragmentContainer);

        // If not present, create it
        if (trajectoryMapFragment == null) {
            trajectoryMapFragment = new TrajectoryMapFragment();
            getChildFragmentManager()
                    .beginTransaction()
                    .replace(R.id.trajectoryMapFragmentContainer, trajectoryMapFragment)
                    .commit();
        }

        // Initialize UI references
        elevation = view.findViewById(R.id.currentElevation);
        distanceTravelled = view.findViewById(R.id.currentDistanceTraveled);
        gnssError = view.findViewById(R.id.gnssError);
        floorStatus = view.findViewById(R.id.currentFloorStatus);
        systemStatus = view.findViewById(R.id.systemStatus);
        lastUpdateTime = view.findViewById(R.id.lastUpdateTime);
        trackingConfidence = view.findViewById(R.id.trackingConfidence);
        trackingContextHint = view.findViewById(R.id.trackingContextHint);

        resetRecordingSessionState();

        addMarkerButton = view.findViewById(R.id.addMarkerButton);
        addMarkerButton.setEnabled(true);

// 同步清空地图上的 TP marker（如果地图已经起来）
        if (trajectoryMapFragment != null) {
            trajectoryMapFragment.clearMapAndReset();
            trajectoryMapFragment.clearTestPointMarkers();
        }



        completeButton = view.findViewById(R.id.stopButton);
        cancelButton = view.findViewById(R.id.cancelButton);
        recIcon = view.findViewById(R.id.redDot);
        timeRemaining = view.findViewById(R.id.timeRemainingBar);

        // Hide or initialize default values
        gnssError.setVisibility(View.GONE);
        elevation.setText(getString(R.string.elevation, "0"));
        distanceTravelled.setText(getString(R.string.travelled_distance_value, "0"));
        distanceTravelled.setVisibility(View.GONE);
        floorStatus.setText(getString(R.string.floor_status_unknown));
        systemStatus.setText(getString(R.string.system_status_default));
        lastUpdateTime.setText(getString(R.string.last_update_default));
        trackingConfidence.setText(getString(R.string.tracking_confidence_unknown));
        trackingContextHint.setVisibility(View.GONE);

        // Buttons
        completeButton.setOnClickListener(v -> {
            // Stop recording & go to correction
            if (autoStop != null) autoStop.cancel();
            sensorFusion.stopRecording();
            sensorFusion.stopListening();
            // Show Correction screen
            ((RecordingActivity) requireActivity()).showCorrectionScreen();
        });


        // Cancel button with confirmation dialog
        cancelButton.setOnClickListener(v -> {
            AlertDialog dialog = new AlertDialog.Builder(requireActivity())
                    .setTitle("Confirm Cancel")
                    .setMessage("Are you sure you want to cancel the recording? Your progress will be lost permanently!")
                    .setNegativeButton("Yes", (dialogInterface, which) -> {
                        // User confirmed cancellation
                        sensorFusion.stopRecording();
                        sensorFusion.stopListening();
                        if (autoStop != null) autoStop.cancel();
                        requireActivity().onBackPressed();
                    })
                    .setPositiveButton("No", (dialogInterface, which) -> {
                        // User cancelled the dialog. Do nothing.
                        dialogInterface.dismiss();
                    })
                    .create(); // Create the dialog but do not show it yet

            // Show the dialog and change the button color
            dialog.setOnShowListener(dialogInterface -> {
                Button negativeButton = dialog.getButton(AlertDialog.BUTTON_NEGATIVE);
                negativeButton.setTextColor(Color.RED); // Set "Yes" button color to red
            });

            dialog.show(); // Finally, show the dialog
        });

        // MarkerButton event
        addMarkerButton.setOnClickListener(v -> {
            long tMs = SystemClock.elapsedRealtime() - recordingStartElapsedMs;
            int nextIndex = markerIndex + 1;

            // Prefer GNSS; fall back to current fused/PDR map location; then start location.
            LatLng pos = sensorFusion.getCurrentGnssLatLng();
            if (pos == null && trajectoryMapFragment != null) {
                pos = trajectoryMapFragment.getCurrentLocation();
            }
            if (pos == null) {
                float[] start = sensorFusion.getGNSSLatitude(true);
                if (start != null && start.length >= 2) {
                    pos = new LatLng(start[0], start[1]);
                }
            }

            boolean hasValidPos = pos != null && !(pos.latitude == 0 && pos.longitude == 0);

            if (hasValidPos) {
                double altitude = sensorFusion.getCurrentGnssAltitude();
                boolean saved = sensorFusion.addTestPoint(pos, altitude, nextIndex);
                if (!saved) {
                    Toast.makeText(requireContext(), "Marker not saved to trajectory", Toast.LENGTH_SHORT).show();
                    return;
                }
                markerIndex = nextIndex;
                markerPoints.add(new MarkerPoint(markerIndex, tMs, pos));
                if (trajectoryMapFragment != null) {
                    trajectoryMapFragment.addTestPointMarker(pos, markerIndex);
                }
            } else {
                Toast.makeText(requireContext(), "No location yet — marker not saved to trajectory", Toast.LENGTH_SHORT).show();
                return;
            }

            Log.d("MARKER", "TP" + markerIndex + " tMs=" + tMs + " pos=" + pos);
            Toast.makeText(requireContext(),
                    "Marker TP" + markerIndex + " @" + (tMs / 1000.0) + "s",
                    Toast.LENGTH_SHORT).show();
        });



        // The blinking effect for recIcon
        blinkingRecordingIcon();

        // Start the timed or indefinite UI refresh
        if (this.settings.getBoolean("split_trajectory", false)) {
            // A maximum recording time is set
            long limit = this.settings.getInt("split_duration", 30) * 60000L;
            timeRemaining.setMax((int) (limit / 1000));
            timeRemaining.setProgress(0);
            timeRemaining.setScaleY(3f);

            autoStop = new CountDownTimer(limit, 1000) {
                @Override
                public void onTick(long millisUntilFinished) {
                    timeRemaining.incrementProgressBy(1);
                }

                @Override
                public void onFinish() {
                    sensorFusion.stopRecording();
                    sensorFusion.stopListening();
                    ((RecordingActivity) requireActivity()).showCorrectionScreen();
                }
            }.start();
        }
        scheduleRefreshLoop();
    }

    /**
     * Update the UI with sensor data and pass map updates to TrajectoryMapFragment.
     */
    private void updateUIandPosition() {
        LatLng mapAnchor = getBestAvailableAbsoluteAnchor();
        if (trajectoryMapFragment != null) {
            trajectoryMapFragment.primeIndoorMapContext(mapAnchor);
        }
        SensorFusion.MotionDebugSnapshot motionDebug = sensorFusion.getMotionDebugSnapshot();

        FusedPose fusedPose = sensorFusion.getLatestFusedPose();
        updateTrackingConfidence(fusedPose);
        if (fusedPose == null) {
            floorStatus.setText(getString(R.string.floor_status_unknown));
            systemStatus.setText(getString(
                    R.string.system_status_value,
                    getString(sensorFusion.isWaitingForAbsoluteFix()
                            ? R.string.system_status_waiting_fix
                            : R.string.system_status_no_pose)
            ));
            lastUpdateTime.setText(getString(R.string.last_update_value, getString(R.string.last_update_unknown)));
            updateTrackingContextHint(null);
            distanceTravelled.setVisibility(View.GONE);
            updateObservationMarkers();
            if (sensorFusion.isWaitingForAbsoluteFix()) {
                gnssError.setVisibility(View.VISIBLE);
                gnssError.setText(getString(R.string.waiting_for_absolute_fix));
            } else {
                gnssError.setVisibility(View.GONE);
            }
            logUiPoseDiagnostic(
                    sensorFusion.isWaitingForAbsoluteFix()
                            ? "no_fused_pose_waiting_first_absolute_fix"
                            : "no_fused_pose_available",
                    motionDebug,
                    null
            );
            return;
        }
        sensorFusion.recordLatestFusedPoseIfNeeded();
        boolean hasFreshFusedPose = fusedPose.getTimestampMs() != lastRenderedFusedPoseTimestampMs;
        int bestKnownFloor = sensorFusion.getUserVisibleFloor();
        boolean floorCalibrated = sensorFusion.isFloorCalibrated();
        Integer trustedFloor = FloorDisplayGate.resolveTrustedFloorForDisplay(
                floorCalibrated,
                bestKnownFloor
        );
        double fusedDisplacementMeters = Math.hypot(
                fusedPose.getX() - previousLocalX,
                fusedPose.getY() - previousLocalY
        );
        boolean shouldSuppressDistanceIncrement = sensorFusion.isStationary()
                && fusedDisplacementMeters < MapPointerDisplayFilter.STATIONARY_VISUAL_POSITION_DEADBAND_M;

        // Distance
        if (hasFreshFusedPose && !shouldSuppressDistanceIncrement) {
            distance += fusedDisplacementMeters;
        }
        distanceTravelled.setText(getString(R.string.travelled_distance_value, String.format("%.2f", distance)));
        distanceTravelled.setVisibility(shouldShowTrackedPath() ? View.VISIBLE : View.GONE);

        // Elevation
        float elevationVal = sensorFusion.getElevation();
        elevation.setText(getString(R.string.elevation, String.format("%.1f", elevationVal)));
        String floorStatusText = resolveFloorStatusText(bestKnownFloor, floorCalibrated);
        logFloorDiag(
                "event=recording_floor_render"
                        + " floorCalibrated=" + floorCalibrated
                        + " rawFloor=" + bestKnownFloor
                        + " gateResult=" + trustedFloor
                        + " displayedText=" + floorStatusText
        );
        floorStatus.setText(floorStatusText);
        systemStatus.setText(getString(
                R.string.system_status_value,
                resolveSystemStatusLabel()
        ));
        lastUpdateTime.setText(getString(
                R.string.last_update_value,
                updateTimeFormat.format(new Date(fusedPose.getTimestampMs()))
        ));
        updateTrackingContextHint(fusedPose);

        LatLng newLocation = sensorFusion.getLatLngForFusedPose(fusedPose);
        TrajectoryMapFragment.DisplayDebugSnapshot displayDebug = trajectoryMapFragment == null
                ? null
                : trajectoryMapFragment.getDisplayDebugSnapshot();
        if (newLocation != null) {
            long headingSampleTimestampMs = sensorFusion.getDisplayOrientationTimestampMs();
            boolean hasFreshHeadingSample = headingSampleTimestampMs > 0L
                    && headingSampleTimestampMs != lastObservedHeadingSampleTimestampMs;
            float orientationDeg = (float) Math.toDegrees(sensorFusion.passDisplayOrientation());
            orientationDeg = (orientationDeg % 360f + 360f) % 360f;
            boolean hasFreshHeading = hasFreshHeadingSample && shouldForwardOrientation(orientationDeg);
            if (hasFreshHeading) {
                logArrowUiTrace(orientationDeg, !sensorFusion.isWaitingForAbsoluteFix());
                if (SensorFusion.DEBUG_HEADING) {
                    long now = SystemClock.elapsedRealtime();
                    if (now - headingDbgUiLastLogMs >= 1000) {
                        Log.d("HeadingDbg", "UI tick orientation(deg)=" + orientationDeg);
                        headingDbgUiLastLogMs = now;
                    }
                }
            }

            if (trajectoryMapFragment != null) {
                if (hasFreshFusedPose) {
                    trajectoryMapFragment.updateUserLocation(
                            newLocation,
                            orientationDeg,
                            resolveMapUpdateFloor(Integer.valueOf(bestKnownFloor), fusedPose),
                            fusedPose.getTimestampMs()
                    );
                } else if (hasFreshHeading) {
                    trajectoryMapFragment.updateUserHeading(orientationDeg);
                }
                displayDebug = trajectoryMapFragment.getDisplayDebugSnapshot();
                motionDebug = sensorFusion.getMotionDebugSnapshot();
            }
            if (hasFreshHeading) {
                lastForwardedOrientationDeg = orientationDeg;
            }
            if (hasFreshHeadingSample) {
                lastObservedHeadingSampleTimestampMs = headingSampleTimestampMs;
            }
            if (!hasFreshFusedPose) {
                logUiPoseDiagnostic("no_new_pose_arrived", motionDebug, displayDebug);
            } else if (displayDebug != null && displayDebug.poseTimestampMs == fusedPose.getTimestampMs()) {
                logUiPoseDiagnostic(
                        displayDebug.markerMoved
                                ? "displayed_marker_moved"
                                : "pose_arrived_but_marker_did_not_move",
                        motionDebug,
                        displayDebug
                );
            } else if (hasFreshFusedPose) {
                logUiPoseDiagnostic("pose_arrived_display_status_missing", motionDebug, displayDebug);
            }
        } else if (hasFreshFusedPose) {
            logUiPoseDiagnostic("fresh_pose_has_no_renderable_latlng", motionDebug, displayDebug);
        }
        if (hasFreshFusedPose) {
            lastRenderedFusedPoseTimestampMs = fusedPose.getTimestampMs();
        }

        LatLng gnssLocation = sensorFusion.getCurrentGnssLatLng();
        if (gnssLocation != null) {
            if (newLocation != null) {
                double errorDist = UtilFunctions.distanceBetweenPoints(newLocation, gnssLocation);
                gnssError.setVisibility(View.VISIBLE);
                gnssError.setText(String.format(Locale.getDefault(), "%s %.2fm",
                        getString(R.string.gnss_error), errorDist));
            }
        } else {
            gnssError.setVisibility(View.GONE);
        }
        updateObservationMarkers();

        // Update previous
        if (hasFreshFusedPose) {
            previousLocalX = fusedPose.getX();
            previousLocalY = fusedPose.getY();
        }
    }

    // 记录传给地图前的最终角度，以及当前是否已经拿到 first absolute fix。
    private void logArrowUiTrace(float orientationDeg, boolean hasFirstAbsoluteFix) {
        if (!BuildConfig.DEBUG) {
            return;
        }
        long now = SystemClock.elapsedRealtime();
        if (now - arrowDbgUiLastLogMs < ARROW_DBG_INTERVAL_MS) {
            return;
        }
        Log.d(
                ARROW_DBG_TAG,
                "stage=RecordingFragment.beforeMap"
                        + " orientationRad=" + String.format(Locale.US, "%.3f", Math.toRadians(orientationDeg))
                        + " orientationDeg=" + String.format(Locale.US, "%.3f", orientationDeg)
                        + " orientationDeg360=" + String.format(Locale.US, "%.3f", normalizeDegrees(orientationDeg))
                        + " hasFirstAbsoluteFix=" + hasFirstAbsoluteFix
        );
        arrowDbgUiLastLogMs = now;
    }

    private double normalizeDegrees(double degrees) {
        double normalized = degrees % 360.0;
        return normalized < 0.0 ? normalized + 360.0 : normalized;
    }

    private boolean shouldForwardOrientation(float orientationDeg) {
        if (!Float.isFinite(lastForwardedOrientationDeg)) {
            return true;
        }
        return absoluteShortestAngularDifferenceDeg(orientationDeg, lastForwardedOrientationDeg)
                >= ORIENTATION_UPDATE_THRESHOLD_DEG;
    }

    private void scheduleRefreshLoop() {
        refreshDataHandler.removeCallbacks(refreshDataTask);
        refreshDataHandler.postDelayed(refreshDataTask, UI_REFRESH_INTERVAL_MS);
    }

    private void resetRecordingSessionState() {
        markerPoints.clear();
        markerIndex = 0;
        recordingStartElapsedMs = SystemClock.elapsedRealtime();
        lastRenderedFusedPoseTimestampMs = Long.MIN_VALUE;
        lastForwardedOrientationDeg = Float.NaN;
        lastObservedHeadingSampleTimestampMs = Long.MIN_VALUE;
        lastUiPoseDiagnosticState = "";
        distance = 0f;
        previousLocalX = 0.0;
        previousLocalY = 0.0;
    }

    private void logUiPoseDiagnostic(
            @NonNull String state,
            @NonNull SensorFusion.MotionDebugSnapshot motionDebug,
            @Nullable TrajectoryMapFragment.DisplayDebugSnapshot displayDebug
    ) {
        if (!BuildConfig.DEBUG) {
            return;
        }
        String diagnosticState = state
                + "|poseTs=" + motionDebug.lastFusedPoseTimestampMs
                + "|displayTs=" + (displayDebug == null ? Long.MIN_VALUE : displayDebug.poseTimestampMs)
                + "|block=" + motionDebug.lastBlockReason
                + "|reject=" + resolveDebugRejectReason(motionDebug, displayDebug);
        if (diagnosticState.equals(lastUiPoseDiagnosticState)) {
            return;
        }
        lastUiPoseDiagnosticState = diagnosticState;
        Log.d(
                POSE_DIAG_TAG,
                "ui_pose state=" + state
                        + " stationary=" + motionDebug.stationary
                        + " motionResumeActive=" + motionDebug.motionResumeActive
                        + " floorConsensus=" + motionDebug.floorConsensus
                        + " floorSource=" + motionDebug.floorSource
                        + " elevatorGate=" + motionDebug.elevatorGate
                        + " currentAbsFloor=" + motionDebug.currentAbsoluteFloor
                        + " currentRelFloor=" + motionDebug.currentRelativeFloor
                        + " lastStepDecision=" + motionDebug.lastStepDecision
                        + " lastAcceptedStepTs=" + motionDebug.lastAcceptedStepTimestampMs
                        + " lastAbsDecision=" + motionDebug.lastAbsoluteFixDecision
                        + " lastAcceptedAbsTs=" + motionDebug.lastAcceptedAbsoluteFixTimestampMs
                        + " lastReject=" + resolveDebugRejectReason(motionDebug, displayDebug)
                        + " lastBlock=" + motionDebug.lastBlockReason
                        + " poseSource=" + motionDebug.lastPoseAdvanceSource
                        + " poseTs=" + motionDebug.lastFusedPoseTimestampMs
                        + " displayedTs=" + motionDebug.lastDisplayedFusedMarkerTimestampMs
                        + " displayDecision=" + (displayDebug == null ? "--" : displayDebug.lastDecision)
                        + " displayReject=" + (displayDebug == null ? "--" : displayDebug.lastRejectReason)
                        + " headingSource=" + (displayDebug == null ? "--" : displayDebug.headingSource)
                        + " headingMotionDeg=" + (displayDebug == null ? Float.NaN : displayDebug.headingMotionDeg)
                        + " headingDeviceDeg=" + (displayDebug == null ? Float.NaN : displayDebug.headingDeviceDeg)
                        + " markerHold=" + (displayDebug != null && displayDebug.markerHoldActive)
                        + " markerHoldReason=" + (displayDebug == null ? "--" : displayDebug.markerHoldReason)
                        + " markerHoldAgeMs=" + (displayDebug == null ? Long.MIN_VALUE : displayDebug.markerHoldAgeMs)
                        + " displayMoved=" + (displayDebug != null && displayDebug.markerMoved)
        );
    }

    @NonNull
    private String resolveDebugRejectReason(
            @NonNull SensorFusion.MotionDebugSnapshot motionDebug,
            @Nullable TrajectoryMapFragment.DisplayDebugSnapshot displayDebug
    ) {
        if (displayDebug != null && !TextUtils.isEmpty(displayDebug.lastRejectReason)
                && !"none".equals(displayDebug.lastRejectReason)) {
            return displayDebug.lastRejectReason;
        }
        if (!TextUtils.isEmpty(motionDebug.lastBlockReason)
                && !"none".equals(motionDebug.lastBlockReason)) {
            return motionDebug.lastBlockReason;
        }
        return motionDebug.lastRejectReason;
    }

    private float absoluteShortestAngularDifferenceDeg(float fromDeg, float toDeg) {
        return Math.abs(shortestAngularDifferenceDeg(fromDeg, toDeg));
    }

    private float shortestAngularDifferenceDeg(float fromDeg, float toDeg) {
        return (fromDeg - toDeg + 180f + 360f) % 360f - 180f;
    }

    private void logFloorDiag(@NonNull String message) {
        try {
            Log.d(FLOOR_DIAG_TAG, message);
        } catch (RuntimeException ignored) {
        }
    }

    private String resolveSystemStatusLabel() {
        boolean hasGnss = sensorFusion.getCurrentGnssLatLng() != null;
        boolean hasWifi = sensorFusion.getLatLngWifiPositioning() != null;
        if (hasGnss && hasWifi) {
            return getString(R.string.system_status_tracking_gnss_wifi);
        }
        if (hasGnss) {
            return getString(R.string.system_status_tracking_gnss);
        }
        if (hasWifi) {
            return getString(R.string.system_status_tracking_wifi);
        }
        return getString(R.string.system_status_tracking_pdr);
    }

    private boolean shouldShowTrackedPath() {
        return trajectoryMapFragment != null && trajectoryMapFragment.isMappedVenueActive();
    }

    private String resolveFloorStatusText(int floor, boolean floorCalibrated) {
        if (!FloorDisplayGate.shouldAllowUserVisibleFloor(floorCalibrated, floor)) {
            return getString(R.string.floor_status_unknown);
        }
        if (trajectoryMapFragment != null && !trajectoryMapFragment.isMappedVenueActive()) {
            return getString(R.string.floor_status_relative_value, floor);
        }
        String floorDisplayLabel = IndoorMapManager.resolveFloorDisplayLabel(
                sensorFusion.getCollectionVenue(),
                floor
        );
        if (TextUtils.isEmpty(floorDisplayLabel)) {
            // Modified: fall back to the absolute floor number when a venue label is unavailable.
            return getString(R.string.floor_status_value, floor);
        }
        return getString(R.string.floor_status_display_value, floorDisplayLabel);
    }

    private void updateTrackingConfidence(@Nullable FusedPose fusedPose) {
        if (trackingConfidence == null) {
            return;
        }
        if (fusedPose == null) {
            trackingConfidence.setText(getString(R.string.tracking_confidence_unknown));
            trackingConfidence.setTextColor(ContextCompat.getColor(requireContext(), R.color.md_theme_secondary));
            return;
        }
        int labelRes = resolveConfidenceLabel(fusedPose.getConfidence());
        trackingConfidence.setText(getString(
                R.string.tracking_confidence_value,
                getString(labelRes),
                fusedPose.getConfidence()
        ));
        trackingConfidence.setTextColor(ContextCompat.getColor(
                requireContext(),
                resolveConfidenceColor(fusedPose.getConfidence())
        ));
    }

    private void updateTrackingContextHint(@Nullable FusedPose fusedPose) {
        if (trackingContextHint == null) {
            return;
        }
        MapMatchingStateResolver.MapMatchingUiState mapState = trajectoryMapFragment == null
                ? inferMapStateWithoutFragment()
                : trajectoryMapFragment.getMapMatchingUiState();
        int hintRes;
        switch (mapState) {
            case WAITING_FOR_ABSOLUTE_FIX:
                hintRes = R.string.map_constraints_waiting_hint;
                break;
            case PENDING:
                hintRes = R.string.map_constraints_pending_hint;
                break;
            case DISPLAY_ONLY:
                hintRes = R.string.map_constraints_display_only_hint;
                break;
            case UNAVAILABLE:
                hintRes = R.string.map_constraints_unavailable_hint;
                break;
            case ACTIVE:
            default:
                trackingContextHint.setText(null);
                trackingContextHint.setVisibility(View.GONE);
                return;
        }
        trackingContextHint.setVisibility(View.VISIBLE);
        trackingContextHint.setText(getString(hintRes));
    }

    @NonNull
    private MapMatchingStateResolver.MapMatchingUiState inferMapStateWithoutFragment() {
        if (getBestAvailableAbsoluteAnchor() == null) {
            return MapMatchingStateResolver.MapMatchingUiState.WAITING_FOR_ABSOLUTE_FIX;
        }
        return MapMatchingStateResolver.MapMatchingUiState.PENDING;
    }

    private int resolveConfidenceLabel(double confidence) {
        if (confidence >= 0.65) {
            return R.string.tracking_confidence_high;
        }
        if (confidence >= 0.40) {
            return R.string.tracking_confidence_medium;
        }
        return R.string.tracking_confidence_low;
    }

    private int resolveConfidenceColor(double confidence) {
        if (confidence >= 0.65) {
            return R.color.md_theme_secondary;
        }
        if (confidence >= 0.40) {
            return R.color.md_theme_tertiary;
        }
        return R.color.md_theme_error;
    }

    @Nullable
    private LatLng getBestAvailableAbsoluteAnchor() {
        LatLng gnssLocation = sensorFusion.getCurrentGnssLatLng();
        if (gnssLocation != null) {
            return gnssLocation;
        }
        LatLng wifiLocation = sensorFusion.getLatLngWifiPositioning();
        if (wifiLocation != null) {
            return wifiLocation;
        }
        float[] start = sensorFusion.getGNSSLatitude(true);
        if (start != null && start.length >= 2 && !(start[0] == 0f && start[1] == 0f)) {
            return new LatLng(start[0], start[1]);
        }
        return null;
    }

    private void updateObservationMarkers() {
        if (trajectoryMapFragment == null) {
            return;
        }
        LatLng gnssLocation = sensorFusion.getCurrentGnssLatLng();
        if (gnssLocation != null) {
            trajectoryMapFragment.updateGNSS(gnssLocation);
        }
        LatLng wifiLocation = sensorFusion.getLatLngWifiPositioning();
        if (wifiLocation != null) {
            trajectoryMapFragment.updateWifiFix(wifiLocation, sensorFusion.getWifiFloor());
        }

        float[] pdrLocalPosition = sensorFusion.getSensorValueMap().get(SensorTypes.PDR);
        LatLng pdrLocation = sensorFusion.getLatLngForLocalPosition(pdrLocalPosition);
        if (pdrLocation != null) {
            trajectoryMapFragment.updatePdrObservation(pdrLocation);
        }
    }

    /**
     * Start the blinking effect for the recording icon.
     */
    private void blinkingRecordingIcon() {
        Animation blinking = new AlphaAnimation(1, 0);
        blinking.setDuration(800);
        blinking.setInterpolator(new LinearInterpolator());
        blinking.setRepeatCount(Animation.INFINITE);
        blinking.setRepeatMode(Animation.REVERSE);
        recIcon.startAnimation(blinking);
    }

    @Override
    public void onPause() {
        super.onPause();
        refreshDataHandler.removeCallbacks(refreshDataTask);
        if (!sensorFusion.isRecordingInProgress()) {
            sensorFusion.stopListening();
        }
    }

    @Override
    public void onResume() {
        super.onResume();
        sensorFusion.resumeListening();
        scheduleRefreshLoop();
    }


    @Override
    public void onDestroyView() {
        super.onDestroyView();
        Activity activity = getActivity();
        boolean isFinishing = activity != null && activity.isFinishing();
        boolean isChangingConfig = activity != null && activity.isChangingConfigurations();
        // 旋转等配置变化会销毁 view，但不应停止录制/扫描；仅在真正退出时收尾
        if (!isChangingConfig && (isRemoving() || isFinishing)) {
            // 先停止录制再停止监听，防止定时器残留
            sensorFusion.stopRecording();
            sensorFusion.stopListening();
        }
        // 离开录制界面后恢复屏幕常亮标志
        View root = getView();
        if (root != null) {
            root.setKeepScreenOn(false);
        }
    }
}
