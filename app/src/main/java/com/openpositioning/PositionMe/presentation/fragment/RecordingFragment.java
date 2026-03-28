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
import androidx.fragment.app.Fragment;
import androidx.preference.PreferenceManager;

import com.openpositioning.PositionMe.R;
import com.openpositioning.PositionMe.presentation.activity.RecordingActivity;
import com.openpositioning.PositionMe.sensors.FusedPose;
import com.openpositioning.PositionMe.sensors.SensorFusion;
import com.openpositioning.PositionMe.sensors.SensorTypes;
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
    // UI elements
    private MaterialButton completeButton, cancelButton, addMarkerButton;
    private ImageView recIcon;
    private ProgressBar timeRemaining;
    private TextView elevation, distanceTravelled, gnssError, floorStatus, elevatorStatus, systemStatus, lastUpdateTime, trackingContextHint;

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
    private static final long AUTO_FLOOR_STABLE_MS = 1200L;
    private static final long INITIAL_TRAJECTORY_DRAW_DELAY_MS = 5000L;
    private Integer pendingAutoFloorSemantic = null;
    private long pendingAutoFloorSinceMs = 0L;

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
            refreshDataHandler.postDelayed(refreshDataTask, 200);
        }
    };
    private final Runnable enableTrajectoryDrawingTask = new Runnable() {
        @Override
        public void run() {
            if (trajectoryMapFragment != null) {
                trajectoryMapFragment.setFusedTrajectoryDrawingEnabled(true, true);
            }
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
        if (trajectoryMapFragment != null) {
            trajectoryMapFragment.setFusedTrajectoryDrawingEnabled(false, true);
            refreshDataHandler.removeCallbacks(enableTrajectoryDrawingTask);
            refreshDataHandler.postDelayed(enableTrajectoryDrawingTask, INITIAL_TRAJECTORY_DRAW_DELAY_MS);
        }

        // Initialize UI references
        elevation = view.findViewById(R.id.currentElevation);
        distanceTravelled = view.findViewById(R.id.currentDistanceTraveled);
        gnssError = view.findViewById(R.id.gnssError);
        floorStatus = view.findViewById(R.id.currentFloorStatus);
        elevatorStatus = view.findViewById(R.id.elevatorStatus);
        systemStatus = view.findViewById(R.id.systemStatus);
        lastUpdateTime = view.findViewById(R.id.lastUpdateTime);
        trackingContextHint = view.findViewById(R.id.trackingContextHint);

        // Marker button and data
        markerPoints.clear();
        markerIndex = 0;
        // Marker button and data (new recording session)
        markerPoints.clear();
        markerIndex = 0;
        recordingStartElapsedMs = SystemClock.elapsedRealtime();

        addMarkerButton = view.findViewById(R.id.addMarkerButton);
        addMarkerButton.setEnabled(true);

// 同步清空地图上的 TP marker（如果地图已经起来）
        if (trajectoryMapFragment != null) {
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
        elevatorStatus.setText(getString(R.string.elevator_status_unknown));
        systemStatus.setText(getString(R.string.system_status_default));
        lastUpdateTime.setText(getString(R.string.last_update_default));
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
                    updateUIandPosition();
                }

                @Override
                public void onFinish() {
                    sensorFusion.stopRecording();
                    sensorFusion.stopListening();
                    ((RecordingActivity) requireActivity()).showCorrectionScreen();
                }
            }.start();
        } else {
            // No set time limit, just keep refreshing
            refreshDataHandler.post(refreshDataTask);
        }
    }

    /**
     * Update the UI with sensor data and pass map updates to TrajectoryMapFragment.
     */
    private void updateUIandPosition() {
        FusedPose fusedPose = sensorFusion.getLatestFusedPose();
        if (fusedPose == null) {
            floorStatus.setText(getString(R.string.floor_status_unknown));
            elevatorStatus.setText(getString(R.string.elevator_status_unknown));
            systemStatus.setText(getString(
                    R.string.system_status_value,
                    getString(sensorFusion.isWaitingForAbsoluteFix()
                            ? R.string.system_status_waiting_fix
                            : R.string.system_status_no_pose)
            ));
            lastUpdateTime.setText(getString(R.string.last_update_value, getString(R.string.last_update_unknown)));
            trackingContextHint.setVisibility(View.GONE);
            distanceTravelled.setVisibility(View.GONE);
            if (sensorFusion.isWaitingForAbsoluteFix()) {
                gnssError.setVisibility(View.VISIBLE);
                gnssError.setText(getString(R.string.waiting_for_absolute_fix));
            } else {
                gnssError.setVisibility(View.GONE);
            }
            return;
        }
        sensorFusion.recordLatestFusedPoseIfNeeded();

        // Distance
        distance += Math.sqrt(
                Math.pow(fusedPose.getX() - previousLocalX, 2)
                        + Math.pow(fusedPose.getY() - previousLocalY, 2)
        );
        distanceTravelled.setText(getString(R.string.travelled_distance_value, String.format("%.2f", distance)));
        distanceTravelled.setVisibility(shouldShowTrackedPath() ? View.VISIBLE : View.GONE);

        // Elevation
        float elevationVal = sensorFusion.getElevation();
        elevation.setText(getString(R.string.elevation, String.format("%.1f", elevationVal)));
        floorStatus.setText(resolveFloorStatusText(fusedPose.getFloor()));
        elevatorStatus.setText(resolveElevatorStatusText());
        systemStatus.setText(getString(
                R.string.system_status_value,
                resolveSystemStatusLabel()
        ));
        lastUpdateTime.setText(getString(
                R.string.last_update_value,
                updateTimeFormat.format(new Date(fusedPose.getTimestampMs()))
        ));
        updateTrackingContextHint();

        LatLng newLocation = sensorFusion.getLatLngForFusedPose(fusedPose);
        if (newLocation != null) {
            double orientationDeg = normalizeHeadingDeg(Math.toDegrees(sensorFusion.getMapHeadingRad()));
            if (SensorFusion.DEBUG_HEADING) {
                long now = SystemClock.elapsedRealtime();
                if (now - headingDbgUiLastLogMs >= 1000) {
                    Log.d("HeadingDbg", "UI tick orientation(deg)=" + orientationDeg);
                    headingDbgUiLastLogMs = now;
                }
            }

            if (trajectoryMapFragment != null) {
                trajectoryMapFragment.updateUserLocation(
                        newLocation,
                        (float) orientationDeg,
                        fusedPose.getTimestampMs()
                );
            }
        }
        if (trajectoryMapFragment != null && trajectoryMapFragment.isAutoFloorEnabled()) {
            maybeSyncDisplayedFloorFromWifi();
        }

        LatLng gnssLocation = sensorFusion.getCurrentGnssLatLng();
        if (gnssLocation != null) {
            if (newLocation != null) {
                double errorDist = UtilFunctions.distanceBetweenPoints(newLocation, gnssLocation);
                gnssError.setVisibility(View.VISIBLE);
                gnssError.setText(String.format(Locale.getDefault(), "%s %.2fm",
                        getString(R.string.gnss_error), errorDist));
            }
            if (trajectoryMapFragment != null) {
                trajectoryMapFragment.updateGNSS(gnssLocation);
            }
        } else {
            gnssError.setVisibility(View.GONE);
        }

        if (trajectoryMapFragment != null) {
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

        // Update previous
        previousLocalX = fusedPose.getX();
        previousLocalY = fusedPose.getY();
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

    private String resolveFloorStatusText(int floor) {
        if (trajectoryMapFragment != null && !trajectoryMapFragment.isMappedVenueActive()) {
            return getString(R.string.floor_status_relative_value, floor);
        }
        if (trajectoryMapFragment != null) {
            String displayedLabel = trajectoryMapFragment.getCurrentDisplayedFloorLabel();
            if (displayedLabel != null && !displayedLabel.isEmpty()) {
                return getString(R.string.floor_status_label_value, displayedLabel);
            }
            String floorLabel = trajectoryMapFragment.getFloorDisplayLabelFor(floor);
            if (floorLabel != null && !floorLabel.isEmpty()) {
                return getString(R.string.floor_status_label_value, floorLabel);
            }
        }
        return getString(R.string.floor_status_value, floor);
    }

    private void maybeSyncDisplayedFloorFromWifi() {
        if (trajectoryMapFragment == null || !trajectoryMapFragment.isMappedVenueActive()) {
            return;
        }
        Integer candidateFloor = resolveAutoFloorCandidate();
        if (candidateFloor == null) {
            pendingAutoFloorSemantic = null;
            return;
        }
        Integer currentSemanticFloor = trajectoryMapFragment.getCurrentDisplayedFloorSemanticLevel();
        if (currentSemanticFloor != null && currentSemanticFloor == candidateFloor) {
            pendingAutoFloorSemantic = null;
            return;
        }

        long now = SystemClock.elapsedRealtime();
        if (pendingAutoFloorSemantic == null || pendingAutoFloorSemantic != candidateFloor) {
            pendingAutoFloorSemantic = candidateFloor;
            pendingAutoFloorSinceMs = now;
            return;
        }
        if (now - pendingAutoFloorSinceMs >= AUTO_FLOOR_STABLE_MS) {
            trajectoryMapFragment.syncDisplayedFloor(candidateFloor);
            pendingAutoFloorSemantic = null;
        }
    }

    @Nullable
    private Integer resolveAutoFloorCandidate() {
        if (sensorFusion.getLatLngWifiPositioning() != null) {
            return sensorFusion.getWifiFloor();
        }
        FusedPose fusedPose = sensorFusion.getLatestFusedPose();
        if (fusedPose != null) {
            return fusedPose.getFloor();
        }
        return null;
    }

    private float normalizeHeadingDeg(double headingDeg) {
        double normalized = headingDeg % 360.0;
        if (normalized < 0.0) {
            normalized += 360.0;
        }
        return (float) normalized;
    }

    private String resolveElevatorStatusText() {
        if (trajectoryMapFragment != null && !trajectoryMapFragment.isMappedVenueActive()) {
            return getString(R.string.elevator_status_unknown);
        }
        return getString(
                R.string.elevator_status_value,
                getString(sensorFusion.getElevator() ? R.string.elevator_active : R.string.elevator_inactive)
        );
    }

    private void updateTrackingContextHint() {
        if (trackingContextHint == null) {
            return;
        }
        boolean hasAbsoluteTracking = sensorFusion.getCurrentGnssLatLng() != null
                || sensorFusion.getLatLngWifiPositioning() != null;
        boolean outsideMappedVenue = trajectoryMapFragment != null && !trajectoryMapFragment.isMappedVenueActive();
        boolean showHint = hasAbsoluteTracking && outsideMappedVenue;
        trackingContextHint.setVisibility(showHint ? View.VISIBLE : View.GONE);
        if (showHint) {
            trackingContextHint.setText(getString(R.string.tracking_context_fallback));
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
        refreshDataHandler.removeCallbacks(enableTrajectoryDrawingTask);
        sensorFusion.stopListening();
    }

    @Override
    public void onResume() {
        super.onResume();
        sensorFusion.resumeListening();
        if(!this.settings.getBoolean("split_trajectory", false)) {
            refreshDataHandler.postDelayed(refreshDataTask, 500);
        }
    }


    @Override
    public void onDestroyView() {
        super.onDestroyView();
        refreshDataHandler.removeCallbacks(enableTrajectoryDrawingTask);
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
