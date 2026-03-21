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

import java.util.ArrayList;
import java.util.List;

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
    private TextView elevation, distanceTravelled, gnssError, floorStatus, elevatorStatus;

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

    // Distance tracking
    private float distance = 0f;
    private double previousLocalX = 0.0;
    private double previousLocalY = 0.0;

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
        elevatorStatus = view.findViewById(R.id.elevatorStatus);

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
        distanceTravelled.setText(getString(R.string.meter, "0"));
        floorStatus.setText(getString(R.string.floor_status_value, 0));
        elevatorStatus.setText(getString(R.string.elevator_status_value, getString(R.string.elevator_inactive)));

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
        distanceTravelled.setText(getString(R.string.meter, String.format("%.2f", distance)));

        // Elevation
        float elevationVal = sensorFusion.getElevation();
        elevation.setText(getString(R.string.elevation, String.format("%.1f", elevationVal)));
        floorStatus.setText(getString(R.string.floor_status_value, fusedPose.getFloor()));
        elevatorStatus.setText(getString(
                R.string.elevator_status_value,
                getString(sensorFusion.getElevator() ? R.string.elevator_active : R.string.elevator_inactive)
        ));

        LatLng newLocation = sensorFusion.getLatLngForFusedPose(fusedPose);
        if (newLocation != null) {
            double orientationDeg = Math.toDegrees(sensorFusion.passOrientation());
            if (SensorFusion.DEBUG_HEADING) {
                long now = SystemClock.elapsedRealtime();
                if (now - headingDbgUiLastLogMs >= 1000) {
                    Log.d("HeadingDbg", "UI tick orientation(deg)=" + orientationDeg);
                    headingDbgUiLastLogMs = now;
                }
            }

            if (trajectoryMapFragment != null) {
                trajectoryMapFragment.updateUserLocation(newLocation, (float) orientationDeg);
                if (trajectoryMapFragment.isAutoFloorEnabled()) {
                    trajectoryMapFragment.syncDisplayedFloor(fusedPose.getFloor());
                }
            }
        }

        // GNSS logic if you want to show GNSS error, etc.
        float[] gnss = sensorFusion.getSensorValueMap().get(SensorTypes.GNSSLATLONG);
        if (gnss != null && trajectoryMapFragment != null) {
            // If user toggles showing GNSS in the map, call e.g.
            if (trajectoryMapFragment.isGnssEnabled()) {
                LatLng gnssLocation = new LatLng(gnss[0], gnss[1]);
                LatLng currentLoc = trajectoryMapFragment.getCurrentLocation();
                if (currentLoc != null) {
                    double errorDist = UtilFunctions.distanceBetweenPoints(currentLoc, gnssLocation);
                    gnssError.setVisibility(View.VISIBLE);
                    gnssError.setText(String.format(getString(R.string.gnss_error) + "%.2fm", errorDist));
                }
                trajectoryMapFragment.updateGNSS(gnssLocation);
            } else {
                gnssError.setVisibility(View.GONE);
                trajectoryMapFragment.clearGNSS();
            }
        }

        if (trajectoryMapFragment != null) {
            LatLng wifiLocation = sensorFusion.getLatLngWifiPositioning();
            if (wifiLocation != null) {
                trajectoryMapFragment.updateWifiFix(wifiLocation, sensorFusion.getWifiFloor());
            } else {
                trajectoryMapFragment.clearWifiFix();
            }
        }

        // Update previous
        previousLocalX = fusedPose.getX();
        previousLocalY = fusedPose.getY();
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
