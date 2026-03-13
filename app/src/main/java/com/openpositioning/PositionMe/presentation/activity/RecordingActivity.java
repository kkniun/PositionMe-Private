package com.openpositioning.PositionMe.presentation.activity;

import android.os.Bundle;
import android.view.WindowManager;

import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;
import androidx.fragment.app.FragmentTransaction;

import com.openpositioning.PositionMe.R;
import com.openpositioning.PositionMe.presentation.fragment.StartLocationFragment;
import com.openpositioning.PositionMe.presentation.fragment.RecordingFragment;
import com.openpositioning.PositionMe.presentation.fragment.CorrectionFragment;
import com.openpositioning.PositionMe.sensors.SensorFusion;


/**
 * The RecordingActivity manages the recording flow of the application, guiding the user through
 * recording and correction before finalizing the process.
 * <p>
 * This activity follows a structured workflow:
 * <ol>
 *     <li>RecordingFragment - Handles the recording process and contains a TrajectoryMapFragment.</li>
 *     <li>CorrectionFragment - Enables users to review and correct recorded data before completion.</li>
 * </ol>
 * <p>
 * The activity ensures that the screen remains on during the recording process to prevent interruptions.
 * It also provides fragment transactions for seamless navigation between different stages of the workflow.
 * <p>
 * This class is referenced in various fragments such as HomeFragment, RecordingFragment, and
 * CorrectionFragment to control navigation through the recording flow.
 *
 * @see RecordingFragment Handles data recording and map visualization.
 * @see CorrectionFragment Allows users to review and make corrections before finalizing the process.
 * @see com.openpositioning.PositionMe.R.layout#activity_recording The associated layout for this activity.
 *
 * @author ShuGu
 */

public class RecordingActivity extends AppCompatActivity {

    public static final String EXTRA_TRAJECTORY_NAME = "extra_trajectory_name";

    private SensorFusion sensorFusion;
    private String requestedTrajectoryName;

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_recording);
        this.sensorFusion = SensorFusion.getInstance();
        this.requestedTrajectoryName = getIntent().getStringExtra(EXTRA_TRAJECTORY_NAME);

        if (savedInstanceState == null) {
            startFormalRecordingSession(requestedTrajectoryName);
            showRecordingScreen();
        }

        // Keep screen on
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
    }

    @Override
    protected void onResume() {
        super.onResume();
        // Ensure sensors are re-registered when returning to recording flow
        sensorFusion.resumeListening();
    }

    private void startFormalRecordingSession(@Nullable String trajectoryName) {
        sensorFusion.clearLegacyManualStartLocation();
        sensorFusion.setCollectionVenue(null);
        sensorFusion.setTrajectoryName(trajectoryName);
        sensorFusion.startRecording();
    }

    /**
     * Legacy developer-only manual-start tool. This is intentionally not used in the default
     * recording path so manual set cannot affect the formal fusion flow.
     */
    public void showStartLocationScreen() {
        FragmentTransaction ft = getSupportFragmentManager().beginTransaction();
        ft.replace(R.id.mainFragmentContainer, new StartLocationFragment());
        ft.commit();
    }

    /**
     * Show the RecordingFragment, which contains the TrajectoryMapFragment internally.
     */
    public void showRecordingScreen() {
        FragmentTransaction ft = getSupportFragmentManager().beginTransaction();
        ft.replace(R.id.mainFragmentContainer, new RecordingFragment());
        ft.addToBackStack(null);
        ft.commit();
    }

    /**
     * Show the CorrectionFragment after the user stops recording.
     */
    public void showCorrectionScreen() {
        FragmentTransaction ft = getSupportFragmentManager().beginTransaction();
        ft.replace(R.id.mainFragmentContainer, new CorrectionFragment());
        ft.addToBackStack(null);
        ft.commit();
    }

    /**
     * Finish the Activity (or do any final steps) once corrections are done.
     */
    public void finishFlow() {
        getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        finish();
    }

    @Override
    protected void onDestroy() {
        if (sensorFusion != null && isFinishing()) {
            sensorFusion.stopListening();
        }
        super.onDestroy();
    }
}
