package com.openpositioning.PositionMe.presentation.activity;

import android.os.Bundle;
import android.util.Log;
import java.io.File;

import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;

import com.openpositioning.PositionMe.R;
import com.openpositioning.PositionMe.presentation.fragment.ReplayFragment;


/**
 * The ReplayActivity is responsible for managing the replay session of a user's trajectory.
 * It handles the process of retrieving the trajectory data and launching the replay UI directly.
 * <p>
 * The activity starts by extracting the trajectory file path from the intent that launched it. If
 * the file path is not provided or is empty, it uses a default file path. It ensures that the trajectory
 * file exists before proceeding and then switches directly to {@link ReplayFragment}, where replay
 * initialization is derived from the file contents with no user selection.
 * <p>
 * The activity also provides functionality to finish the replay session and exit the activity once the replay
 * process has completed.
 * <p>
 * This activity makes use of a key constant for passing the trajectory file path into the replay fragment.
 * <p>
 * The ReplayActivity manages the transition into the replay fragment.
 *
 * @see ReplayFragment The fragment responsible for showing the trajectory replay.
 *
 * @author Shu Gu
 */

public class ReplayActivity extends AppCompatActivity {

    public static final String TAG = "ReplayActivity";
    public static final String EXTRA_TRAJECTORY_FILE_PATH = "extra_trajectory_file_path";

    private String filePath;

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_replay);
        // Get the trajectory file path from the Intent
        filePath = getIntent().getStringExtra(EXTRA_TRAJECTORY_FILE_PATH);

        // Debug log: Received file path
        Log.i(TAG, "Received trajectory file path: " + filePath);

        if (filePath == null || filePath.isEmpty()) {
            // If not provided, set a default path (or show an error message)
            filePath = "/storage/emulated/0/Download/trajectory_default.txt";
            Log.e(TAG, "No trajectory file path provided, using default: " + filePath);
        }

        // Check if file exists before proceeding
        if (!new File(filePath).exists()) {
            Log.e(TAG, "Trajectory file does NOT exist: " + filePath);
        } else {
            Log.i(TAG, "Trajectory file exists: " + filePath);
        }

        // Formal replay flow is fully automatic; origin selection comes from the trajectory file.
        if (savedInstanceState == null) {
            showReplayFragment(filePath);
        }
    }

    /**
     * Display ReplayFragment, passing the trajectory file path as an argument.
     */
    public void showReplayFragment(String filePath) {
        Log.d(TAG, "Switching to ReplayFragment with file: " + filePath);

        ReplayFragment replayFragment = new ReplayFragment();
        Bundle args = new Bundle();
        args.putString(EXTRA_TRAJECTORY_FILE_PATH, filePath);
        replayFragment.setArguments(args);

        getSupportFragmentManager()
                .beginTransaction()
                .replace(R.id.replayActivityContainer, replayFragment)
                .commit();
    }

    /**
     * Legacy compatibility hook. Manual replay start selection is ignored in the formal flow.
     */
    public void onStartLocationChosen(float lat, float lon) {
        Log.w(TAG, "Legacy manual replay start ignored. Using automatic replay initialization.");
        showReplayFragment(filePath);
    }

    /**
     * Finish replay session
     * Called when the replay process is completed.
     */
    public void finishFlow() {
        Log.d(TAG, "Replay session finished.");
        finish();
    }
}
