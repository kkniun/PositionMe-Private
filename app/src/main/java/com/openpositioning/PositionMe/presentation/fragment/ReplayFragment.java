package com.openpositioning.PositionMe.presentation.fragment;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;
import androidx.lifecycle.ViewModelProvider;

import com.bumptech.glide.Glide;
import com.bumptech.glide.request.target.CustomTarget;
import com.bumptech.glide.request.transition.Transition;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.GroundOverlay;
import com.google.android.gms.maps.model.GroundOverlayOptions;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.LatLngBounds;
import com.google.android.gms.maps.model.Polygon;
import com.google.android.gms.maps.model.PolygonOptions;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.openpositioning.PositionMe.R;
import com.openpositioning.PositionMe.data.local.TrajParser;
import com.openpositioning.PositionMe.presentation.activity.ReplayActivity;
import com.openpositioning.PositionMe.viewmodels.MapViewModel;

import java.io.File;
import java.util.ArrayList;
import java.util.List;


/**
 * Sub fragment of Replay Activity. Fragment that replays trajectory data on a map.
 * <p>
 * The ReplayFragment is responsible for visualizing and replaying trajectory data captured during
 * previous recordings. It loads trajectory data from a JSON file, updates the map with user movement,
 * and provides UI controls for playback, pause, and seek functionalities.
 * <p>
 * Features:
 * - Loads trajectory data from a file and displays it on a map.
 * - Provides playback controls including play, pause, restart, and go to end.
 * - Updates the trajectory dynamically as playback progresses.
 * - Allows users to manually seek through the recorded trajectory.
 * - Integrates with {@link TrajectoryMapFragment} for map visualization.
 *
 * @see TrajectoryMapFragment The map fragment displaying the trajectory.
 * @see ReplayActivity The activity managing the replay workflow.
 * @see TrajParser Utility class for parsing trajectory data.
 *
 * @author Shu Gu
 */
public class ReplayFragment extends Fragment {

    private static final String TAG = "ReplayFragment";

    private String filePath = "";
    private int lastIndex = -1;
    private TrajParser.ReplayInitialization replayInitialization =
            TrajParser.ReplayInitialization.unavailable(false);

    // UI Controls
    private TrajectoryMapFragment trajectoryMapFragment;
    private Button playPauseButton, restartButton, exitButton, goEndButton;
    private SeekBar playbackSeekBar;

    // Playback-related
    private final Handler playbackHandler = new Handler();
    private final long PLAYBACK_INTERVAL_MS = 500; // milliseconds
    private List<TrajParser.ReplayPoint> replayData = new ArrayList<>();
    private List<TrajParser.ReplayTestPoint> replayTestPoints = new ArrayList<>();
    private int currentIndex = 0;
    private boolean isPlaying = false;

    private MapViewModel mapViewModel;
    private List<Polygon> venuePolygons = new ArrayList<>();
    private GroundOverlay floorplanOverlay;
    private TextView replayFloorStatus;
    private TextView replayElevatorStatus;

    @Override
    public void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // Retrieve transferred data from ReplayActivity
        if (getArguments() != null) {
            filePath = getArguments().getString(ReplayActivity.EXTRA_TRAJECTORY_FILE_PATH, "");
        }

        // Log the received data
        Log.i(TAG, "ReplayFragment received data:");
        Log.i(TAG, "Trajectory file path: " + filePath);

        // Check if file exists before parsing
        File trajectoryFile = new File(filePath);
        if (!trajectoryFile.exists()) {
            Log.e(TAG, "ERROR: Trajectory file does NOT exist at: " + filePath);
            return;
        }
        if (!trajectoryFile.canRead()) {
            Log.e(TAG, "ERROR: Trajectory file exists but is NOT readable: " + filePath);
            return;
        }

        Log.i(TAG, "Trajectory file confirmed to exist and is readable.");

        replayInitialization = TrajParser.resolveReplayInitialization(filePath);
        Log.i(TAG, "Replay auto-init source: " + replayInitialization.source
                + ", useFusedPose=" + replayInitialization.useFusedPose
                + ", hasOrigin=" + replayInitialization.hasOrigin());

        // Parse the trajectory file using the auto-selected replay initialization.
        replayData = TrajParser.parseTrajectoryData(filePath, requireContext(), replayInitialization);
        replayTestPoints = TrajParser.parseTestPoints(filePath);

        // Log the number of parsed points
        if (replayData != null && !replayData.isEmpty()) {
            Log.i(TAG, "Trajectory data loaded successfully. Total points: " + replayData.size());
        } else {
            Log.e(TAG, "Failed to load trajectory data! replayData is empty or null.");
        }
    }


    @Nullable
    @Override
    public View onCreateView(@NonNull LayoutInflater inflater,
                             @Nullable ViewGroup container,
                             @Nullable Bundle savedInstanceState) {
        return inflater.inflate(R.layout.fragment_replay, container, false);
    }
    @Override
    public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);

        mapViewModel = new ViewModelProvider(this).get(MapViewModel.class);

        mapViewModel.getFloorplanResponse().observe(getViewLifecycleOwner(), response -> {
            if (response != null && trajectoryMapFragment.getMap() != null) {
                Log.d(TAG, "Floorplan response received and observed by child fragment.");
            } else {
                Log.d(TAG, "Failed to fetch floorplans or no floorplans nearby.");
            }
        });

        trajectoryMapFragment = (TrajectoryMapFragment)
                getChildFragmentManager().findFragmentById(R.id.replayMapFragmentContainer);
        if (trajectoryMapFragment == null) {
            trajectoryMapFragment = new TrajectoryMapFragment();
            getChildFragmentManager()
                    .beginTransaction()
                    .replace(R.id.replayMapFragmentContainer, trajectoryMapFragment)
                    .commit();
        }

        trajectoryMapFragment.getMapAsync(googleMap -> {
            googleMap.setOnPolygonClickListener(polygon -> {
                Log.d(TAG, "A polygon was clicked!");
                if (trajectoryMapFragment != null) {
                    trajectoryMapFragment.handlePolygonClick(polygon);
                }
            });

            renderReplayTestPoints();

            LatLng replayStartLocation = getReplayStartLocation();
            if (replayStartLocation != null) {
                setupInitialMapPosition(
                        (float) replayStartLocation.latitude,
                        (float) replayStartLocation.longitude
                );
            } else {
                Log.w(TAG, "Replay start location unavailable. Camera will remain at map default.");
            }
        });

        // Initialize UI controls
        playPauseButton = view.findViewById(R.id.playPauseButton);
        restartButton   = view.findViewById(R.id.restartButton);
        exitButton      = view.findViewById(R.id.exitButton);
        goEndButton     = view.findViewById(R.id.goEndButton);
        playbackSeekBar = view.findViewById(R.id.playbackSeekBar);
        replayFloorStatus = view.findViewById(R.id.replayFloorStatus);
        replayElevatorStatus = view.findViewById(R.id.replayElevatorStatus);
        replayFloorStatus.setText(getString(R.string.floor_status_value, 0));
        replayElevatorStatus.setText(getString(
                R.string.elevator_status_value,
                getString(R.string.elevator_inactive)
        ));

        // Set SeekBar max value based on replay data
        if (!replayData.isEmpty()) {
            playbackSeekBar.setMax(replayData.size() - 1);
        }

        playPauseButton.setOnClickListener(v -> {
            if (replayData.isEmpty()) {
                Log.w(TAG, "Play/Pause button pressed but replayData is empty.");
                return;
            }
            if (isPlaying) {
                isPlaying = false;
                playPauseButton.setText("Play");
                Log.i(TAG, "Playback paused at index: " + currentIndex);
            } else {
                isPlaying = true;
                playPauseButton.setText("Pause");
                Log.i(TAG, "Playback started from index: " + currentIndex);
                if (currentIndex >= replayData.size()) {
                    currentIndex = 0;
                }
                playbackHandler.post(playbackRunnable);
            }
        });

        restartButton.setOnClickListener(v -> {
            if (replayData.isEmpty()) return;
            currentIndex = 0;
            playbackSeekBar.setProgress(0);
            Log.i(TAG, "Restart button pressed. Resetting playback to index 0.");
            updateMapForIndex(0);
        });

        goEndButton.setOnClickListener(v -> {
            if (replayData.isEmpty()) return;
            currentIndex = replayData.size() - 1;
            playbackSeekBar.setProgress(currentIndex);
            Log.i(TAG, "Go to End button pressed. Moving to last index: " + currentIndex);
            updateMapForIndex(currentIndex);
            isPlaying = false;
            playPauseButton.setText("Play");
        });

        exitButton.setOnClickListener(v -> {
            Log.i(TAG, "Exit button pressed. Exiting replay.");
            if (getActivity() instanceof ReplayActivity) {
                ((ReplayActivity) getActivity()).finishFlow();
            } else {
                requireActivity().getOnBackPressedDispatcher().onBackPressed();
            }
        });

        playbackSeekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                if (fromUser) {
                    Log.i(TAG, "SeekBar moved by user. New index: " + progress);
                    currentIndex = progress;
                    updateMapForIndex(currentIndex);
                }
            }
            @Override public void onStartTrackingTouch(SeekBar seekBar) {}
            @Override public void onStopTrackingTouch(SeekBar seekBar) {}
        });

        if (!replayData.isEmpty()) {
            updateMapForIndex(0);
        }
    }

    private void setupInitialMapPosition(float latitude, float longitude) {
        LatLng startPoint = new LatLng(latitude, longitude);
        Log.i(TAG, "Setting initial map position: " + startPoint.toString());
        trajectoryMapFragment.setInitialCameraPosition(startPoint);
    }

    /**
     * Replay camera priority:
     * 1) parsed replay origin (fused_pose or first absolute fix)
     * 2) first reconstructed track point if available
     */
    @Nullable
    private LatLng getReplayStartLocation() {
        if (replayInitialization.origin != null) {
            return replayInitialization.origin;
        }
        for (TrajParser.ReplayPoint point : replayData) {
            if (point.trackLocation != null) {
                return point.trackLocation;
            }
        }
        return null;
    }


    /**
     * Runnable for playback of trajectory data.
     * This runnable is called repeatedly to update the map with the next point in the replayData list.
     */
    private final Runnable playbackRunnable = new Runnable() {
        @Override
        public void run() {
            if (!isPlaying || replayData.isEmpty()) return;

            Log.i(TAG, "Playing index: " + currentIndex);
            updateMapForIndex(currentIndex);
            currentIndex++;
            playbackSeekBar.setProgress(currentIndex);

            if (currentIndex < replayData.size()) {
                playbackHandler.postDelayed(this, PLAYBACK_INTERVAL_MS);
            } else {
                Log.i(TAG, "Playback completed. Reached end of data.");
                isPlaying = false;
                playPauseButton.setText("Play");
            }
        }
    };


    /**
     * Update the map with the user location and GNSS location (if available) for the given index.
     * Clears the map and redraws up to the given index.
     *
     * @param newIndex
     */
    private void updateMapForIndex(int newIndex) {
        if (newIndex < 0 || newIndex >= replayData.size()) return;

        // Detect if user is playing sequentially (lastIndex + 1)
        // or is skipping around (backwards, or jump forward)
        boolean isSequentialForward = (newIndex == lastIndex + 1);

        if (!isSequentialForward) {
            // Clear everything and redraw up to newIndex
            trajectoryMapFragment.clearMapAndReset();
            for (int i = 0; i <= newIndex; i++) {
                TrajParser.ReplayPoint p = replayData.get(i);
                trajectoryMapFragment.updateUserLocation(p.trackLocation, p.orientation);
                trajectoryMapFragment.syncDisplayedFloor(p.floor);
                if (p.gnssLocation != null) {
                    trajectoryMapFragment.updateGNSS(p.gnssLocation);
                }
            }
        } else {
            // Normal sequential forward step: add just the new point
            TrajParser.ReplayPoint p = replayData.get(newIndex);
            trajectoryMapFragment.updateUserLocation(p.trackLocation, p.orientation);
            trajectoryMapFragment.syncDisplayedFloor(p.floor);
            if (p.gnssLocation != null) {
                trajectoryMapFragment.updateGNSS(p.gnssLocation);
            }
        }

        TrajParser.ReplayPoint currentPoint = replayData.get(newIndex);
        replayFloorStatus.setText(getString(R.string.floor_status_value, currentPoint.floor));
        replayElevatorStatus.setText(getString(
                R.string.elevator_status_value,
                getString(currentPoint.elevator ? R.string.elevator_active : R.string.elevator_inactive)
        ));

        lastIndex = newIndex;
    }

    private void renderReplayTestPoints() {
        if (trajectoryMapFragment == null) {
            return;
        }
        trajectoryMapFragment.clearTestPointMarkers();
        for (TrajParser.ReplayTestPoint point : replayTestPoints) {
            if (point == null || point.position == null) {
                continue;
            }
            trajectoryMapFragment.addTestPointMarker(point.position, point.index);
        }
    }

    @Override
    public void onPause() {
        super.onPause();
        isPlaying = false;
        playbackHandler.removeCallbacks(playbackRunnable);
    }

    @Override
    public void onDestroyView() {
        super.onDestroyView();
        playbackHandler.removeCallbacks(playbackRunnable);
    }

    /**
     * Draws venue outlines on the map based on the API response.
     *
     * @param apiResponse The JSON object received from the floorplan API.
     */
    private void drawVenueOutlines(JsonObject apiResponse) {
        for (Polygon p : venuePolygons) {
            p.remove();
        }
        venuePolygons.clear();

        JsonArray venues = apiResponse.getAsJsonArray("venues");
        if (venues == null || trajectoryMapFragment.getMap() == null) return;

        for (JsonElement venueElement : venues) {
            JsonObject venue = venueElement.getAsJsonObject();
            JsonArray outlineCoords = venue.getAsJsonObject("outline").getAsJsonArray("coordinates").get(0).getAsJsonArray();

            PolygonOptions polygonOptions = new PolygonOptions()
                    .strokeColor(Color.BLUE)
                    .strokeWidth(5)
                    .fillColor(Color.argb(50, 0, 0, 255))
                    .clickable(true);

            for (JsonElement coordElement : outlineCoords) {
                JsonArray lngLat = coordElement.getAsJsonArray();
                polygonOptions.add(new LatLng(lngLat.get(1).getAsDouble(), lngLat.get(0).getAsDouble()));
            }

            Polygon polygon = trajectoryMapFragment.getMap().addPolygon(polygonOptions);
            polygon.setTag(venue);
            venuePolygons.add(polygon);
        }
    }

    /**
     * Updates the selected venue and displays the first available floorplan.
     *
     * @param venueData The JSON data attached to the selected polygon.
     */
    private void selectVenue(JsonObject venueData) {
        String venueId = venueData.get("id").getAsString();
        Log.d(TAG, "Venue selected: " + venueId);

        mapViewModel.setSelectedVenueId(venueId);

        JsonArray floorplans = venueData.getAsJsonArray("floorplans");
        if (floorplans != null && floorplans.size() > 0) {
            JsonObject firstFloor = floorplans.get(0).getAsJsonObject();
            displayFloorplan(firstFloor);
        }
    }

    /**
     * Displays a floorplan image as a ground overlay.
     *
     * @param floorplan The floor definition containing the image URL and bounding box.
     */
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
                        if (trajectoryMapFragment.getMap() != null) {
                            GroundOverlayOptions options = new GroundOverlayOptions()
                                    .image(BitmapDescriptorFactory.fromBitmap(resource))
                                    .positionFromBounds(bounds);
                            floorplanOverlay = trajectoryMapFragment.getMap().addGroundOverlay(options);
                        }
                    }

                    @Override
                    public void onLoadCleared(@Nullable Drawable placeholder) {
                    }
                });
    }
}
