package com.openpositioning.PositionMe.presentation.fragment;

import android.graphics.Color;
import android.os.Bundle;
import android.text.Editable;
import android.text.TextWatcher;
import android.view.KeyEvent;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;
import androidx.fragment.app.Fragment;

import com.openpositioning.PositionMe.R;
import com.openpositioning.PositionMe.presentation.activity.RecordingActivity;
import com.openpositioning.PositionMe.sensors.SensorFusion;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.LatLngBounds;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.model.Polyline;
import com.google.android.gms.maps.model.PolylineOptions;

import androidx.lifecycle.ViewModelProvider;
import com.openpositioning.PositionMe.viewmodels.MapViewModel;

import java.util.List;

/**
 * A simple {@link Fragment} subclass. Corrections Fragment is displayed after a recording session
 * is finished to enable manual adjustments to the PDR. The adjustments are not saved as of now.
 */
public class CorrectionFragment extends Fragment {
    private static final String START_MARKER_TITLE = "Start Position";
    private static final float SINGLE_POINT_ZOOM = 19f;
    private static final float TRAJECTORY_POLYLINE_WIDTH_PX = 10f;
    private static final int TRAJECTORY_CAMERA_PADDING_PX = 160;

    //Map variable
    public GoogleMap mMap;
    //Button to go to next
    private Button button;
    //Singleton SensorFusion class
    private final SensorFusion sensorFusion = SensorFusion.getInstance();
    private TextView averageStepLengthText;
    private EditText stepLengthInput;
    private float averageStepLength;
    private float recordedAverageStepLength;
    private CharSequence changedText;
    private float trajectoryScaleFactor = 1f;
    private Marker startMarker;
    private Polyline trajectoryPolyline;
    private MapViewModel mapViewModel;

    public CorrectionFragment() {
        // Required empty public constructor
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        AppCompatActivity activity = (AppCompatActivity) getActivity();
        if (activity != null && activity.getSupportActionBar() != null) {
            activity.getSupportActionBar().hide();
        }
        View rootView = inflater.inflate(R.layout.fragment_correction, container, false);

        // A. Initialize ViewModel to access shared data
        // Use requireActivity() to ensure that we get the instance shared with the Activity.
        mapViewModel = new ViewModelProvider(requireActivity()).get(MapViewModel.class);

        // B. Obtain the venueId from the ViewModel
        String selectedVenueId = mapViewModel.getSelectedVenueId().getValue();

        if (selectedVenueId != null) {
            // Recording 已经在 TrajectoryMapFragment 设置过 venue_id；这里仅作为兜底。
            sensorFusion.setVenueIdForTrajectory(selectedVenueId);
        }

        // Send trajectory data to the cloud
        sensorFusion.sendTrajectoryToCloud();

        initializeMapFragment();

        return rootView;
    }

    @Override
    public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);

        this.averageStepLengthText = view.findViewById(R.id.averageStepView);
        this.stepLengthInput = view.findViewById(R.id.inputStepLength);

        averageStepLength = sensorFusion.passAverageStepLength();
        recordedAverageStepLength = averageStepLength;
        updateAverageStepLengthText(averageStepLength);

        // Listen for ENTER key
        this.stepLengthInput.setOnKeyListener((v, keyCode, event) -> {
            if (keyCode == KeyEvent.KEYCODE_ENTER && event != null && event.getAction() == KeyEvent.ACTION_UP) {
                applyStepLengthCorrection();
                return true;
            }
            return false;
        });

        this.stepLengthInput.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count,int after) {}
            @Override
            public void onTextChanged(CharSequence s, int start, int before,int count) {}
            @Override
            public void afterTextChanged(Editable s) {
                changedText = s;
            }
        });

        // Button to finalize corrections
        this.button = view.findViewById(R.id.correction_done);
        this.button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                ((RecordingActivity) requireActivity()).finishFlow();
            }
        });
    }

    @Override
    public void onDestroyView() {
        super.onDestroyView();
        button = null;
        averageStepLengthText = null ;
        stepLengthInput = null;
        startMarker = null;
        trajectoryPolyline = null;
        mMap = null;
    }

    private void initializeMapFragment() {
        SupportMapFragment supportMapFragment = (SupportMapFragment)
                getChildFragmentManager().findFragmentById(R.id.map);
        if (supportMapFragment == null) {
            return;
        }
        supportMapFragment.getMapAsync(map -> {
            mMap = map;
            configureMapUi(map);
            renderRecordedTrajectory(true);
        });
    }

    private void configureMapUi(@NonNull GoogleMap map) {
        map.setMapType(GoogleMap.MAP_TYPE_HYBRID);
        map.getUiSettings().setCompassEnabled(true);
        map.getUiSettings().setTiltGesturesEnabled(true);
        map.getUiSettings().setRotateGesturesEnabled(true);
        map.getUiSettings().setScrollGesturesEnabled(true);
    }

    private void applyStepLengthCorrection() {
        if (changedText == null) {
            return;
        }
        String candidateStepLength = changedText.toString().trim();
        if (candidateStepLength.isEmpty()) {
            return;
        }
        float parsedStepLength;
        try {
            parsedStepLength = Float.parseFloat(candidateStepLength);
        } catch (NumberFormatException ignored) {
            return;
        }
        if (parsedStepLength <= 0f) {
            return;
        }

        trajectoryScaleFactor = recordedAverageStepLength > 0f
                ? parsedStepLength / recordedAverageStepLength
                : 1f;
        averageStepLength = parsedStepLength;
        updateAverageStepLengthText(parsedStepLength);
        renderRecordedTrajectory(true);
    }

    private void updateAverageStepLengthText(float stepLengthMeters) {
        if (averageStepLengthText == null) {
            return;
        }
        averageStepLengthText.setText(getString(R.string.averageStepLgn) + ": "
                + String.format("%.2f", stepLengthMeters));
    }

    private void renderRecordedTrajectory(boolean fitCamera) {
        if (mMap == null) {
            return;
        }

        List<LatLng> trajectoryPoints = sensorFusion.getRecordedTrajectoryLatLngs(trajectoryScaleFactor);
        LatLng origin = sensorFusion.getRecordedTrajectoryOriginLatLng();
        LatLng startPoint = origin != null
                ? origin
                : (trajectoryPoints.isEmpty() ? null : trajectoryPoints.get(0));

        updateStartMarker(startPoint);
        updateTrajectoryPolyline(trajectoryPoints);

        if (fitCamera) {
            fitCameraToTrajectory(trajectoryPoints, startPoint);
        }
    }

    private void updateStartMarker(@Nullable LatLng startPoint) {
        if (mMap == null) {
            return;
        }
        if (startPoint == null) {
            if (startMarker != null) {
                startMarker.remove();
                startMarker = null;
            }
            return;
        }
        if (startMarker == null) {
            startMarker = mMap.addMarker(new MarkerOptions().position(startPoint).title(START_MARKER_TITLE));
            return;
        }
        startMarker.setPosition(startPoint);
        startMarker.setTitle(START_MARKER_TITLE);
    }

    private void updateTrajectoryPolyline(@NonNull List<LatLng> trajectoryPoints) {
        if (mMap == null) {
            return;
        }
        if (trajectoryPoints.size() < 2) {
            if (trajectoryPolyline != null) {
                trajectoryPolyline.remove();
                trajectoryPolyline = null;
            }
            return;
        }
        if (trajectoryPolyline == null) {
            trajectoryPolyline = mMap.addPolyline(new PolylineOptions()
                    .color(Color.BLUE)
                    .width(TRAJECTORY_POLYLINE_WIDTH_PX));
        }
        trajectoryPolyline.setPoints(trajectoryPoints);
    }

    private void fitCameraToTrajectory(@NonNull List<LatLng> trajectoryPoints, @Nullable LatLng fallbackPoint) {
        if (mMap == null) {
            return;
        }
        if (trajectoryPoints.isEmpty()) {
            if (fallbackPoint != null) {
                mMap.moveCamera(CameraUpdateFactory.newLatLngZoom(fallbackPoint, SINGLE_POINT_ZOOM));
            }
            return;
        }

        LatLngBounds.Builder boundsBuilder = LatLngBounds.builder();
        int pointCount = 0;
        if (fallbackPoint != null) {
            boundsBuilder.include(fallbackPoint);
            pointCount++;
        }
        for (LatLng point : trajectoryPoints) {
            boundsBuilder.include(point);
            pointCount++;
        }

        if (pointCount <= 1) {
            LatLng cameraTarget = fallbackPoint != null ? fallbackPoint : trajectoryPoints.get(0);
            mMap.moveCamera(CameraUpdateFactory.newLatLngZoom(cameraTarget, SINGLE_POINT_ZOOM));
            return;
        }

        LatLngBounds bounds = boundsBuilder.build();
        View fragmentView = getView();
        Runnable fitCameraTask = () -> {
            if (mMap != null) {
                mMap.animateCamera(CameraUpdateFactory.newLatLngBounds(
                        bounds,
                        TRAJECTORY_CAMERA_PADDING_PX
                ));
            }
        };
        if (fragmentView != null) {
            fragmentView.post(fitCameraTask);
        } else {
            fitCameraTask.run();
        }
    }
}
