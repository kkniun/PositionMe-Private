package com.openpositioning.PositionMe.presentation.fragment;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.os.SystemClock;
import android.text.TextUtils;
import android.util.Log;
import android.util.TypedValue;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
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
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.android.material.floatingactionbutton.FloatingActionButton;
import com.google.android.material.switchmaterial.SwitchMaterial;
import com.openpositioning.PositionMe.R;
import com.openpositioning.PositionMe.presentation.activity.RecordingActivity;
import com.openpositioning.PositionMe.sensors.SensorFusion;
import com.openpositioning.PositionMe.utils.IndoorMapManager;
import com.openpositioning.PositionMe.utils.UtilFunctions;
import com.openpositioning.PositionMe.viewmodels.MapViewModel;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class TrajectoryMapFragment extends Fragment {
    private GoogleMap gMap;
    private LatLng currentLocation;
    private Marker orientationMarker;
    private Marker gnssMarker;
    private Polyline polyline;
    private Polyline gnssPolyline;
    private LatLng lastGnssLocation;
    private LatLng pendingCameraPosition;
    private boolean hasPendingCameraMove;
    private boolean isRed = true;
    private boolean isGnssOn;
    private boolean venueSelectionEnabled;

    private SensorFusion sensorFusion;
    private long headingDbgMapLastLogMs = 0;
    private IndoorMapManager indoorMapManager;

    private Spinner switchMapSpinner;
    private SwitchMaterial gnssSwitch;
    private SwitchMaterial autoFloorSwitch;
    private FloatingActionButton floorUpButton;
    private FloatingActionButton floorDownButton;
    private Button switchColorButton;
    private TextView selectedVenueText;
    private Polygon buildingPolygon;

    private MapViewModel mapViewModel;
    private List<Polygon> venuePolygons = new ArrayList<>();
    private GroundOverlay floorplanOverlay;
    private OnMapReadyCallback mapReadyCallback;

    // Marker
    private final List<Marker> testPointMarkers = new ArrayList<>();
    private final Map<Integer, BitmapDescriptor> testPointIconCache = new HashMap<>();



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
        gnssSwitch = view.findViewById(R.id.gnssSwitch);
        autoFloorSwitch = view.findViewById(R.id.autoFloor);
        floorUpButton = view.findViewById(R.id.floorUpButton);
        floorDownButton = view.findViewById(R.id.floorDownButton);
        switchColorButton = view.findViewById(R.id.lineColorButton);
        selectedVenueText = view.findViewById(R.id.selectedVenueText);

        setFloorControlsVisibility(View.GONE);
        if (!venueSelectionEnabled && selectedVenueText != null) {
            selectedVenueText.setVisibility(View.GONE);
        }

        SupportMapFragment mapFragment =
                (SupportMapFragment) getChildFragmentManager().findFragmentById(R.id.trajectoryMap);
        if (mapFragment != null) {
            mapFragment.getMapAsync(new OnMapReadyCallback() {
                @Override
                public void onMapReady(@NonNull GoogleMap googleMap) {
                    gMap = googleMap;
                    initMapSettings(gMap);
                    if (hasPendingCameraMove && pendingCameraPosition != null) {
                        gMap.moveCamera(CameraUpdateFactory.newLatLngZoom(pendingCameraPosition, 19f));
                        hasPendingCameraMove = false;
                        pendingCameraPosition = null;
                    }
                    if (venueSelectionEnabled) {
                        gMap.setOnPolygonClickListener(polygon -> {
                            if (indoorMapManager != null && indoorMapManager.onVenuePolygonClicked(polygon)) {
                                updateVenueLabel();
                                setFloorControlsVisibility(indoorMapManager.getIsIndoorMapSet() ? View.VISIBLE : View.GONE);
                            }
                        });
                    }

                    Log.d("TrajectoryMapFragment", "onMapReady: Map is ready!");

                    if (mapReadyCallback != null) {
                        mapReadyCallback.onMapReady(gMap);
                    }
                }
            });
        }

        initMapTypeSpinner();

        gnssSwitch.setOnCheckedChangeListener((buttonView, isChecked) -> {
            isGnssOn = isChecked;
            if (!isChecked) {
                clearGNSS();
            }
        });

        switchColorButton.setOnClickListener(v -> {
            if (polyline == null) {
                return;
            }
            if (isRed) {
                switchColorButton.setBackgroundColor(Color.BLACK);
                polyline.setColor(Color.BLACK);
            } else {
                switchColorButton.setBackgroundColor(Color.RED);
                polyline.setColor(Color.RED);
            }
            isRed = !isRed;
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

        polyline = map.addPolyline(new PolylineOptions().color(Color.RED).width(5f).add());
        gnssPolyline = map.addPolyline(new PolylineOptions().color(Color.BLUE).width(5f).add());
        updateVenueLabel();
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
        if (gMap == null) {
            return;
        }

        LatLng oldLocation = currentLocation;
        currentLocation = newLocation;

        if (orientationMarker == null) {
            orientationMarker = gMap.addMarker(new MarkerOptions()
                    .position(newLocation)
                    .flat(true)
                    .title("Current Position")
                    .icon(BitmapDescriptorFactory.fromBitmap(
                            UtilFunctions.getBitmapFromVector(requireContext(), R.drawable.ic_baseline_navigation_24)))
            );
            Log.d("HeadingDbg", "marker created hash=" + orientationMarker.hashCode());
            gMap.moveCamera(CameraUpdateFactory.newLatLngZoom(newLocation, 19f));
        } else {
            orientationMarker.setPosition(newLocation);
            if (SensorFusion.DEBUG_HEADING) {
                long now = SystemClock.elapsedRealtime();
                if (now - headingDbgMapLastLogMs >= 1000) {
                    Log.d("HeadingDbg", "map rotate=" + orientation);
                    headingDbgMapLastLogMs = now;
                }
            }
            Log.d("HeadingDbg", "will setRotation=" + orientation + " markerNull?" + (orientationMarker==null));
            orientationMarker.setRotation(orientation);
            gMap.moveCamera(CameraUpdateFactory.newLatLng(newLocation));
        }

        if (oldLocation != null && !oldLocation.equals(newLocation) && polyline != null) {
            List<LatLng> points = new ArrayList<>(polyline.getPoints());
            points.add(newLocation);
            polyline.setPoints(points);
        }

        if (indoorMapManager != null) {
            indoorMapManager.setCurrentLocation(newLocation);
            indoorMapManager.refreshNearbyVenues(newLocation, sensorFusion.getWifiList());
            setFloorControlsVisibility(indoorMapManager.getIsIndoorMapSet() ? View.VISIBLE : View.GONE);
            updateVenueLabel();
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

    public void setInitialCameraPosition(@NonNull LatLng startLocation) {
        if (gMap != null) {
            gMap.moveCamera(CameraUpdateFactory.newLatLngZoom(startLocation, 19f));
        } else {
            pendingCameraPosition = startLocation;
            hasPendingCameraMove = true;
        }
    }

    public LatLng getCurrentLocation() {
        return currentLocation;
    }

    public void updateGNSS(@NonNull LatLng gnssLocation) {
        if (gMap == null || !isGnssOn) {
            return;
        }

        if (gnssMarker == null) {
            gnssMarker = gMap.addMarker(new MarkerOptions()
                    .position(gnssLocation)
                    .title("GNSS Position")
                    .icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_AZURE)));
            lastGnssLocation = gnssLocation;
            return;
        }

        gnssMarker.setPosition(gnssLocation);
        if (lastGnssLocation != null && !lastGnssLocation.equals(gnssLocation) && gnssPolyline != null) {
            List<LatLng> points = new ArrayList<>(gnssPolyline.getPoints());
            points.add(gnssLocation);
            gnssPolyline.setPoints(points);
        }
        lastGnssLocation = gnssLocation;
    }

    public void clearGNSS() {
        if (gnssMarker != null) {
            gnssMarker.remove();
            gnssMarker = null;
        }
        lastGnssLocation = null;
    }

    public boolean isGnssEnabled() {
        return isGnssOn;
    }

    private void setFloorControlsVisibility(int visibility) {
        int finalVisibility = venueSelectionEnabled ? visibility : View.GONE;
        floorUpButton.setVisibility(finalVisibility);
        floorDownButton.setVisibility(finalVisibility);
        autoFloorSwitch.setVisibility(finalVisibility);
    }

    public void clearMapAndReset() {
        if (polyline != null) {
            polyline.remove();
            polyline = null;
        }
        if (gnssPolyline != null) {
            gnssPolyline.remove();
            gnssPolyline = null;
        }
        if (orientationMarker != null) {
            orientationMarker.remove();
            orientationMarker = null;
        }
        clearGNSS();
        currentLocation = null;

        if (gMap != null) {
            polyline = gMap.addPolyline(new PolylineOptions().color(Color.RED).width(5f).add());
            gnssPolyline = gMap.addPolyline(new PolylineOptions().color(Color.BLUE).width(5f).add());
        }
    }

    /**
     * Registers a callback that is invoked when the map instance is available.
     *
     * @param callback The callback to be invoked when the map is ready.
     */
    public void getMapAsync(OnMapReadyCallback callback) {
        if (gMap != null) {
            callback.onMapReady(gMap);
        } else {
            this.mapReadyCallback = callback;
        }
    }

    /**
     * Returns the current map instance.
     *
     * @return The GoogleMap object, or null if it's not ready.
     */
    public GoogleMap getMap() {
        return gMap;
    }

    public void addTestPointMarker(@NonNull LatLng pos, int idx) {
        if (gMap == null) return;

        Marker m = gMap.addMarker(new MarkerOptions()
                .position(pos)
                .title("TP " + idx)
                .anchor(0.5f, 0.5f)
                .icon(getNumberedTestPointIcon(idx)));

        if (m != null) testPointMarkers.add(m);
    }

    public void clearTestPointMarkers() {
        for (Marker m : testPointMarkers) {
            if (m != null) m.remove();
        }
        testPointMarkers.clear();
    }

    public void syncDisplayedFloor(int floor) {
        if (indoorMapManager == null || !indoorMapManager.getIsIndoorMapSet()) {
            return;
        }
        indoorMapManager.setCurrentFloor(floor, true);
        updateVenueLabel();
    }

    private BitmapDescriptor getNumberedTestPointIcon(int idx) {
        BitmapDescriptor cached = testPointIconCache.get(idx);
        if (cached != null) {
            return cached;
        }

        int sizePx = (int) TypedValue.applyDimension(
                TypedValue.COMPLEX_UNIT_DIP,
                36,
                getResources().getDisplayMetrics()
        );

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


    /**
     * Draws venue outlines from the floorplan API response.
     *
     * @param apiResponse The JSON object received from the floorplan API.
     */
    private void drawVenueOutlines(JsonObject apiResponse) {
        for (Polygon p : venuePolygons) {
            p.remove();
        }
        venuePolygons.clear();

        JsonArray venues = apiResponse.getAsJsonArray("venues");
        if (venues == null || gMap == null) return;

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

            Polygon polygon = gMap.addPolygon(polygonOptions);
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
        Log.d("TrajectoryMapFragment", "Venue selected: " + venueId);

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
                        if (gMap != null) {
                            GroundOverlayOptions options = new GroundOverlayOptions()
                                    .image(BitmapDescriptorFactory.fromBitmap(resource))
                                    .positionFromBounds(bounds);
                            floorplanOverlay = gMap.addGroundOverlay(options);
                        }
                    }

                    @Override
                    public void onLoadCleared(@Nullable Drawable placeholder) {
                    }
                });


    }

    /**
     * Dispatches polygon click events to the venue selection handler.
     *
     * @param polygon The polygon that was clicked.
     */
    public void handlePolygonClick(Polygon polygon) {
        JsonObject venueData = (JsonObject) polygon.getTag();
        if (venueData != null) {
            selectVenue(venueData);
        } else {
            Log.w("TrajectoryMapFragment", "Clicked a polygon with no venue data attached.");
        }
    }
}
