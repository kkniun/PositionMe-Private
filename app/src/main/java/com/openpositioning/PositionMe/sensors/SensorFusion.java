package com.openpositioning.PositionMe.sensors;

import android.content.Context;
import android.content.SharedPreferences;
import android.app.ActivityManager;
import android.content.Intent;
import android.hardware.GeomagneticField;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationListener;
import android.os.Build;
import android.os.PowerManager;
import android.os.SystemClock;
import android.provider.Settings;
import android.util.Log;
import android.view.Surface;
import android.view.WindowManager;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.preference.PreferenceManager;

import com.google.android.gms.maps.model.LatLng;
import com.openpositioning.PositionMe.BuildConfig;
import com.openpositioning.PositionMe.presentation.activity.MainActivity;
import com.openpositioning.PositionMe.utils.BuildingPolygon;
import com.openpositioning.PositionMe.utils.CoordinateConverter;
import com.openpositioning.PositionMe.utils.MapConstraintRepository;
import com.openpositioning.PositionMe.utils.PathView;
import com.openpositioning.PositionMe.utils.PdrProcessing;
import com.openpositioning.PositionMe.utils.UtilFunctions;
import com.openpositioning.PositionMe.data.remote.ServerCommunications;
import com.openpositioning.PositionMe.Traj;
import com.openpositioning.PositionMe.presentation.fragment.SettingsFragment;

import org.json.JSONException;
import org.json.JSONObject;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.Timer;
import java.util.TimerTask;
import java.util.UUID;
import java.util.stream.Collectors;
import java.util.stream.Stream;


/**
 * The SensorFusion class is the main data gathering and processing class of the application.
 *
 * It follows the singleton design pattern to ensure that every fragment and process has access to
 * the same date and sensor instances. Hence it has a private constructor, and must be initialised
 * with the application context after creation.
 * <p>
 * The class implements {@link SensorEventListener} and has instances of {@link MovementSensor} for
 * every device type necessary for data collection. As such, it implements the
 * {@link SensorFusion#onSensorChanged(SensorEvent)} function, and process and records the data
 * provided by the sensor hardware, which are stored in a {@link Traj} object. Data is read
 * continuously but is only saved to the trajectory when recording is enabled.
 * <p>
 * The class provides a number of setters and getters so that other classes can have access to the
 * sensor data and influence the behaviour of data collection.
 *
 * @author Michal Dvorak
 * @author Mate Stodulka
 * @author Virginia Cangelosi
 */
public class SensorFusion implements SensorEventListener, Observer {
    private static final String ARROW_DBG_TAG = "ARROW_DBG";
    private static final long ARROW_DBG_INTERVAL_MS = 500L;
    private static final long DECLINATION_REFRESH_INTERVAL_MS = 10 * 60 * 1000L;
    private static final String VENUE_KEY_NUCLEUS = "nucleus";
    private static final String VENUE_KEY_LIBRARY = "library";
    private static final String VENUE_KEY_MURCHISON = "murchison";

    // Store the last event timestamps for each sensor type
    private HashMap<Integer, Long> lastEventTimestamps = new HashMap<>();
    private HashMap<Integer, Integer> eventCounts = new HashMap<>();

    long maxReportLatencyNs = 0;  // Disable batching to deliver events immediately

    // Define a threshold for large time gaps (in milliseconds)
    private static final long LARGE_GAP_THRESHOLD_MS = 500;  // Adjust this if needed

    //region Static variables
    // Singleton Class
    private static final SensorFusion sensorFusion = new SensorFusion();
    // Static constant for calculations with milliseconds
    private static final long TIME_CONST = 10;
    // Coefficient for fusing gyro-based and magnetometer-based orientation
    public static final float FILTER_COEFFICIENT = 0.96f;
    // Toggle for heading debug logs across modules
    public static final boolean DEBUG_HEADING = false;
    //Tuning value for low pass filter
    private static final float ALPHA = 0.8f;
    // String for creating WiFi fingerprint JSO N object
    private static final String WIFI_FINGERPRINT= "wf";
    private static final String DEFAULT_COLLECTION_VENUE = "traj";
    private static final float DEFAULT_WIFI_ACCURACY_M = 8.0f;
    private static final double MIN_ABSOLUTE_FIX_START_STD_M = 1.0;
    // 临时联调开关：定位 fused 内部状态与 UI 显示偏差后可删除。
    public static final boolean DEBUG_FUSION_TRACE = BuildConfig.DEBUG;
    //endregion

    //region Instance variables
    // Keep device awake while recording
    private PowerManager.WakeLock wakeLock;
    private Context appContext;

    // Settings
    private SharedPreferences settings;

    // Movement sensor instances
    private MovementSensor accelerometerSensor;
    private MovementSensor barometerSensor;
    private MovementSensor gyroscopeSensor;
    private MovementSensor lightSensor;
    private MovementSensor proximitySensor;
    private MovementSensor magnetometerSensor;
    private MovementSensor stepDetectionSensor;
    private MovementSensor rotationSensor;
    private MovementSensor gravitySensor;
    private MovementSensor linearAccelerationSensor;
    // Other data recording
    private WifiDataProcessor wifiProcessor;
    private GNSSDataProcessor gnssProcessor;
    // Data listener

    private BleDataProcessor bleProcessor;
    private List<BLE> bleList;
    // BLE values
    private Set<String> recordedBleMacs = new HashSet<>();
    private String lastBleFingerprintSignature;

    private long lastBleUiToastMs = 0;


    private final LocationListener locationListener;

    // Server communication class for sending data
    private ServerCommunications serverCommunications;
    // Trajectory object containing all data
    private Traj.Trajectory.Builder trajectory;

    // Settings
    private boolean saveRecording;
    private float filter_coefficient;
    // Variables to help with timed events
    private long absoluteStartTime;
    private long bootTime;
    long lastStepTime = 0;
    // Timer object for scheduling data recording
    private Timer storeTrajectoryTimer;
    // Counters for dividing timer to record data every 1 second/ every 5 seconds
    private int counter;
    private int secondCounter;

    // Sensor values
    private float[] acceleration;
    private float[] filteredAcc;
    private float[] gravity;
    private float[] magneticField;
    private float[] angularVelocity;
    private float[] orientation;
    // 仅用于地图箭头显示，避免影响 PDR/PF 仍在使用的原始 device azimuth。
    private float[] displayOrientation;
    private float[] rotation;
    private float pressure;
    private float light;
    private float proximity;
    private float[] R;
    // Throttling timestamp for heading debug logs (rotation vector)
    private long headingDbgRotvecLastLogMs = 0;
    private boolean hasGeomagneticDeclination = false;
    private float geomagneticDeclinationDeg = 0f;
    private long geomagneticDeclinationTimestampMs = 0L;
    private double geomagneticDeclinationLatitudeDeg = Double.NaN;
    private double geomagneticDeclinationLongitudeDeg = Double.NaN;
    private float geomagneticDeclinationAltitudeMeters = 0f;
    // 地图箭头链路最小日志节流，避免 logcat 刷屏。
    private long arrowDbgRawLastLogMs = 0;
    private long arrowDbgDisplayLastLogMs = 0;
    private int stepCounter ;
    // Derived values
    private float elevation;
    private boolean elevator;
    // Location values
    private float latitude;
    private float longitude;
    private float[] startLocation;
    // Wifi values
    private List<Wifi> wifiList;
    private Set<Long> recordedApMacs = new HashSet<>();
    private String lastFingerprintSignature;
    // 当前录制会话的轨迹标识，用于上报 Wi-Fi 指纹
    private String trajectoryId;
    // 用户可见的轨迹名称
    private String trajectoryName = "";
    // 用户选择的场地标识，默认占位符
    private String collectionVenue = DEFAULT_COLLECTION_VENUE;

    // For Marker
    private double gnssAltitude = 0.0;
    private final List<Traj.TestPoint> testPoints = new ArrayList<>();



    // Over time accelerometer magnitude values since last step
    private List<Double> accelMagnitude;

    // PDR calculation class
    private PdrProcessing pdrProcessing;
    private ParticleFilterEngine particleFilterEngine;
    private boolean pfInitialized;
    private FusedPose latestFusedPose;
    private CoordinateConverter coordinateConverter;
    private boolean listenersActive;
    private final AbsoluteFloorTransitionResolver absoluteFloorTransitionResolver = new AbsoluteFloorTransitionResolver();
    private boolean hasManualStartLocation;
    private float lastPredictHeadingRad;
    private float lastPredictElevation;
    private long lastWaitingForAbsoluteFixLogMs;
    private long lastRecordedFusedPoseTimestampMs;
    // Converts barometer/PDR relative floors into absolute map floor indices.
    private int pdrFloorOffset = 0;
    private boolean isFloorOffsetInitialized = false;
    private long lastWifiScanWallClockMs = -1L;
    @Nullable
    private LatLng lastDisplayedFusedMarkerLatLng;
    @Nullable
    private LatLng lastDisplayedFusedMarkerRawLatLng;
    private long lastDisplayedFusedMarkerTimestampMs = -1L;

    // Trajectory displaying class
    private PathView pathView;
    // WiFi positioning object
    private WiFiPositioning wiFiPositioning;

    // Convert "AA:BB:CC:DD:EE:FF" -> int64 (same idea as WiFi bssid long)
    private long macStringToLong(String mac) {
        if (mac == null) return 0L;
        // remove ":" or "-"
        String hex = mac.replace(":", "").replace("-", "");
        if (hex.isEmpty()) return 0L;
        try {
            return Long.parseUnsignedLong(hex, 16);
        } catch (Exception e) {
            return 0L;
        }
    }

    private String safeBleName(String name) {
        if (name == null) return "unknown";
        name = name.trim();
        return name.isEmpty() ? "unknown" : name;
    }


    //region Initialisation
    /**
     * Private constructor for implementing singleton design pattern for SensorFusion.
     * Initialises empty arrays and new objects that do not depends on outside information.
     */
    private SensorFusion() {
        // Location listener to be used by the GNSS class
        this.locationListener= new myLocationListener();
        // Timer to store sensor values in the trajectory object
        this.storeTrajectoryTimer = new Timer();
        // Counters to track elements with slower frequency
        this.counter = 0;
        this.secondCounter = 0;
        // Step count initial value
        this.stepCounter = 0;
        // PDR elevation initial values
        this.elevation = 0;
        this.elevator = false;
        // PDR position array
        this.startLocation = new float[2];
        // Empty array initialisation
        this.acceleration = new float[3];
        this.filteredAcc = new float[3];
        this.gravity = new float[3];
        this.magneticField = new float[3];
        this.angularVelocity = new float[3];
        this.orientation = new float[3];
        this.displayOrientation = new float[3];
        this.rotation = new float[4];
        this.rotation[3] = 1.0f;
        this.R = new float[9];
        // GNSS initial Long-Lat array
        this.startLocation = new float[2];
    }


    /**
     * Static function to access singleton instance of SensorFusion.
     *
     * @return  singleton instance of SensorFusion class.
     */
    public static SensorFusion getInstance() {
        return sensorFusion;
    }

    /**
     * Initialisation function for the SensorFusion instance.
     *
     * Initialise all Movement sensor instances from context and predetermined types. Creates a
     * server communication instance for sending trajectories. Saves current absolute and relative
     * time, and initialises saving the recording to false.
     *
     * @param context   application context for permissions and device access.
     *
     * @see MovementSensor handling all SensorManager based data collection devices.
     * @see ServerCommunications handling communication with the server.
     * @see GNSSDataProcessor for location data processing.
     * @see WifiDataProcessor for network data processing.
     */
    public void setContext(Context context) {
        this.appContext = context.getApplicationContext(); // store app context for later use

        // Initialise data collection devices (unchanged)...
        this.accelerometerSensor = new MovementSensor(context, Sensor.TYPE_ACCELEROMETER);
        this.barometerSensor = new MovementSensor(context, Sensor.TYPE_PRESSURE);
        this.gyroscopeSensor = new MovementSensor(context, Sensor.TYPE_GYROSCOPE);
        this.lightSensor = new MovementSensor(context, Sensor.TYPE_LIGHT);
        this.proximitySensor = new MovementSensor(context, Sensor.TYPE_PROXIMITY);
        this.magnetometerSensor = new MovementSensor(context, Sensor.TYPE_MAGNETIC_FIELD);
        this.stepDetectionSensor = new MovementSensor(context, Sensor.TYPE_STEP_DETECTOR);
        this.rotationSensor = new MovementSensor(context, Sensor.TYPE_ROTATION_VECTOR);
        this.gravitySensor = new MovementSensor(context, Sensor.TYPE_GRAVITY);
        this.linearAccelerationSensor = new MovementSensor(context, Sensor.TYPE_LINEAR_ACCELERATION);
        // Listener based devices
        this.wifiProcessor = new WifiDataProcessor(context);
        wifiProcessor.registerObserver(this);

        this.bleProcessor = new BleDataProcessor(context);
        bleProcessor.registerObserver(this);


        this.gnssProcessor = new GNSSDataProcessor(context, locationListener);
        // Create object handling HTTPS communication
        this.serverCommunications = new ServerCommunications(context);
        // Save absolute and relative start time
        this.absoluteStartTime = System.currentTimeMillis();
        this.bootTime = SystemClock.uptimeMillis();
        // Initialise saveRecording to false
        this.saveRecording = false;
        this.listenersActive = false;

        // Other initialisations...
        this.accelMagnitude = new ArrayList<>();
        this.pdrProcessing = new PdrProcessing(context);
        this.settings = PreferenceManager.getDefaultSharedPreferences(context);
        this.pathView = new PathView(context, null);
        this.wiFiPositioning = new WiFiPositioning(context);
        this.particleFilterEngine = createParticleFilterEngine();
        this.pfInitialized = false;
        this.latestFusedPose = null;
        this.coordinateConverter = null;
        this.hasManualStartLocation = false;
        this.lastPredictHeadingRad = 0f;
        this.lastPredictElevation = 0f;
        this.lastRecordedFusedPoseTimestampMs = -1L;

        if(settings.getBoolean("overwrite_constants", false)) {
            this.filter_coefficient = Float.parseFloat(settings.getString("accel_filter", "0.96"));
        } else {
            this.filter_coefficient = FILTER_COEFFICIENT;
        }

        // Keep app awake during the recording (using stored appContext)
        PowerManager powerManager = (PowerManager) this.appContext.getSystemService(Context.POWER_SERVICE);
        wakeLock = powerManager.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "MyApp::MyWakelockTag");
    }

    //endregion

    //region Sensor processing
    /**
     * {@inheritDoc}
     *
     * Called every time a Sensor value is updated.
     *
     * Checks originating sensor type, if the data is meaningful save it to a local variable.
     *
     * @param sensorEvent   SensorEvent of sensor with values changed, includes types and values.
     */
    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        long currentTime = System.currentTimeMillis();  // Current time in milliseconds
        int sensorType = sensorEvent.sensor.getType();

        // Get the previous timestamp for this sensor type
        Long lastTimestamp = lastEventTimestamps.get(sensorType);

        if (lastTimestamp != null) {
            long timeGap = currentTime - lastTimestamp;

//            // Log a warning if the time gap is larger than the threshold
//            if (timeGap > LARGE_GAP_THRESHOLD_MS) {
//                Log.e("SensorFusion", "Large time gap detected for sensor " + sensorType +
//                        " | Time gap: " + timeGap + " ms");
//            }
        }

        // Update timestamp and frequency counter for this sensor
        lastEventTimestamps.put(sensorType, currentTime);
        eventCounts.put(sensorType, eventCounts.getOrDefault(sensorType, 0) + 1);



        switch (sensorType) {
            case Sensor.TYPE_ACCELEROMETER:
                acceleration[0] = sensorEvent.values[0];
                acceleration[1] = sensorEvent.values[1];
                acceleration[2] = sensorEvent.values[2];
                break;

            case Sensor.TYPE_PRESSURE:
                pressure = (1 - ALPHA) * pressure + ALPHA * sensorEvent.values[0];
                if (saveRecording) {
                    this.elevation = pdrProcessing.updateElevation(
                            SensorManager.getAltitude(SensorManager.PRESSURE_STANDARD_ATMOSPHERE, pressure)
                    );
                }
                break;

            case Sensor.TYPE_GYROSCOPE:
                angularVelocity[0] = sensorEvent.values[0];
                angularVelocity[1] = sensorEvent.values[1];
                angularVelocity[2] = sensorEvent.values[2];

            case Sensor.TYPE_LINEAR_ACCELERATION:
                filteredAcc[0] = sensorEvent.values[0];
                filteredAcc[1] = sensorEvent.values[1];
                filteredAcc[2] = sensorEvent.values[2];

                // Compute magnitude & add to accelMagnitude
                double accelMagFiltered = Math.sqrt(
                        Math.pow(filteredAcc[0], 2) +
                                Math.pow(filteredAcc[1], 2) +
                                Math.pow(filteredAcc[2], 2)
                );
                this.accelMagnitude.add(accelMagFiltered);

//                // Debug logging
//                Log.v("SensorFusion",
//                        "Added new linear accel magnitude: " + accelMagFiltered
//                                + "; accelMagnitude size = " + accelMagnitude.size());

                elevator = pdrProcessing.estimateElevator(gravity, filteredAcc);
                break;

            case Sensor.TYPE_GRAVITY:
                gravity[0] = sensorEvent.values[0];
                gravity[1] = sensorEvent.values[1];
                gravity[2] = sensorEvent.values[2];

                // Possibly log gravity values if needed
                //Log.v("SensorFusion", "Gravity: " + Arrays.toString(gravity));

                elevator = pdrProcessing.estimateElevator(gravity, filteredAcc);
                break;

            case Sensor.TYPE_LIGHT:
                light = sensorEvent.values[0];
                break;

            case Sensor.TYPE_PROXIMITY:
                proximity = sensorEvent.values[0];
                break;

            case Sensor.TYPE_MAGNETIC_FIELD:
                magneticField[0] = sensorEvent.values[0];
                magneticField[1] = sensorEvent.values[1];
                magneticField[2] = sensorEvent.values[2];
                break;

            case Sensor.TYPE_ROTATION_VECTOR:
                this.rotation = sensorEvent.values.clone();
                float[] rotationVectorDCM = new float[9];
                float[] horizontalWorldDCM = new float[9];
                SensorManager.getRotationMatrixFromVector(rotationVectorDCM, this.rotation);
                if (!SensorManager.remapCoordinateSystem(
                        rotationVectorDCM,
                        SensorManager.AXIS_X,
                        SensorManager.AXIS_Z,
                        horizontalWorldDCM
                )) {
                    System.arraycopy(rotationVectorDCM, 0, horizontalWorldDCM, 0, rotationVectorDCM.length);
                }
                updateTrueNorthOrientation(horizontalWorldDCM);
                updateDisplayOrientation(horizontalWorldDCM);
                logArrowRawOrientationTrace();

                Log.d("HeadingDbg", "onSensorChanged azimuth(rad)=" + orientation[0]);

                if (DEBUG_HEADING) {
                    long now = SystemClock.elapsedRealtime();
                    if (now - headingDbgRotvecLastLogMs >= 1000) {
                        Log.d("HeadingDbg", "rotvec azimuth(rad)=" + orientation[0]);
                        headingDbgRotvecLastLogMs = now;
                    }
                }
                break;

            case Sensor.TYPE_STEP_DETECTOR:
                long stepTime = SystemClock.uptimeMillis() - bootTime;


                if (currentTime - lastStepTime < 20) {
                    Log.e("SensorFusion", "Ignoring step event, too soon after last step event:" + (currentTime - lastStepTime) + " ms");
                    // Ignore rapid successive step events
                    break;
                }

                else {
                    lastStepTime = currentTime;
                    // Log if accelMagnitude is empty
                    if (accelMagnitude.isEmpty()) {
                        Log.e("SensorFusion",
                                "stepDetection triggered, but accelMagnitude is empty! " +
                                        "This can cause updatePdr(...) to fail or return bad results.");
                    } else {
                        Log.d("SensorFusion",
                                "stepDetection triggered, accelMagnitude size = " + accelMagnitude.size());
                    }

                    float currentHeadingRad = getCurrentHeadingRad();
                    float deltaHeadingRad = normalizeHeadingDelta(currentHeadingRad - lastPredictHeadingRad);
                    float heightDeltaMeters = this.elevation - lastPredictElevation;
                    PdrDelta pdrDelta = this.pdrProcessing.buildStepDelta(
                            this.accelMagnitude,
                            deltaHeadingRad,
                            heightDeltaMeters
                    );
                    float[] newCords = this.pdrProcessing.applyStepDelta(pdrDelta, currentHeadingRad);
                    long stepEventAgeMs = Math.max(
                            0L,
                            (SystemClock.elapsedRealtimeNanos() - sensorEvent.timestamp) / 1_000_000L
                    );

                    // Clear the accelMagnitude after using it
                    this.accelMagnitude.clear();
                    handlePdrPredict(
                            pdrDelta,
                            getAbsoluteCurrentFloor(),
                            currentTime,
                            stepEventAgeMs
                    );
                    this.lastPredictHeadingRad = currentHeadingRad;
                    this.lastPredictElevation = this.elevation;


                    if (saveRecording) {
                        this.pathView.drawTrajectory(newCords);
                        stepCounter++;
                        trajectory.addPdrData(Traj.RelativePosition.newBuilder()
                                .setRelativeTimestamp(SystemClock.uptimeMillis() - bootTime)
                                .setX(newCords[0])
                                .setY(newCords[1])
                                .setFloor(getAbsoluteCurrentFloor())
                                .setElevator(this.elevator)
                                .setElevation(this.elevation));
                    }
                    break;
                }

        }
    }

    /**
     * Utility function to log the event frequency of each sensor.
     * Call this periodically for debugging purposes.
     */
    public void logSensorFrequencies() {
        for (int sensorType : eventCounts.keySet()) {
            Log.d("SensorFusion", "Sensor " + sensorType + " | Event Count: " + eventCounts.get(sensorType));
        }
    }

    /**
     * {@inheritDoc}
     *
     * Location listener class to receive updates from the location manager.
     *
     * Passed to the {@link GNSSDataProcessor} to receive the location data in this class. Save the
     * values in instance variables.
     */
    class myLocationListener implements LocationListener{
        @Override
        public void onLocationChanged(@NonNull Location location) {
            //Toast.makeText(context, "Location Changed", Toast.LENGTH_SHORT).show();
            latitude = (float) location.getLatitude();
            longitude = (float) location.getLongitude();
            float altitude = (float) location.getAltitude();
            updateGeomagneticDeclination(latitude, longitude, altitude, location.getTime());
            float accuracy = (float) location.getAccuracy();
            float speed = (float) location.getSpeed();
            String provider = location.getProvider();
            if(saveRecording) {
                long relativeTimestamp = System.currentTimeMillis() - absoluteStartTime;
                Traj.GNSSPosition.Builder position = Traj.GNSSPosition.newBuilder()
                        .setRelativeTimestamp(relativeTimestamp)
                        .setLatitude(latitude)
                        .setLongitude(longitude)
                        .setAltitude(altitude)
                        .setFloor(String.valueOf(getCurrentFloor()));

                Traj.GNSSReading.Builder gnssBuilder = Traj.GNSSReading.newBuilder()
                        .setPosition(position)
                        .setAccuracy(accuracy)
                        .setSpeed(speed)
                        .setProvider(provider);

                if (location.hasBearing()) {
                    gnssBuilder.setBearing(location.getBearing());
                }

                trajectory.addGnssData(gnssBuilder);
                long fixTimestampMs = System.currentTimeMillis();
                long observationAgeMs = resolveGnssObservationAgeMs(location, fixTimestampMs);
                handleAbsoluteFixInternal(
                        new AbsoluteFix(
                                fixTimestampMs,
                                latitude,
                                longitude,
                                accuracy
                        ),
                        resolveStep1InitializationFloor(null),
                        null,
                        "GNSS",
                        observationAgeMs
                );
            }
            gnssAltitude = location.getAltitude();

        }
    }

    /**
     * {@inheritDoc}
     *
     * Receives updates from {@link WifiDataProcessor}.
     *
     * @see WifiDataProcessor object for wifi scanning.
     */
    /*@Override
    public void update(Object[] wifiList) {
        // Save newest wifi values to local variable
        this.wifiList = Stream.of(wifiList).map(o -> (Wifi) o).collect(Collectors.toList());

        if(this.saveRecording) {
            List<Wifi> sortedWifi = new ArrayList<>(this.wifiList);
            sortedWifi.sort((a, b) -> Long.compare(a.getBssid(), b.getBssid()));
            StringBuilder signatureBuilder = new StringBuilder();
            int apCount = 0;
            for (Wifi data : sortedWifi) {
                if (data.getBssid() == 0) {
                    // BSSID=0 代表未知/解析失败，跳过避免与真实 AP 去重冲突
                    continue;
                }
                signatureBuilder.append(data.getBssid())
                        .append(':')
                        .append(data.getLevel())
                        .append(';');
                apCount++;
            }
            String fingerprintSignature = signatureBuilder.toString();
            boolean isDuplicateFingerprint = fingerprintSignature.equals(lastFingerprintSignature);
            if (isDuplicateFingerprint) {
                Log.d("WifiDedup", "Skipping duplicate fingerprint: " + fingerprintSignature + " apCount=" + apCount);
                Log.d("SensorFusion", "Skipping duplicate WiFi fingerprint: " + fingerprintSignature);
            } else {
                long sampleTimestamp = SystemClock.uptimeMillis() - bootTime;
                Traj.Fingerprint.Builder fingerprint = Traj.Fingerprint.newBuilder()
                        .setRelativeTimestamp(sampleTimestamp);

                for (Wifi data : this.wifiList) {
                    if (data.getBssid() == 0) {
                        // 无有效 BSSID 时不记录指纹/元数据，防止 0 被当成唯一键
                        continue;
                    }
                    fingerprint.addRfScans(Traj.RFScan.newBuilder()
                            .setRelativeTimestamp(sampleTimestamp)
                            .setMac(data.getBssid())
                            .setRssi(data.getLevel()));

                    if (!recordedApMacs.contains(data.getBssid())) {
                        boolean rttCapable = data.isRttSupported();
                        // 复用统一规范化逻辑，避免分支重复
                        String ssid = WifiDataProcessor.normalizeSsid(data.getSsid());
                        long frequency = WifiDataProcessor.normalizeFrequency(data.getFrequency());
                        Traj.WiFiAPData.Builder apData = Traj.WiFiAPData.newBuilder()
                                .setMac(data.getBssid())
                                .setSsid(ssid)
                                .setFrequency(frequency)
                                .setRttEnabled(rttCapable);
                        trajectory.addApsData(apData);
                        recordedApMacs.add(data.getBssid());
                        String freqLabel = (frequency == 0) ? "0(unknown)" : String.valueOf(frequency);
                        Log.d("SensorFusion", "AP data added: bssid=" + data.getBssid()
                                + " ssid=" + ssid
                                + " freq=" + freqLabel
                                + " rtt=" + rttCapable);
                    }
                }
                int rfCount = fingerprint.getRfScansCount();
                Log.d("WifiDedup", "Accepted fingerprint: " + fingerprintSignature + " apCount=" + rfCount);
                // Adding WiFi fingerprint data to Trajectory
                this.trajectory.addWifiFingerprints(fingerprint);
                Log.d("SensorFusion", "WiFi fingerprint added: count="
                        + this.trajectory.getWifiFingerprintsCount()
                        + " apCount=" + fingerprint.getRfScansCount());
                lastFingerprintSignature = fingerprintSignature;
            }
        }
        createWifiPositioningRequest();
    }*/

    @Override
    public void update(Object[] objList) {
        if (objList == null || objList.length == 0) return;

        // Case 1: WiFi update
        if (objList[0] instanceof Wifi) {
            Object[] wifiArr = objList;

            this.wifiList = Stream.of(wifiArr)
                    .map(o -> (Wifi) o)
                    .collect(Collectors.toList());
            lastWifiScanWallClockMs = System.currentTimeMillis();

            // === keep ALL your existing WiFi fingerprint code here ===
            // (the whole "if(saveRecording) { ... trajectory.addWifiFingerprints ... }"
            //  plus createWifiPositioningRequest(); )

            if (this.saveRecording) {
                List<Wifi> sortedWifi = new ArrayList<>(this.wifiList);
                sortedWifi.sort((a, b) -> Long.compare(a.getBssid(), b.getBssid()));
                StringBuilder signatureBuilder = new StringBuilder();
                for (Wifi data : sortedWifi) {
                    if (data.getBssid() == 0) {
                        continue; // 无效 MAC 不参与去重签名
                    }
                    signatureBuilder.append(data.getBssid())
                            .append(':')
                            .append(data.getLevel())
                            .append(';');
                }
                String fingerprintSignature = signatureBuilder.toString();
                boolean isDuplicateFingerprint = fingerprintSignature.equals(lastFingerprintSignature);
                if (isDuplicateFingerprint) {
                    Log.d("SensorFusion", "Skipping duplicate WiFi fingerprint: " + fingerprintSignature);
                }

                long sampleTimestamp = SystemClock.uptimeMillis() - bootTime;
                Traj.Fingerprint.Builder fingerprint = Traj.Fingerprint.newBuilder()
                        .setRelativeTimestamp(sampleTimestamp);

                if (!isDuplicateFingerprint) {
                    for (Wifi data : this.wifiList) {
                        if (data.getBssid() == 0) {
                            continue; // 过滤无效 AP
                        }

                        fingerprint.addRfScans(Traj.RFScan.newBuilder()
                                .setRelativeTimestamp(sampleTimestamp)
                                .setMac(data.getBssid())
                                .setRssi(data.getLevel()));

                        if (!recordedApMacs.contains(data.getBssid())) {
                            boolean rttCapable = data.isRttSupported();
                            String ssid = data.getSsid();
                            if (ssid == null || ssid.isEmpty() || "<unknown ssid>".equalsIgnoreCase(ssid)) {
                                ssid = "hidden";
                            } else if (ssid.length() >= 2 && ssid.startsWith("\"") && ssid.endsWith("\"")) {
                                ssid = ssid.substring(1, ssid.length() - 1);
                                if (ssid.isEmpty()) {
                                    ssid = "hidden";
                                }
                            }
                            long frequency = data.getFrequency();
                            if (frequency <= 0) frequency = 0;

                            Traj.WiFiAPData.Builder apData = Traj.WiFiAPData.newBuilder()
                                    .setMac(data.getBssid())
                                    .setSsid(ssid)
                                    .setFrequency(frequency)
                                    .setRttEnabled(rttCapable);

                            trajectory.addApsData(apData);
                            recordedApMacs.add(data.getBssid());
                        }
                    }
                    this.trajectory.addWifiFingerprints(fingerprint);
                    lastFingerprintSignature = fingerprintSignature;
                }
            }

            createWifiPositioningRequest();
            return;
        }

        // Case 2: BLE update
        if (objList[0] instanceof BLE) {

            Log.d("BLE_PIPE", "SensorFusion.update(): got BLE count=" + objList.length);

            BLE first = (BLE) objList[0];
            Log.d("BLE_PIPE", "First BLE: mac=" + first.getMac()
                    + " rssi=" + first.getRssi()
                    + " name=" + first.getName());

            // 1) 保存 bleList
            Object[] bleArr = objList;
            this.bleList = Stream.of(bleArr)
                    .map(o -> (BLE) o)
                    .collect(Collectors.toList());

            // ✅ 2) 不管是否 recording，都给 UI 一个提示（3秒一次）

            /* Sb uu y uu
            long now = SystemClock.uptimeMillis();

            if (appContext != null && (now - lastBleUiToastMs) > 3000) {
                String msg = "BLE ok: count=" + objList.length
                        + (saveRecording && trajectory != null
                        ? (" bleFp=" + trajectory.getBleFingerprintsCount()
                        + " bleData=" + trajectory.getBleDataCount())
                        : " (not recording)");
                android.widget.Toast.makeText(appContext, msg, android.widget.Toast.LENGTH_SHORT).show();
                lastBleUiToastMs = now;
            }
            */

            // 3) 只有 recording 才写入 traj
            if (this.saveRecording && trajectory != null) {

                // fingerprint signature：mac+rssi 排序拼接（防重复）
                List<BLE> sortedBle = new ArrayList<>(this.bleList);
                sortedBle.sort((a, b) -> {
                    String ma = (a.getMac() == null) ? "" : a.getMac();
                    String mb = (b.getMac() == null) ? "" : b.getMac();
                    return ma.compareTo(mb);
                });

                StringBuilder sig = new StringBuilder();
                for (BLE d : sortedBle) sig.append(d.getMac()).append(':').append(d.getRssi()).append(';');
                String bleFingerprintSignature = sig.toString();

                if (bleFingerprintSignature.equals(lastBleFingerprintSignature)) {
                    Log.d("SensorFusion", "Skipping duplicate BLE fingerprint");
                    return;
                }

                long sampleTimestamp = SystemClock.uptimeMillis() - bootTime;

                Traj.Fingerprint.Builder bleFp = Traj.Fingerprint.newBuilder()
                        .setRelativeTimestamp(sampleTimestamp);

                for (BLE d : this.bleList) {
                    long macLong = macStringToLong(d.getMac());

                    bleFp.addRfScans(
                            Traj.RFScan.newBuilder()
                                    .setRelativeTimestamp(sampleTimestamp)
                                    .setMac(macLong)
                                    .setRssi(d.getRssi())
                    );

                    String macStr = d.getMac();
                    if (macStr != null && !recordedBleMacs.contains(macStr)) {
                        Traj.BleData.Builder bleData = Traj.BleData.newBuilder()
                                .setMacAddress(macStr)
                                .setName(safeBleName(d.getName()))
                                .setTxPowerLevel(0)
                                .setAdvertiseFlags(0);

                        trajectory.addBleData(bleData);
                        recordedBleMacs.add(macStr);
                    }
                }

                trajectory.addBleFingerprints(bleFp);
                lastBleFingerprintSignature = bleFingerprintSignature;

                Log.d("BLE_PIPE",
                        "traj updated: bleFpCount=" + trajectory.getBleFingerprintsCount()
                                + ", bleDataCount=" + trajectory.getBleDataCount()
                                + ", lastFpScans=" + bleFp.getRfScansCount());
            }

            return;
        }
 
    }




    /**
     * Function to create a request to obtain a wifi location for the obtained wifi fingerprint
     *
     */
    private void createWifiPositioningRequest(){
        // Try catch block to catch any errors and prevent app crashing
        try {
            final long wifiObservationTimestampMs = lastWifiScanWallClockMs > 0L
                    ? lastWifiScanWallClockMs
                    : System.currentTimeMillis();
            // Creating a JSON object to store the WiFi access points
            JSONObject wifiAccessPoints=new JSONObject();
            for (Wifi data : this.wifiList){
                if (data.getBssid() == 0) {
                    // 0 代表未知 BSSID，过滤避免服务器误解析或键冲突
                    continue;
                }
                wifiAccessPoints.put(String.valueOf(data.getBssid()), data.getLevel());
            }
            // Creating POST Request
            JSONObject wifiFingerPrint = new JSONObject();
            wifiFingerPrint.put(WIFI_FINGERPRINT, wifiAccessPoints);
            // 绑定当前轨迹 ID；未在录制时为空则不写入，保持行为最小化
            if (trajectoryId != null && !trajectoryId.isEmpty()) {
                wifiFingerPrint.put("trajectory_id", trajectoryId);
            }
            this.wiFiPositioning.request(wifiFingerPrint, new WiFiPositioning.VolleyCallback() {
                @Override
                public void onSuccess(LatLng wifiLocation, int floor) {
                    if (wifiLocation == null) {
                        return;
                    }
                    long fixTimestampMs = System.currentTimeMillis();
                    long observationAgeMs = Math.max(0L, fixTimestampMs - wifiObservationTimestampMs);
                    handleAbsoluteFixInternal(
                            new AbsoluteFix(
                                    fixTimestampMs,
                                    wifiLocation.latitude,
                                    wifiLocation.longitude,
                                    DEFAULT_WIFI_ACCURACY_M
                            ),
                            resolveStep1InitializationFloor(floor),
                            floor,
                            "WIFI",
                            observationAgeMs
                    );
                }

                @Override
                public void onError(String message) {
                    Log.w("SensorFusion", "WiFi absolute fix failed: " + message);
                }
            });
        } catch (JSONException e) {
            // Catching error while making JSON object, to prevent crashes
            // Error log to keep record of errors (for secure programming and maintainability)
            Log.e("jsonErrors","Error creating json object"+e.toString());
        }
    }

    /**
     * Method to get user position obtained using {@link WiFiPositioning}.
     *
     * @return {@link LatLng} corresponding to user's position.
     */
    public LatLng getLatLngWifiPositioning(){return this.wiFiPositioning.getWifiLocation();}

    /**
     * Method to get current floor the user is at, obtained using WiFiPositioning
     * @see WiFiPositioning for WiFi positioning
     * @return Current floor user is at using WiFiPositioning
     */
    public int getWifiFloor(){
        return this.wiFiPositioning.getFloor();
    }

    public void handleAbsoluteFix(@NonNull AbsoluteFix absoluteFix, int floor) {
        handleAbsoluteFixInternal(
                absoluteFix.getLatitudeDeg(),
                absoluteFix.getLongitudeDeg(),
                floor,
                floor,
                absoluteFix.getTimestampMs(),
                absoluteFix.getAccuracyMeters(),
                "ABS",
                -1L
        );
    }

    public void handleAbsoluteFix(double latitudeDeg, double longitudeDeg, int floor, long timestampMs) {
        handleAbsoluteFixInternal(
                latitudeDeg,
                longitudeDeg,
                floor,
                floor,
                timestampMs,
                DEFAULT_WIFI_ACCURACY_M,
                "ABS",
                -1L
        );
    }

    public void handleAbsoluteFix(
            double latitudeDeg,
            double longitudeDeg,
            int floor,
            long timestampMs,
            float accuracyMeters
    ) {
        handleAbsoluteFixInternal(
                latitudeDeg,
                longitudeDeg,
                floor,
                floor,
                timestampMs,
                accuracyMeters,
                "ABS",
                -1L
        );
    }

    private void handleAbsoluteFixInternal(
            double latitudeDeg,
            double longitudeDeg,
            int initializationFloor,
            @Nullable Integer floorPrior,
            long timestampMs,
            float accuracyMeters,
            @NonNull String debugSource,
            long observationAgeMs
    ) {
        if (!saveRecording) {
            return;
        }

        if (this.particleFilterEngine == null) {
            this.particleFilterEngine = createParticleFilterEngine();
        }

        updateGeomagneticDeclination(latitudeDeg, longitudeDeg, 0f, timestampMs);
        CoordinateConverter converter = getOrCreateCoordinateConverter(latitudeDeg, longitudeDeg);
        if (converter == null) {
            return;
        }
        Integer acceptedFloorPrior = resolveAcceptedAbsoluteFloorPrior(latitudeDeg, longitudeDeg, floorPrior);
        maybeCalibrateFloorOffset(acceptedFloorPrior);
        double[] localFix = converter.toLocalMeters(latitudeDeg, longitudeDeg);
        maybeSetInitialPositionIfAbsent(this.trajectory, latitudeDeg, longitudeDeg);
        this.latestFusedPose = applyAbsoluteFixForStep1(
                this.particleFilterEngine,
                converter,
                this.pfInitialized,
                latitudeDeg,
                longitudeDeg,
                initializationFloor,
                acceptedFloorPrior,
                timestampMs,
                accuracyMeters,
                getCurrentHeadingRad()
        );
        this.pfInitialized = this.latestFusedPose != null;
        this.lastPredictHeadingRad = getCurrentHeadingRad();
        this.lastPredictElevation = this.elevation;
        recordLatestFusedPoseIfNeeded();
        logAbsoluteFusionTrace(
                debugSource,
                timestampMs,
                observationAgeMs,
                latitudeDeg,
                longitudeDeg,
                localFix,
                acceptedFloorPrior,
                accuracyMeters
        );
    }

    private void handleAbsoluteFixInternal(@NonNull AbsoluteFix absoluteFix, int initializationFloor,
                                           @Nullable Integer floorPrior,
                                           @NonNull String debugSource,
                                           long observationAgeMs) {
        handleAbsoluteFixInternal(
                absoluteFix.getLatitudeDeg(),
                absoluteFix.getLongitudeDeg(),
                initializationFloor,
                floorPrior,
                absoluteFix.getTimestampMs(),
                absoluteFix.getAccuracyMeters(),
                debugSource,
                observationAgeMs
        );
    }

    public void handlePdrPredict(@Nullable PdrDelta pdrDelta, int floor, long timestampMs) {
        handlePdrPredict(pdrDelta, floor, timestampMs, 0L);
    }

    public void handlePdrPredict(
            @Nullable PdrDelta pdrDelta,
            int floor,
            long timestampMs,
            long observationAgeMs
    ) {
        if (!saveRecording) {
            return;
        }

        // Formal fusion flow must wait for the first real absolute fix.
        // PDR-only prediction is ignored until GNSS/WiFi/other absolute positioning arrives.
        if (!pfInitialized || this.particleFilterEngine == null || pdrDelta == null) {
            if (saveRecording && pdrDelta != null && !pfInitialized) {
                long now = SystemClock.elapsedRealtime();
                if (now - lastWaitingForAbsoluteFixLogMs >= 3_000L) {
                    Log.i(
                            "SensorFusion",
                            "Waiting for first absolute fix; holding PDR prediction until GNSS/WiFi anchor arrives."
                    );
                    lastWaitingForAbsoluteFixLogMs = now;
                }
            }
            return;
        }

        PdrDelta constrainedDelta = getElevator()
                ? new PdrDelta(0f, pdrDelta.getDeltaHeadingRad(), pdrDelta.getHeightDeltaMeters())
                : pdrDelta;
        this.latestFusedPose = applyPdrPredictionForStep1(
                this.particleFilterEngine,
                this.pfInitialized,
                constrainedDelta,
                floor,
                timestampMs
        );
        recordLatestFusedPoseIfNeeded();
        logPdrFusionTrace(constrainedDelta, floor, timestampMs, observationAgeMs);
    }

    public FusedPose getLatestFusedPose() {
        return latestFusedPose;
    }

    public boolean isWaitingForAbsoluteFix() {
        return saveRecording && !pfInitialized;
    }

    @Nullable
    public LatLng getLatLngForFusedPose(@Nullable FusedPose fusedPose) {
        if (fusedPose == null) {
            return null;
        }
        CoordinateConverter converter = getOrCreateCoordinateConverter(latitude, longitude);
        if (converter == null) {
            return null;
        }
        return converter.toLatLng(fusedPose.getX(), fusedPose.getY());
    }

    @Nullable
    public LatLng getLatLngForLocalPosition(@Nullable float[] localPosition) {
        if (localPosition == null || localPosition.length < 2) {
            return null;
        }
        CoordinateConverter converter = getOrCreateCoordinateConverter(latitude, longitude);
        if (converter == null) {
            return null;
        }
        return converter.toLatLng(localPosition[0], localPosition[1]);
    }

    @Nullable
    public double[] getLocalMetersForLatLng(@Nullable LatLng latLng) {
        if (latLng == null || coordinateConverter == null) {
            return null;
        }
        return coordinateConverter.toLocalMeters(latLng.latitude, latLng.longitude);
    }

    // UI 侧回写当前实际显示的 fused marker，便于区分“内部 fused 错了”还是“UI 画错了”。
    public void noteDisplayedFusedMarker(
            @Nullable LatLng rawLocation,
            @Nullable LatLng displayedLocation,
            long timestampMs
    ) {
        lastDisplayedFusedMarkerRawLatLng = rawLocation;
        lastDisplayedFusedMarkerLatLng = displayedLocation;
        lastDisplayedFusedMarkerTimestampMs = timestampMs;
    }

    public void recordLatestFusedPoseIfNeeded() {
        if (!saveRecording || trajectory == null || latestFusedPose == null) {
            return;
        }
        long poseTimestampMs = latestFusedPose.getTimestampMs();
        if (poseTimestampMs <= 0 || poseTimestampMs == lastRecordedFusedPoseTimestampMs) {
            return;
        }

        long relativeTimestampMs = Math.max(0L, poseTimestampMs - absoluteStartTime);
        trajectory.addFusedPose(Traj.FusedPoseRecord.newBuilder()
                .setRelativeTimestamp(relativeTimestampMs)
                .setX(latestFusedPose.getX())
                .setY(latestFusedPose.getY())
                .setFloor(latestFusedPose.getFloor())
                .setConfidence((float) latestFusedPose.getConfidence())
                .build());
        lastRecordedFusedPoseTimestampMs = poseTimestampMs;
    }

    @Nullable
    private CoordinateConverter getOrCreateCoordinateConverter(double fallbackLatitudeDeg, double fallbackLongitudeDeg) {
        if (coordinateConverter != null) {
            return coordinateConverter;
        }
        if (hasManualStartLocation && startLocation != null && startLocation.length >= 2) {
            // Legacy developer-only manual origin. Formal recording flow clears this before start.
            coordinateConverter = new CoordinateConverter(startLocation[0], startLocation[1]);
            return coordinateConverter;
        }
        if (fallbackLatitudeDeg == 0.0 && fallbackLongitudeDeg == 0.0) {
            return null;
        }
        setTrajectoryOrigin(fallbackLatitudeDeg, fallbackLongitudeDeg, false);
        return coordinateConverter;
    }

    private void setTrajectoryOrigin(double latitudeDeg, double longitudeDeg, boolean manualOrigin) {
        this.startLocation = new float[]{(float) latitudeDeg, (float) longitudeDeg};
        this.hasManualStartLocation = manualOrigin;
        this.coordinateConverter = new CoordinateConverter(latitudeDeg, longitudeDeg);
        updateGeomagneticDeclination(latitudeDeg, longitudeDeg, 0f, System.currentTimeMillis());
    }

    private int getRelativeCurrentFloor() {
        return this.pdrProcessing == null ? 0 : this.pdrProcessing.getCurrentFloor();
    }

    /**
     * Get the current absolute floor aligned to the map floor index space.
     */
    private int getAbsoluteCurrentFloor() {
        return getRelativeCurrentFloor() + pdrFloorOffset;
    }

    private void maybeCalibrateFloorOffset(@Nullable Integer absoluteFloor) {
        if (absoluteFloor == null || this.pdrProcessing == null) {
            return;
        }
        int desiredOffset = absoluteFloor - getRelativeCurrentFloor();
        if (!isFloorOffsetInitialized || getAbsoluteCurrentFloor() != absoluteFloor) {
            this.pdrFloorOffset = desiredOffset;
            this.isFloorOffsetInitialized = true;
        }
    }

    @Nullable
    private Integer resolveAcceptedAbsoluteFloorPrior(
            double latitudeDeg,
            double longitudeDeg,
            @Nullable Integer reportedFloor
    ) {
        if (reportedFloor == null) {
            absoluteFloorTransitionResolver.reset();
            return null;
        }
        int currentFloor = latestFusedPose != null ? latestFusedPose.getFloor() : getAbsoluteCurrentFloor();
        boolean hasMapConstraints = MapConstraintRepository.hasAnyConstraints();
        boolean transitionAllowed = !hasMapConstraints
                || reportedFloor == currentFloor
                || allowsAbsoluteFixFloorTransition(
                new LatLng(latitudeDeg, longitudeDeg),
                currentFloor,
                reportedFloor
        );
        return absoluteFloorTransitionResolver.resolveAcceptedFloor(
                currentFloor,
                reportedFloor,
                pfInitialized,
                hasMapConstraints,
                transitionAllowed
        );
    }

    private boolean allowsAbsoluteFixFloorTransition(
            @NonNull LatLng absoluteFixLatLng,
            int currentFloor,
            int newFloor
    ) {
        if (newFloor == currentFloor) {
            return true;
        }
        LatLng previousLatLng = getLatLngForFusedPose(latestFusedPose);
        if (previousLatLng == null) {
            previousLatLng = absoluteFixLatLng;
        }
        if (getElevator()) {
            return MapConstraintRepository.isPointInsideLift(previousLatLng, currentFloor)
                    || MapConstraintRepository.isPointInsideLift(absoluteFixLatLng, currentFloor)
                    || MapConstraintRepository.isPointInsideLift(absoluteFixLatLng, newFloor)
                    || MapConstraintRepository.doesPathIntersectLift(previousLatLng, absoluteFixLatLng, currentFloor)
                    || MapConstraintRepository.doesPathIntersectLift(previousLatLng, absoluteFixLatLng, newFloor);
        }
        return MapConstraintRepository.isPointInsideStairs(previousLatLng, currentFloor)
                || MapConstraintRepository.isPointInsideStairs(absoluteFixLatLng, currentFloor)
                || MapConstraintRepository.isPointInsideStairs(absoluteFixLatLng, newFloor)
                || MapConstraintRepository.doesPathIntersectStairs(previousLatLng, absoluteFixLatLng, currentFloor)
                || MapConstraintRepository.doesPathIntersectStairs(previousLatLng, absoluteFixLatLng, newFloor);
    }

    private int resolveStep1InitializationFloor(@Nullable Integer preferredFloor) {
        if (preferredFloor != null) {
            return preferredFloor;
        }
        if (latestFusedPose != null) {
            return latestFusedPose.getFloor();
        }
        return getCurrentFloor();
    }

    static FusedPose applyAbsoluteFixForStep1(
            @NonNull ParticleFilterEngine particleFilterEngine,
            @NonNull CoordinateConverter coordinateConverter,
            boolean pfInitialized,
            double latitudeDeg,
            double longitudeDeg,
            int initializationFloor,
            @Nullable Integer floorPrior,
            long timestampMs,
            float accuracyMeters,
            float headingRad
    ) {
        double[] localFix = coordinateConverter.toLocalMeters(latitudeDeg, longitudeDeg);
        if (!pfInitialized) {
            particleFilterEngine.initialize(
                    localFix[0],
                    localFix[1],
                    initializationFloor,
                    timestampMs,
                    headingRad,
                    Math.max(MIN_ABSOLUTE_FIX_START_STD_M, accuracyMeters)
            );
        } else {
            particleFilterEngine.updateWithAbsoluteFix(
                    localFix[0],
                    localFix[1],
                    floorPrior,
                    timestampMs,
                    accuracyMeters
            );
        }
        return particleFilterEngine.estimatePose();
    }

    @Nullable
    static FusedPose applyPdrPredictionForStep1(
            @NonNull ParticleFilterEngine particleFilterEngine,
            boolean pfInitialized,
            @Nullable PdrDelta pdrDelta,
            int floor,
            long timestampMs
    ) {
        if (!pfInitialized || pdrDelta == null) {
            return particleFilterEngine.estimatePose();
        }
        particleFilterEngine.predict(pdrDelta, floor, timestampMs);
        return particleFilterEngine.estimatePose();
    }

    private float getCurrentHeadingRad() {
        if (orientation == null || orientation.length == 0 || Float.isNaN(orientation[0])) {
            return 0f;
        }
        return orientation[0];
    }

    private ParticleFilterEngine createParticleFilterEngine() {
        ParticleInitializer.SpawnValidator particleValidator = new ParticleInitializer.SpawnValidator() {
            @Override
            public boolean isValid(double x, double y, int floor) {
                return isValidParticlePrediction(x, y, floor);
            }

            @Override
            public boolean isValidMotion(
                    double previousX,
                    double previousY,
                    double predictedX,
                    double predictedY,
                    int previousFloor,
                    int predictedFloor
            ) {
                return isValidParticleMotion(
                        previousX,
                        previousY,
                        predictedX,
                        predictedY,
                        previousFloor,
                        predictedFloor
                );
            }
        };
        return new ParticleFilterEngine(
                new ParticleInitializer(),
                particleValidator,
                this::allowsMapBasedFloorTransition
        );
    }

    /**
     * When map_shapes provides stairs/lift polygons for the UI-selected floor, only allow a change
     * in particle floor (from barometer / PdrProcessing) if the step starts or ends inside one
     * of those zones. If no transition geometry is loaded, fail-open so height-based floor still works.
     * WiFi/GNSS absolute fixes use {@link #handleAbsoluteFix} and are not gated here.
     */
    private boolean allowsMapBasedFloorTransition(
            double previousXMeters,
            double previousYMeters,
            double predictedXMeters,
            double predictedYMeters,
            int previousFloor,
            int newFloor
    ) {
        if (newFloor == previousFloor) {
            return true;
        }
        boolean hasAnyMapConstraints = MapConstraintRepository.hasAnyConstraints();
        if (!hasAnyMapConstraints) {
            return true;
        }
        if (!MapConstraintRepository.hasTransitionConstraints(previousFloor)
                && !MapConstraintRepository.hasTransitionConstraints(newFloor)) {
            return false;
        }
        if (coordinateConverter == null) {
            return false;
        }
        LatLng previousLatLng = coordinateConverter.toLatLng(previousXMeters, previousYMeters);
        LatLng predictedLatLng = coordinateConverter.toLatLng(predictedXMeters, predictedYMeters);
        if (previousLatLng == null || predictedLatLng == null) {
            return false;
        }
        if (getElevator()) {
            return MapConstraintRepository.isPointInsideLift(previousLatLng, previousFloor)
                    || MapConstraintRepository.isPointInsideLift(predictedLatLng, newFloor)
                    || MapConstraintRepository.doesPathIntersectLift(previousLatLng, predictedLatLng, previousFloor)
                    || MapConstraintRepository.doesPathIntersectLift(previousLatLng, predictedLatLng, newFloor);
        }
        return MapConstraintRepository.isPointInsideStairs(previousLatLng, previousFloor)
                || MapConstraintRepository.isPointInsideStairs(predictedLatLng, newFloor)
                || MapConstraintRepository.doesPathIntersectStairs(previousLatLng, predictedLatLng, previousFloor)
                || MapConstraintRepository.doesPathIntersectStairs(previousLatLng, predictedLatLng, newFloor);
    }

    private boolean isValidParticlePrediction(double x, double y, int floor) {
        if (coordinateConverter == null) {
            // Fail-open: map constraints are unavailable before origin/converter is ready.
            return true;
        }

        LatLng candidateLatLng = coordinateConverter.toLatLng(x, y);
        if (candidateLatLng == null) {
            return true;
        }

        if (MapConstraintRepository.hasWallConstraints(floor)
                && MapConstraintRepository.isPointInsideWall(candidateLatLng, floor)) {
            return false;
        }

        if (MapConstraintRepository.hasVenueOutline()) {
            return MapConstraintRepository.isPointInsideVenueOutline(candidateLatLng);
        }

        int buildingMode = resolveConstraintBuildingMode();
        if (buildingMode == 1) {
            return BuildingPolygon.inNucleus(candidateLatLng);
        }
        if (buildingMode == 2) {
            return BuildingPolygon.inLibrary(candidateLatLng);
        }
        return true;
    }

    private boolean isValidParticleMotion(
            double previousX,
            double previousY,
            double predictedX,
            double predictedY,
            int previousFloor,
            int predictedFloor
    ) {
        if (!isValidParticlePrediction(predictedX, predictedY, predictedFloor)) {
            return false;
        }
        if (coordinateConverter == null) {
            return true;
        }

        LatLng previousLatLng = coordinateConverter.toLatLng(previousX, previousY);
        LatLng predictedLatLng = coordinateConverter.toLatLng(predictedX, predictedY);
        if (previousLatLng == null || predictedLatLng == null) {
            return true;
        }
        if (MapConstraintRepository.hasWallConstraints(predictedFloor)
                && MapConstraintRepository.doesPathIntersectWall(previousLatLng, predictedLatLng, predictedFloor)) {
            return false;
        }
        return previousFloor == predictedFloor
                || !MapConstraintRepository.hasWallConstraints(previousFloor)
                || !MapConstraintRepository.doesPathIntersectWall(previousLatLng, predictedLatLng, previousFloor);
    }

    private int resolveConstraintBuildingMode() {
        String venue = collectionVenue == null ? "" : collectionVenue.toLowerCase(Locale.US);
        if (venue.contains(VENUE_KEY_NUCLEUS)) {
            return 1;
        }
        if (venue.contains(VENUE_KEY_LIBRARY) || venue.contains(VENUE_KEY_MURCHISON)) {
            return 2;
        }

        if (startLocation == null || startLocation.length < 2) {
            return 0;
        }

        LatLng origin = new LatLng(startLocation[0], startLocation[1]);
        if (BuildingPolygon.inNucleus(origin)) {
            return 1;
        }
        if (BuildingPolygon.inLibrary(origin)) {
            return 2;
        }
        return 0;
    }

    private float normalizeHeadingDelta(float deltaHeadingRad) {
        float twoPi = (float) (Math.PI * 2.0);
        float normalized = deltaHeadingRad % twoPi;
        if (normalized > Math.PI) {
            normalized -= twoPi;
        } else if (normalized < -Math.PI) {
            normalized += twoPi;
        }
        return normalized;
    }

    /**
     * Method used for converting an array of orientation angles into a rotation matrix.
     *
     * @param o An array containing orientation angles in radians
     * @return resultMatrix representing the orientation angles
     */
    private float[] getRotationMatrixFromOrientation(float[] o) {
        float[] xM = new float[9];
        float[] yM = new float[9];
        float[] zM = new float[9];

        float sinX = (float)Math.sin(o[1]);
        float cosX = (float)Math.cos(o[1]);
        float sinY = (float)Math.sin(o[2]);
        float cosY = (float)Math.cos(o[2]);
        float sinZ = (float)Math.sin(o[0]);
        float cosZ = (float)Math.cos(o[0]);

        // rotation about x-axis (pitch)
        xM[0] = 1.0f; xM[1] = 0.0f; xM[2] = 0.0f;
        xM[3] = 0.0f; xM[4] = cosX; xM[5] = sinX;
        xM[6] = 0.0f; xM[7] = -sinX; xM[8] = cosX;

        // rotation about y-axis (roll)
        yM[0] = cosY; yM[1] = 0.0f; yM[2] = sinY;
        yM[3] = 0.0f; yM[4] = 1.0f; yM[5] = 0.0f;
        yM[6] = -sinY; yM[7] = 0.0f; yM[8] = cosY;

        // rotation about z-axis (azimuth)
        zM[0] = cosZ; zM[1] = sinZ; zM[2] = 0.0f;
        zM[3] = -sinZ; zM[4] = cosZ; zM[5] = 0.0f;
        zM[6] = 0.0f; zM[7] = 0.0f; zM[8] = 1.0f;

        // rotation order is y, x, z (roll, pitch, azimuth)
        float[] resultMatrix = matrixMultiplication(xM, yM);
        resultMatrix = matrixMultiplication(zM, resultMatrix);
        return resultMatrix;
    }

    /**
     * Performs and matrix multiplication of two 3x3 matrices and returns the product.
     *
     * @param A An array representing a 3x3 matrix
     * @param B An array representing a 3x3 matrix
     * @return result representing the product of A and B
     */
    private float[] matrixMultiplication(float[] A, float[] B) {
        float[] result = new float[9];

        result[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
        result[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
        result[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];

        result[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
        result[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
        result[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];

        result[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
        result[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
        result[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];

        return result;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {}
    //endregion

    //region Getters/Setters
    /**
     * Getter function for core location data.
     *
     * @param start set true to get the initial location
     * @return longitude and latitude data in a float[2].
     */
    public float[] getGNSSLatitude(boolean start) {
        float [] latLong = new float[2];
        if(!start) {
            latLong[0] = latitude;
            latLong[1] = longitude;
        }
        else if (startLocation != null && startLocation.length >= 2) {
            latLong[0] = startLocation[0];
            latLong[1] = startLocation[1];
        }
        return latLong;
    }

    /**
     * Setter function for core location data.
     *
     * @param startPosition contains the initial location set by the user
     */
    public void setStartGNSSLatitude(float[] startPosition){
        if (startPosition == null || startPosition.length < 2) {
            startLocation = null;
            hasManualStartLocation = false;
            coordinateConverter = null;
            return;
        }
        setTrajectoryOrigin(startPosition[0], startPosition[1], true);
    }

    public void clearLegacyManualStartLocation() {
        startLocation = null;
        hasManualStartLocation = false;
        coordinateConverter = null;
    }

    public synchronized void setCollectionVenue(String venueId) {
        this.collectionVenue = sanitiseVenueTagStatic(venueId);
        if (this.trajectory == null) {
            return;
        }
        String existingId = this.trajectory.getTrajectoryId();
        String suffix;
        String[] parts = existingId.split("_", 3);
        if (parts.length == 3) {
            suffix = parts[1] + "_" + parts[2];
        } else {
            String timestamp = new SimpleDateFormat("yyyyMMdd_HHmm", Locale.getDefault()).format(new Date());
            String shortUuid = UUID.randomUUID().toString().replace("-", "").substring(0, 8);
            suffix = timestamp + "_" + shortUuid;
        }
        this.trajectory.setTrajectoryId(this.collectionVenue + "_" + suffix);
    }

    public synchronized String getCollectionVenue() {
        return collectionVenue;
    }

    public synchronized void setTrajectoryName(@Nullable String desiredName) {
        this.trajectoryName = sanitiseTrajectoryName(desiredName);
        if (this.trajectory != null) {
            this.trajectory.setTrajectoryName(buildTrajectoryName(this.trajectoryName, absoluteStartTime));
        }
    }

    public synchronized String getTrajectoryName() {
        return buildTrajectoryName(this.trajectoryName, absoluteStartTime);
    }

    private String sanitiseVenueTag(String venue) {
        return sanitiseVenueTagStatic(venue);
    }

    static String sanitiseVenueTagStatic(String venue) {
        if (venue == null) {
            return DEFAULT_COLLECTION_VENUE;
        }
        String normalised = venue.trim().toLowerCase(Locale.US)
                .replaceAll("[^a-z0-9]+", "_")
                .replaceAll("_+", "_")
                .replaceAll("^_", "")
                .replaceAll("_$", "");
        if (normalised.isEmpty()) {
            return DEFAULT_COLLECTION_VENUE;
        }
        return normalised;
    }

    private String sanitiseTrajectoryName(@Nullable String name) {
        if (name == null) {
            return "";
        }
        String normalised = name.trim().replaceAll("\\s+", " ");
        if (normalised.length() > 60) {
            normalised = normalised.substring(0, 60).trim();
        }
        return normalised;
    }

    static String buildTrajectoryName(@Nullable String inputName, long timestampMs) {
        String trimmed = inputName == null ? "" : inputName.trim();
        if (!trimmed.isEmpty()) {
            return trimmed;
        }
        String formatted = new SimpleDateFormat("yyyy-MM-dd HH:mm", Locale.getDefault())
                .format(new Date(timestampMs > 0 ? timestampMs : System.currentTimeMillis()));
        return "Trajectory " + formatted;
    }


    /**
     * Function to redraw path in corrections fragment.
     *
     * @param scalingRatio new size of path due to updated step length
     */
    public void redrawPath(float scalingRatio){
        pathView.redraw(scalingRatio);
    }

    /**
     * Getter function for average step count.
     * Calls the average step count function in pdrProcessing class
     *
     * @return average step count of total PDR.
     */
    public float passAverageStepLength(){
        return pdrProcessing.getAverageStepLength();
    }

    /**
     * Getter function for the current heading.
     *
     * @return heading in radians, relative to true north with clockwise positive rotation.
     */
    public float passOrientation(){
        return orientation[0];
    }

    /**
     * Heading used by the map marker.
     */
    public float passDisplayOrientation() {
        float resultRad;
        if (displayOrientation == null || displayOrientation.length == 0 || Float.isNaN(displayOrientation[0])) {
            resultRad = passOrientation();
        } else {
            resultRad = displayOrientation[0];
        }
        logArrowDisplayOrientationTrace(resultRad);
        return resultRad;
    }

    // Mirror the corrected heading into the UI-facing path without changing marker/UI code.
    private void updateDisplayOrientation(@NonNull float[] ignoredRotationMatrix) {
        // 这次只修“手机顶部方向 != 地图箭头方向”。
        // ARROW_DBG 已确认固定偏角出在 raw -> displayOrientation，
        // 因此显示链路直接沿用原始 device azimuth，不改 raw orientation，也不影响 PF/PDR。
        System.arraycopy(this.orientation, 0, this.displayOrientation, 0, this.orientation.length);
    }

    private void updateTrueNorthOrientation(@NonNull float[] horizontalWorldRotationMatrix) {
        SensorManager.getOrientation(horizontalWorldRotationMatrix, this.orientation);
        this.orientation[0] = toTrueNorthHeadingRad(this.orientation[0]);
    }

    @SuppressWarnings("deprecation")
    private int getDisplayRotation() {
        if (appContext == null) {
            return Surface.ROTATION_0;
        }
        WindowManager windowManager = (WindowManager) appContext.getSystemService(Context.WINDOW_SERVICE);
        if (windowManager == null || windowManager.getDefaultDisplay() == null) {
            return Surface.ROTATION_0;
        }
        return windowManager.getDefaultDisplay().getRotation();
    }

    // 只跟踪地图箭头显示链路：原始 azimuth、displayOrientation 和当前屏幕 rotation。
    private void logArrowRawOrientationTrace() {
        if (!BuildConfig.DEBUG) {
            return;
        }
        long now = SystemClock.elapsedRealtime();
        if (now - arrowDbgRawLastLogMs < ARROW_DBG_INTERVAL_MS) {
            return;
        }
        float rawRad = orientation == null || orientation.length == 0 ? Float.NaN : orientation[0];
        float displayRad = displayOrientation == null || displayOrientation.length == 0
                ? Float.NaN
                : displayOrientation[0];
        int displayRotation = getDisplayRotation();
        Log.d(
                ARROW_DBG_TAG,
                "stage=SensorFusion.raw"
                        + " rawRad=" + formatDebugDouble(rawRad)
                        + " rawDeg=" + formatDebugDouble(Math.toDegrees(rawRad))
                        + " rawDeg360=" + formatDebugDouble(normalizeDegrees(Math.toDegrees(rawRad)))
                        + " displayRad=" + formatDebugDouble(displayRad)
                        + " displayDeg=" + formatDebugDouble(Math.toDegrees(displayRad))
                        + " displayDeg360=" + formatDebugDouble(normalizeDegrees(Math.toDegrees(displayRad)))
                        + " surfaceRotation=" + displayRotation
                        + "(" + formatSurfaceRotation(displayRotation) + ")"
        );
        arrowDbgRawLastLogMs = now;
    }

    // 记录 UI 真正取走的 displayOrientation 返回值，便于和后续 setRotation 对齐。
    private void logArrowDisplayOrientationTrace(float resultRad) {
        if (!BuildConfig.DEBUG) {
            return;
        }
        long now = SystemClock.elapsedRealtime();
        if (now - arrowDbgDisplayLastLogMs < ARROW_DBG_INTERVAL_MS) {
            return;
        }
        float rawRad = orientation == null || orientation.length == 0 ? Float.NaN : orientation[0];
        int displayRotation = getDisplayRotation();
        Log.d(
                ARROW_DBG_TAG,
                "stage=SensorFusion.passDisplayOrientation"
                        + " returnRad=" + formatDebugDouble(resultRad)
                        + " returnDeg=" + formatDebugDouble(Math.toDegrees(resultRad))
                        + " returnDeg360=" + formatDebugDouble(normalizeDegrees(Math.toDegrees(resultRad)))
                        + " rawRad=" + formatDebugDouble(rawRad)
                        + " rawDeg360=" + formatDebugDouble(normalizeDegrees(Math.toDegrees(rawRad)))
                        + " surfaceRotation=" + displayRotation
                        + "(" + formatSurfaceRotation(displayRotation) + ")"
        );
        arrowDbgDisplayLastLogMs = now;
    }

    private double normalizeDegrees(double degrees) {
        if (!Double.isFinite(degrees)) {
            return Double.NaN;
        }
        double normalized = degrees % 360.0;
        return normalized < 0.0 ? normalized + 360.0 : normalized;
    }

    private float toTrueNorthHeadingRad(float magneticHeadingRad) {
        ensureGeomagneticDeclination();
        float headingTrueDeg = (float) Math.toDegrees(magneticHeadingRad)
                + (hasGeomagneticDeclination ? geomagneticDeclinationDeg : 0f);
        return normalizePositiveHeadingRad(headingTrueDeg);
    }

    private float normalizePositiveHeadingRad(float headingDeg) {
        float normalizedDeg = headingDeg % 360f;
        if (normalizedDeg < 0f) {
            normalizedDeg += 360f;
        }
        return (float) Math.toRadians(normalizedDeg);
    }

    private void ensureGeomagneticDeclination() {
        long nowMs = System.currentTimeMillis();
        if (hasGeomagneticDeclination && !shouldRefreshGeomagneticDeclination(nowMs)) {
            return;
        }
        if (isValidLatLng(latitude, longitude)) {
            updateGeomagneticDeclination(latitude, longitude, (float) gnssAltitude, nowMs);
            return;
        }
        if (startLocation != null && startLocation.length >= 2 && isValidLatLng(startLocation[0], startLocation[1])) {
            updateGeomagneticDeclination(startLocation[0], startLocation[1], 0f, nowMs);
            return;
        }
        LatLng fusedLatLng = getLatLngForFusedPose(latestFusedPose);
        if (fusedLatLng != null) {
            updateGeomagneticDeclination(fusedLatLng.latitude, fusedLatLng.longitude, 0f, nowMs);
        }
    }

    private boolean shouldRefreshGeomagneticDeclination(long timestampMs) {
        if (!hasGeomagneticDeclination) {
            return true;
        }
        return Math.abs(timestampMs - geomagneticDeclinationTimestampMs) >= DECLINATION_REFRESH_INTERVAL_MS;
    }

    private void updateGeomagneticDeclination(
            double latitudeDeg,
            double longitudeDeg,
            float altitudeMeters,
            long timestampMs
    ) {
        if (!isValidLatLng(latitudeDeg, longitudeDeg)) {
            return;
        }
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        if (hasGeomagneticDeclination
                && !shouldRefreshGeomagneticDeclination(safeTimestampMs)
                && Math.abs(latitudeDeg - geomagneticDeclinationLatitudeDeg) < 1e-4
                && Math.abs(longitudeDeg - geomagneticDeclinationLongitudeDeg) < 1e-4
                && Math.abs(altitudeMeters - geomagneticDeclinationAltitudeMeters) < 5f) {
            return;
        }
        GeomagneticField geomagneticField = new GeomagneticField(
                (float) latitudeDeg,
                (float) longitudeDeg,
                altitudeMeters,
                safeTimestampMs
        );
        geomagneticDeclinationDeg = geomagneticField.getDeclination();
        geomagneticDeclinationTimestampMs = safeTimestampMs;
        geomagneticDeclinationLatitudeDeg = latitudeDeg;
        geomagneticDeclinationLongitudeDeg = longitudeDeg;
        geomagneticDeclinationAltitudeMeters = altitudeMeters;
        hasGeomagneticDeclination = true;
    }

    private boolean isValidLatLng(double latitudeDeg, double longitudeDeg) {
        return Double.isFinite(latitudeDeg)
                && Double.isFinite(longitudeDeg)
                && !(latitudeDeg == 0.0 && longitudeDeg == 0.0);
    }

    @NonNull
    private String formatSurfaceRotation(int displayRotation) {
        switch (displayRotation) {
            case Surface.ROTATION_90:
                return "ROTATION_90";
            case Surface.ROTATION_180:
                return "ROTATION_180";
            case Surface.ROTATION_270:
                return "ROTATION_270";
            case Surface.ROTATION_0:
            default:
                return "ROTATION_0";
        }
    }

    /**
     * Return most recent sensor readings.
     *
     * Collects all most recent readings from movement and location sensors, packages them in a map
     * that is indexed by {@link SensorTypes} and makes it accessible for other classes.
     *
     * @return  Map of <code>SensorTypes</code> to float array of most recent values.
     */
    public Map<SensorTypes, float[]> getSensorValueMap() {
        Map<SensorTypes, float[]> sensorValueMap = new HashMap<>();
        sensorValueMap.put(SensorTypes.ACCELEROMETER, acceleration);
        sensorValueMap.put(SensorTypes.GRAVITY, gravity);
        sensorValueMap.put(SensorTypes.MAGNETICFIELD, magneticField);
        sensorValueMap.put(SensorTypes.GYRO, angularVelocity);
        sensorValueMap.put(SensorTypes.LIGHT, new float[]{light});
        sensorValueMap.put(SensorTypes.PRESSURE, new float[]{pressure});
        sensorValueMap.put(SensorTypes.PROXIMITY, new float[]{proximity});
        sensorValueMap.put(SensorTypes.GNSSLATLONG, getGNSSLatitude(false));
        sensorValueMap.put(SensorTypes.PDR, pdrProcessing.getPDRMovement());
        return sensorValueMap;
    }

    /**
     * Return the most recent list of WiFi names and levels.
     * Each Wifi object contains a BSSID and a level value.
     *
     * @return  list of Wifi objects.
     */
    public List<Wifi> getWifiList() {
        return this.wifiList;
    }

    public boolean isRecordingInProgress() {
        return saveRecording;
    }

    public boolean areListenersActive() {
        return listenersActive;
    }

    /**
     * 获取最近一次 BLE 扫描窗口的去重设备数量（只读展示用）。
     */
    public int getLatestBleDeviceCount() {
        if (bleProcessor == null) {
            return 0;
        }
        return bleProcessor.getLatestBleDeviceCount();
    }

    /**
     * 获取最近一次 BLE 扫描窗口的最强 RSSI（只读展示用）。
     */
    public int getLatestStrongestBleRssi() {
        if (bleProcessor == null) {
            return -100;
        }
        return bleProcessor.getLatestStrongestBleRssi();
    }

    /**
     * Get information about all the sensors registered in SensorFusion.
     *
     * @return  List of SensorInfo objects containing name, resolution, power, etc.
     */
    public List<SensorInfo> getSensorInfos() {
        List<SensorInfo> sensorInfoList = new ArrayList<>();
        sensorInfoList.add(this.accelerometerSensor.sensorInfo);
        sensorInfoList.add(this.barometerSensor.sensorInfo);
        sensorInfoList.add(this.gyroscopeSensor.sensorInfo);
        sensorInfoList.add(this.lightSensor.sensorInfo);
        sensorInfoList.add(this.proximitySensor.sensorInfo);
        sensorInfoList.add(this.magnetometerSensor.sensorInfo);
        return sensorInfoList;
    }

    /**
     * Registers the caller observer to receive updates from the server instance.
     * Necessary when classes want to act on a trajectory being successfully or unsuccessfully send
     * to the server. This grants access to observing the {@link ServerCommunications} instance
     * used by the SensorFusion class.
     *
     * @param observer  Instance implementing {@link Observer} class who wants to be notified of
     *                  events relating to sending and receiving trajectories.
     */
    public void registerForServerUpdate(Observer observer) {
        serverCommunications.registerObserver(observer);
    }

    /**
     * Get the estimated elevation value in meters calculated by the PDR class.
     * Elevation is relative to the starting position.
     *
     * @return  float of the estimated elevation in meters.
     */
    public float getElevation() {
        return this.elevation;
    }

    public int getCurrentFloor() {
        return getAbsoluteCurrentFloor();
    }

    public void setVenueFloorHeightMeters(float floorHeightMeters) {
        if (this.pdrProcessing != null && floorHeightMeters > 0f) {
            this.pdrProcessing.setFloorHeightMeters(floorHeightMeters);
        }
    }

    public void clearVenueFloorHeightOverride() {
        if (this.pdrProcessing != null) {
            this.pdrProcessing.clearFloorHeightOverride();
        }
    }

    /**
     * Get an estimate by the PDR class whether it estimates the user is currently taking an elevator.
     *
     * @return  true if the PDR estimates the user is in an elevator, false otherwise.
     */
    public boolean getElevator() {
        return this.elevator;
    }

    /**
     * Estimates position of the phone based on proximity and light sensors.
     *
     * @return int 1 if the phone is by the ear, int 0 otherwise.
     */
    public int getHoldMode(){
        int proximityThreshold = 1, lightThreshold = 100; //holdMode: by ear=1, not by ear =0
        if(proximity<proximityThreshold && light>lightThreshold) { //unit cm
            return 1;
        }
        else{
            return 0;
        }
    }

    //endregion

    //region Start/Stop

    /**
     * Registers all device listeners and enables updates with the specified sampling rate.
     *
     * Should be called from {@link MainActivity} when resuming the application. Sampling rate is in
     * microseconds, IMU needs 100Hz, rest 1Hz
     *
     * @see MovementSensor handles SensorManager based devices.
     * @see WifiDataProcessor handles wifi data.
     * @see GNSSDataProcessor handles location data.
     */
    public void resumeListening() {
        if (!listenersActive) {
            accelerometerSensor.sensorManager.registerListener(this, accelerometerSensor.sensor, 10000, (int) maxReportLatencyNs);
            accelerometerSensor.sensorManager.registerListener(this, linearAccelerationSensor.sensor, 10000, (int) maxReportLatencyNs);
            accelerometerSensor.sensorManager.registerListener(this, gravitySensor.sensor, 10000, (int) maxReportLatencyNs);
            barometerSensor.sensorManager.registerListener(this, barometerSensor.sensor, (int) 1e6);
            gyroscopeSensor.sensorManager.registerListener(this, gyroscopeSensor.sensor, 10000, (int) maxReportLatencyNs);
            lightSensor.sensorManager.registerListener(this, lightSensor.sensor, (int) 1e6);
            proximitySensor.sensorManager.registerListener(this, proximitySensor.sensor, (int) 1e6);
            magnetometerSensor.sensorManager.registerListener(this, magnetometerSensor.sensor, 10000, (int) maxReportLatencyNs);
            stepDetectionSensor.sensorManager.registerListener(this, stepDetectionSensor.sensor, SensorManager.SENSOR_DELAY_NORMAL);
            rotationSensor.sensorManager.registerListener(this, rotationSensor.sensor, (int) 1e6);
            listenersActive = true;
        }
        wifiProcessor.startListening();
        gnssProcessor.startLocationUpdates();
        bleProcessor.startListening();
    }

    /**
     * Un-registers all device listeners and pauses data collection.
     *
     * Should be called from {@link MainActivity} when pausing the application.
     *
     * @see MovementSensor handles SensorManager based devices.
     * @see WifiDataProcessor handles wifi data.
     * @see GNSSDataProcessor handles location data.
     */
    public void stopListening() {
        if (listenersActive) {
            accelerometerSensor.sensorManager.unregisterListener(this);
            barometerSensor.sensorManager.unregisterListener(this);
            gyroscopeSensor.sensorManager.unregisterListener(this);
            lightSensor.sensorManager.unregisterListener(this);
            proximitySensor.sensorManager.unregisterListener(this);
            magnetometerSensor.sensorManager.unregisterListener(this);
            stepDetectionSensor.sensorManager.unregisterListener(this);
            rotationSensor.sensorManager.unregisterListener(this);
            linearAccelerationSensor.sensorManager.unregisterListener(this);
            gravitySensor.sensorManager.unregisterListener(this);
            listenersActive = false;
        }
        wifiProcessor.stopListening();
        gnssProcessor.stopUpdating();
        bleProcessor.stopListening();
    }

    /**
     * Pause only BLE scanning (e.g. when fragment is paused) without changing recording flag.
     */
    public void pauseBleScan() {
        if (bleProcessor != null) {
            try {
                bleProcessor.stopListening();
            } catch (Exception e) {
                Log.w("SensorFusion", "pauseBleScan(): failed to stop BLE", e);
            }
        }
    }

    /**
     * Enables saving sensor values to the trajectory object.
     *
     * Sets save recording to true, resets the absolute start time and create new timer object for
     * periodically writing data to trajectory.
     *
     * @see Traj object for storing data.
     */
    public void startRecording() {
        // If wakeLock is null (e.g. not initialized or was cleared), reinitialize it.
        if (wakeLock == null) {
            PowerManager powerManager = (PowerManager) this.appContext.getSystemService(Context.POWER_SERVICE);
            wakeLock = powerManager.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "MyApp::MyWakelockTag");
        }
        // 确保录制期间 CPU 常驻，避免重复 acquire
        if (wakeLock != null && !wakeLock.isHeld()) {
            wakeLock.acquire(31 * 60 * 1000L /*31 minutes*/);
        }

        // 录制前校验电池优化/后台限制状态，并提示用户
        verifyAlwaysOnReadiness();

        this.saveRecording = true;
        this.stepCounter = 0;
        this.absoluteStartTime = System.currentTimeMillis();
        this.bootTime = SystemClock.uptimeMillis();
        String venueOrBuilding = sanitiseVenueTag(collectionVenue);
        this.collectionVenue = venueOrBuilding;
        String timestamp = new SimpleDateFormat("yyyyMMdd_HHmm", Locale.getDefault())
                .format(new Date(absoluteStartTime));
        String shortUuid = UUID.randomUUID().toString().replace("-", "").substring(0, 8);
        this.trajectoryId = venueOrBuilding + "_" + timestamp + "_" + shortUuid;
        this.trajectoryName = buildTrajectoryName(this.trajectoryName, absoluteStartTime);
        this.testPoints.clear();

        // Protobuf trajectory class for sending sensor data to restful API
        Traj.Trajectory.Builder trajectoryBuilder = Traj.Trajectory.newBuilder()
                .setTrajectoryId(trajectoryId)
                .setTrajectoryName(trajectoryName)
                .setTrajectoryVersion(2.0f)
                .setAndroidVersion(Build.VERSION.RELEASE)
                .setStartTimestamp(absoluteStartTime)
                .setAccelerometerInfo(createInfoBuilder(accelerometerSensor))
                .setGyroscopeInfo(createInfoBuilder(gyroscopeSensor))
                .setMagnetometerInfo(createInfoBuilder(magnetometerSensor))
                .setBarometerInfo(createInfoBuilder(barometerSensor))
                .setLightSensorInfo(createInfoBuilder(lightSensor));

        if (hasManualStartLocation) {
            maybeSetInitialPosition(trajectoryBuilder, startLocation);
        }

        this.trajectory = trajectoryBuilder;

        this.recordedApMacs = new HashSet<>();
        this.lastFingerprintSignature = null;
        // +++ add these for BLE +++
        this.recordedBleMacs = new HashSet<>();
        this.lastBleFingerprintSignature = null;

        this.storeTrajectoryTimer = new Timer();
        this.storeTrajectoryTimer.schedule(new storeDataInTrajectory(), 0, TIME_CONST);
        this.pdrProcessing.resetPDR();
        this.pdrFloorOffset = 0;
        this.isFloorOffsetInitialized = false;
        this.elevation = 0f;
        this.elevator = false;
        this.particleFilterEngine = createParticleFilterEngine();
        this.pfInitialized = false;
        this.latestFusedPose = null;
        this.absoluteFloorTransitionResolver.reset();
        this.coordinateConverter = hasManualStartLocation && startLocation != null && startLocation.length >= 2
                ? new CoordinateConverter(startLocation[0], startLocation[1])
                : null;
        this.hasGeomagneticDeclination = false;
        this.geomagneticDeclinationDeg = 0f;
        this.geomagneticDeclinationTimestampMs = 0L;
        this.geomagneticDeclinationLatitudeDeg = Double.NaN;
        this.geomagneticDeclinationLongitudeDeg = Double.NaN;
        this.geomagneticDeclinationAltitudeMeters = 0f;
        this.lastPredictHeadingRad = getCurrentHeadingRad();
        this.lastPredictElevation = this.elevation;
        this.lastRecordedFusedPoseTimestampMs = -1L;
        this.lastWifiScanWallClockMs = -1L;
        this.lastDisplayedFusedMarkerLatLng = null;
        this.lastDisplayedFusedMarkerRawLatLng = null;
        this.lastDisplayedFusedMarkerTimestampMs = -1L;
        if(settings.getBoolean("overwrite_constants", false)) {
            this.filter_coefficient = Float.parseFloat(settings.getString("accel_filter", "0.96"));
        } else {
            this.filter_coefficient = FILTER_COEFFICIENT;
        }
        Log.d("BLE_PIPE", "startRecording(): saveRecording=" + saveRecording);

    }

    /**
     * 校验“始终运行”相关系统状态：电池优化、后台限制、省电模式。
     * 仅提示用户，不强制跳转。
     */
    private void verifyAlwaysOnReadiness() {
        String pkg = appContext.getPackageName();
        PowerManager pm = (PowerManager) appContext.getSystemService(Context.POWER_SERVICE);
        ActivityManager am = (ActivityManager) appContext.getSystemService(Context.ACTIVITY_SERVICE);

        boolean ignoringOpt = false;
        boolean powerSave = false;
        boolean bgRestricted = false;
        try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
                ignoringOpt = pm != null && pm.isIgnoringBatteryOptimizations(pkg);
            }
            powerSave = pm != null && pm.isPowerSaveMode();
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.P) {
                bgRestricted = am != null && am.isBackgroundRestricted();
            }
        } catch (Exception e) {
            Log.w("SensorFusion", "电池/后台状态检查失败: " + e.getMessage());
            Toast.makeText(appContext, "电池优化状态未知，请手动确认", Toast.LENGTH_LONG).show();
            return;
        }

        if (!ignoringOpt || powerSave || bgRestricted) {
            StringBuilder warn = new StringBuilder("检测到可能的后台限制：");
            if (!ignoringOpt) warn.append("电池优化未豁免; ");
            if (powerSave) warn.append("省电模式开启; ");
            if (bgRestricted) warn.append("后台限制开启; ");
            Log.w("SensorFusion", warn.toString());
            Toast.makeText(appContext,
                    "请在设置中关闭电池优化/省电/后台限制，确保录制不中断",
                    Toast.LENGTH_LONG).show();
        } else {
            Log.i("SensorFusion", "电池优化/后台限制检查通过：已豁免优化且未开启省电/后台限制。");
        }
    }

    /**
     * Disables saving sensor values to the trajectory object.
     *
     * Check if a recording is in progress. If it is, it sets save recording to false, and cancels
     * the timer objects.
     *
     * @see Traj object for storing data.
     * @see SettingsFragment navigation that might cancel recording.
     */
    public void stopRecording() {
        recordLatestFusedPoseIfNeeded();
        // Only cancel if we are running
        if(this.saveRecording) {
            this.saveRecording = false;
            storeTrajectoryTimer.cancel();
        }
        if (wakeLock != null && wakeLock.isHeld()) {
            this.wakeLock.release();
        }
    }

    /**
     * NEW METHOD: Sets the Venue ID for the current trajectory recording.
     * This should be called before sendTrajectoryToCloud().
     * @param venueId The ID of the venue selected by the user.
     */
    public void setVenueIdForTrajectory(String venueId) {
        this.collectionVenue = sanitiseVenueTagStatic(venueId);
        if (this.trajectory != null) {
            applyVenuePrefixToTrajectoryId(this.trajectory, this.collectionVenue);
            Log.i("SensorFusion", "Venue tag '" + this.collectionVenue + "' applied to trajectoryId");
        } else {
            Log.w("SensorFusion", "Trajectory not ready; venue tag cached as " + this.collectionVenue);
        }
    }

    // Visible for tests; prefixes the trajectory_id with venue tag when missing.
    static void applyVenuePrefixToTrajectoryId(Traj.Trajectory.Builder trajectoryBuilder, String venueId) {
        if (trajectoryBuilder == null) {
            return;
        }
        String safeVenue = sanitiseVenueTagStatic(venueId);
        String existingId = trajectoryBuilder.getTrajectoryId();
        if (existingId == null || existingId.isEmpty()) {
            trajectoryBuilder.setTrajectoryId(safeVenue);
            return;
        }
        String prefix = safeVenue + "_";
        if (!existingId.startsWith(prefix)) {
            trajectoryBuilder.setTrajectoryId(prefix + existingId);
        }
    }

    // Visible for tests
    static void maybeSetInitialPosition(Traj.Trajectory.Builder builder, float[] startLoc) {
        if (builder == null || startLoc == null || startLoc.length < 2) {
            return;
        }
        Traj.GNSSPosition.Builder pos = Traj.GNSSPosition.newBuilder()
                .setLatitude(startLoc[0])
                .setLongitude(startLoc[1]);
        builder.setInitialPosition(pos);
    }

    static void maybeSetInitialPositionIfAbsent(
            @Nullable Traj.Trajectory.Builder builder,
            double latitudeDeg,
            double longitudeDeg
    ) {
        if (builder == null || builder.hasInitialPosition()) {
            return;
        }
        Traj.GNSSPosition.Builder pos = Traj.GNSSPosition.newBuilder()
                .setLatitude(latitudeDeg)
                .setLongitude(longitudeDeg);
        builder.setInitialPosition(pos);
    }


    //endregion

    //region Trajectory object

    /**
     * Send the trajectory object to servers.
     *
     * @see ServerCommunications for sending and receiving data via HTTPS.
     */
    public void sendTrajectoryToCloud() {
        // Build object
        Traj.Trajectory sentTrajectory = trajectory.build();
        Log.d("MARKER", "UPLOAD proto test_points_count=" + sentTrajectory.getTestPointsCount()
                + " trajectory_id=" + sentTrajectory.getTrajectoryId()
                + " trajectory_name=" + sentTrajectory.getTrajectoryName());
        // Pass object to communications object
        this.serverCommunications.sendTrajectory(sentTrajectory);
    }

    public LatLng getCurrentGnssLatLng() {
        // 还没拿到定位时，lat/lon 可能为 0
        if (latitude == 0f && longitude == 0f) return null;
        return new LatLng(latitude, longitude);
    }

    /**
     * Add a user-created test point (timestamped marker) into the trajectory proto.
     * This ensures the marker is uploaded together with the rest of the trajectory data.
     *
     * @param position  GNSS lat/lon at the time of the marker tap
     * @param altitudeM altitude in metres (if unavailable, pass 0)
     * @param index     user-visible marker number
     */
    public boolean addTestPoint(@NonNull LatLng position, double altitudeM, int index) {
        if (trajectory == null) {
            Log.w("SensorFusion", "Trajectory not initialized; skip adding test point");
            return false;
        }

        long relativeTimestamp = System.currentTimeMillis() - absoluteStartTime;
        String floorLabel = String.valueOf(getCurrentFloor());

        Traj.GNSSPosition location = Traj.GNSSPosition.newBuilder()
                .setRelativeTimestamp(relativeTimestamp)
                .setLatitude(position.latitude)
                .setLongitude(position.longitude)
                .setAltitude(altitudeM)
                .setFloor(floorLabel)
                .build();

        Traj.TestPoint testPoint = Traj.TestPoint.newBuilder()
                .setIndex(index)
                .setPosition(location)
                .build();

        trajectory.addTestPoints(testPoint);
        Log.d("MARKER", "PROTO test_points_count=" + trajectory.getTestPointsCount());
        testPoints.add(testPoint);
        return true;
    }

    public void clearTestPoints() {
        testPoints.clear();
    }

    // 轨迹里的 relative_timestamp 必须是“相对 start_timestamp 的毫秒”
// 你们工程里 start_timestamp 对应 bootTime，所以这里用 uptimeMillis - bootTime
    public long getRelativeTimestampMs() {
        if (bootTime == 0) return 0;
        return SystemClock.uptimeMillis() - bootTime;
    }

    public double getCurrentGnssAltitude() {
        return gnssAltitude;
    }

    private long resolveGnssObservationAgeMs(@NonNull Location location, long nowMs) {
        long locationTimeMs = location.getTime();
        if (locationTimeMs <= 0L) {
            return -1L;
        }
        return Math.max(0L, nowMs - locationTimeMs);
    }

    private void logAbsoluteFusionTrace(
            @NonNull String source,
            long timestampMs,
            long observationAgeMs,
            double latitudeDeg,
            double longitudeDeg,
            @Nullable double[] localFix,
            @Nullable Integer floorPrior,
            float accuracyMeters
    ) {
        if (!DEBUG_FUSION_TRACE) {
            return;
        }
        double[] displayedLocal = getLocalMetersForLatLng(lastDisplayedFusedMarkerLatLng);
        double[] displayedRawLocal = getLocalMetersForLatLng(lastDisplayedFusedMarkerRawLatLng);
        Log.d(
                "SensorFusion",
                "FUSION_DBG ts=" + timestampMs
                        + " source=" + source
                        + " rawLatLon=" + formatLatLon(latitudeDeg, longitudeDeg)
                        + " localEN=" + formatLocal(localFix)
                        + " floorPrior=" + (floorPrior == null ? "n/a" : floorPrior)
                        + " currentFloor=" + getCurrentFloor()
                        + " accuracyM=" + formatDebugDouble(accuracyMeters)
                        + " obsAgeMs=" + observationAgeMs
                        + " fused=" + formatFusedPose(latestFusedPose)
                        + " displayedRaw=" + formatLatLng(lastDisplayedFusedMarkerRawLatLng)
                        + " displayedRawEN=" + formatLocal(displayedRawLocal)
                        + " displayedMarker=" + formatLatLng(lastDisplayedFusedMarkerLatLng)
                        + " displayedMarkerEN=" + formatLocal(displayedLocal)
                        + " displayedAgeMs=" + resolveDisplayedMarkerAgeMs(timestampMs)
                        + " pf=" + buildParticleDebugSummary()
        );
    }

    private void logPdrFusionTrace(
            @Nullable PdrDelta pdrDelta,
            int floor,
            long timestampMs,
            long observationAgeMs
    ) {
        if (!DEBUG_FUSION_TRACE) {
            return;
        }
        float[] pdrPosition = pdrProcessing == null ? null : pdrProcessing.getPDRMovement();
        double[] displayedLocal = getLocalMetersForLatLng(lastDisplayedFusedMarkerLatLng);
        Log.d(
                "SensorFusion",
                "FUSION_DBG ts=" + timestampMs
                        + " source=PDR"
                        + " rawLatLon=n/a"
                        + " localEN=" + formatFloatLocal(pdrPosition)
                        + " stepLenM=" + formatDebugDouble(pdrDelta == null ? Double.NaN : pdrDelta.getStepLengthMeters())
                        + " deltaHeadingRad=" + formatDebugDouble(pdrDelta == null ? Double.NaN : pdrDelta.getDeltaHeadingRad())
                        + " heightDeltaM=" + formatDebugDouble(pdrDelta == null ? Double.NaN : pdrDelta.getHeightDeltaMeters())
                        + " currentFloor=" + floor
                        + " obsAgeMs=" + observationAgeMs
                        + " fused=" + formatFusedPose(latestFusedPose)
                        + " displayedMarker=" + formatLatLng(lastDisplayedFusedMarkerLatLng)
                        + " displayedMarkerEN=" + formatLocal(displayedLocal)
                        + " displayedAgeMs=" + resolveDisplayedMarkerAgeMs(timestampMs)
                        + " pf=" + buildParticleDebugSummary()
        );
    }

    private long resolveDisplayedMarkerAgeMs(long referenceTimestampMs) {
        if (lastDisplayedFusedMarkerTimestampMs <= 0L) {
            return -1L;
        }
        return Math.max(0L, referenceTimestampMs - lastDisplayedFusedMarkerTimestampMs);
    }

    @NonNull
    private String buildParticleDebugSummary() {
        if (particleFilterEngine == null) {
            return "particleCount=0";
        }
        List<Particle> particles = particleFilterEngine.snapshotParticlesForTesting();
        if (particles.isEmpty()) {
            return "particleCount=0";
        }
        Particle bestParticle = null;
        for (Particle particle : particles) {
            if (bestParticle == null || particle.getWeight() > bestParticle.getWeight()) {
                bestParticle = particle;
            }
        }
        return "particleCount=" + particles.size()
                + " bestParticle=" + formatParticle(bestParticle)
                + " weightedMean=" + formatFusedPose(latestFusedPose)
                + " wallReject=" + particleFilterEngine.getLastPredictWallRejectCount()
                + " floorConstraintReject=" + particleFilterEngine.getLastPredictFloorConstraintRejectCount()
                + " reanchor=" + particleFilterEngine.wasLastAbsoluteFixReanchored();
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
    private String formatParticle(@Nullable Particle particle) {
        if (particle == null) {
            return "n/a";
        }
        return "{e=" + formatDebugDouble(particle.getX())
                + ",n=" + formatDebugDouble(particle.getY())
                + ",floor=" + particle.getFloor()
                + ",w=" + formatDebugDouble(particle.getWeight())
                + "}";
    }

    @NonNull
    private String formatLatLng(@Nullable LatLng latLng) {
        if (latLng == null) {
            return "n/a";
        }
        return formatLatLon(latLng.latitude, latLng.longitude);
    }

    @NonNull
    private String formatLatLon(double latitudeDeg, double longitudeDeg) {
        if (!Double.isFinite(latitudeDeg) || !Double.isFinite(longitudeDeg)) {
            return "n/a";
        }
        return "{lat=" + formatDebugDouble(latitudeDeg)
                + ",lon=" + formatDebugDouble(longitudeDeg)
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
    private String formatFloatLocal(@Nullable float[] localMeters) {
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


    /**
     * Creates a {@link Traj.SensorInfo} objects from the specified sensor's data.
     *
     * @param sensor    MovementSensor objects with populated sensorInfo fields
     * @return          Traj.SensorInfo object to be used in building the trajectory
     *
     * @see Traj            Trajectory object used for communication with the server
     * @see MovementSensor  class abstracting SensorManager based sensors
     */
    private Traj.SensorInfo.Builder createInfoBuilder(MovementSensor sensor) {
        return Traj.SensorInfo.newBuilder()
                .setName(sensor.sensorInfo.getName())
                .setVendor(sensor.sensorInfo.getVendor())
                .setResolution(sensor.sensorInfo.getResolution())
                .setPower(sensor.sensorInfo.getPower())
                .setVersion(sensor.sensorInfo.getVersion())
                .setType(sensor.sensorInfo.getType());
    }

    /**
     * Timer task to record data with the desired frequency in the trajectory class.
     *
     * Inherently threaded, runnables are created in {@link SensorFusion#startRecording()} and
     * destroyed in {@link SensorFusion#stopRecording()}.
     */
    private class storeDataInTrajectory extends TimerTask {
        public void run() {
            // Store IMU and magnetometer data in Trajectory class
            long relativeTimestamp = SystemClock.uptimeMillis() - bootTime;

            Traj.Vector3 accVector = Traj.Vector3.newBuilder()
                    .setX(acceleration[0])
                    .setY(acceleration[1])
                    .setZ(acceleration[2])
                    .build();

            Traj.Vector3 gyrVector = Traj.Vector3.newBuilder()
                    .setX(angularVelocity[0])
                    .setY(angularVelocity[1])
                    .setZ(angularVelocity[2])
                    .build();

            Traj.Quaternion rotationQuat = Traj.Quaternion.newBuilder()
                    .setX(rotation[0])
                    .setY(rotation[1])
                    .setZ(rotation[2])
                    .setW(rotation[3])
                    .build();

            trajectory.addImuData(Traj.IMUReading.newBuilder()
                    .setRelativeTimestamp(relativeTimestamp)
                    .setAcc(accVector)
                    .setGyr(gyrVector)
                    .setRotationVector(rotationQuat)
                    .setStepCount(stepCounter));

            Traj.Vector3 magVector = Traj.Vector3.newBuilder()
                    .setX(magneticField[0])
                    .setY(magneticField[1])
                    .setZ(magneticField[2])
                    .build();

            trajectory.addMagnetometerData(Traj.MagnetometerReading.newBuilder()
                    .setRelativeTimestamp(relativeTimestamp)
                    .setMag(magVector));

            // Divide timer with a counter for storing data every 1 second
            if (counter == 99) {
                counter = 0;
                // Store pressure and light data
                if (barometerSensor.sensor != null) {
                    long timestamp = SystemClock.uptimeMillis() - bootTime;
                    trajectory.addPressureData(Traj.BarometerReading.newBuilder()
                                    .setPressure(pressure)
                                    .setRelativeTimestamp(timestamp))
                            .addLightData(Traj.LightReading.newBuilder()
                                    .setLight(light)
                                    .setRelativeTimestamp(timestamp)
                                    .build());
                }

                // Divide the timer for storing AP data every 5 seconds
                if (secondCounter == 4) {
                    secondCounter = 0;
                    //Current Wifi Object
                    Wifi currentWifi = wifiProcessor.getCurrentWifiData();
                    trajectory.addApsData(Traj.WiFiAPData.newBuilder()
                            .setMac(currentWifi.getBssid())
                            .setSsid(currentWifi.getSsid())
                            .setFrequency(currentWifi.getFrequency())
                            .setRttEnabled(currentWifi.isRttSupported()));
                }
                else {
                    secondCounter++;
                }
            }
            else {
                counter++;
            }

        }
    }

    //endregion

}
