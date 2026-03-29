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
import com.openpositioning.PositionMe.utils.IndoorMapManager;
import com.openpositioning.PositionMe.utils.MapConstraintReadiness;
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
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
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
    private static final String MOTION_DIAG_TAG = "MOTION_DIAG";
    private static final String SENSOR_FLOOR_DIAG_TAG = "SensorFloorDiag";
    private static final long ARROW_DBG_INTERVAL_MS = 500L;
    private static final long DECLINATION_REFRESH_INTERVAL_MS = 10 * 60 * 1000L;
    private static final String VENUE_KEY_NUCLEUS = "nucleus";
    private static final String VENUE_KEY_LIBRARY = "library";
    private static final String VENUE_KEY_MURCHISON = "murchison";
    private static final double MAX_LIFT_HORIZONTAL_TRAVEL_M = 2.0;
    private static final double MIN_STAIRS_HORIZONTAL_TRAVEL_M = 0.45;
    // Modified: cumulative barometer travel required before committing a floor transition.
    private static final float CUMULATIVE_FLOOR_CHANGE_THRESHOLD_M =
            CumulativeFloorTracker.CUMULATIVE_FLOOR_CHANGE_THRESHOLD_M;
    private static final int WIFI_MULTI_FLOOR_SANITY_GAP = 2;
    private static final int WIFI_BOOTSTRAP_REQUIRED_CONFIRMATIONS = 3;
    private static final long WIFI_BOOTSTRAP_CONSENSUS_WINDOW_MS = 12_000L;
    private static final int GROUND_FLOOR_WIFI_BOOTSTRAP_REQUIRED_CONFIRMATIONS = 5;
    private static final long GROUND_FLOOR_WIFI_BOOTSTRAP_REQUIRED_DURATION_MS = 6_000L;
    private static final int INITIAL_ABSOLUTE_FLOOR_LOCK_REQUIRED_CONFIRMATIONS = 3;
    private static final long INITIAL_ABSOLUTE_FLOOR_LOCK_REQUIRED_DURATION_MS = 3_000L;
    private static final int FAST_INITIAL_ABSOLUTE_FLOOR_LOCK_REQUIRED_CONFIRMATIONS = 2;
    private static final long FAST_INITIAL_ABSOLUTE_FLOOR_LOCK_REQUIRED_DURATION_MS = 1_000L;
    private static final long MIN_STEP_EVENT_INTERVAL_MS = 160L;
    private static final int ELEVATOR_ON_CONFIRMATION_SAMPLES = 2;
    private static final long ELEVATOR_ON_CONFIRMATION_WINDOW_MS = 1_500L;
    private static final int ELEVATOR_OFF_CONFIRMATION_SAMPLES = 6;
    private static final int ELEVATOR_OFF_NO_LIFT_CONFIRMATION_SAMPLES = 2;
    private static final long ELEVATOR_HOLD_MS = 2_500L;
    private static final long LIFT_CONTEXT_FUSED_POSE_MAX_AGE_MS = 20_000L;
    private static final long LIFT_CONTEXT_WIFI_FIX_MAX_AGE_MS = 12_000L;
    private static final double LIFT_CONTEXT_MIN_CONFIDENCE = 0.20;
    private static final float ELEVATOR_BAROMETER_VERTICAL_SPEED_THRESHOLD_MPS = 0.18f;
    private static final float ELEVATOR_BAROMETER_VERTICAL_TRAVEL_THRESHOLD_M = 0.75f;
    private static final float ELEVATOR_BAROMETER_NET_VERTICAL_DISPLACEMENT_THRESHOLD_M = 1.90f;
    private static final long ELEVATOR_BAROMETER_MIN_WINDOW_MS = 2_000L;
    private static final long ELEVATOR_BAROMETER_SIGNAL_MAX_AGE_MS = 1_500L;
    private static final long ELEVATOR_BAROMETER_WINDOW_MS = 4_000L;
    private static final long ELEVATOR_STEP_SUPPRESSION_WINDOW_MS = 3_000L;
    private static final long ELEVATOR_STEP_SUPPRESSION_RECENT_STEP_MAX_AGE_MS = 1_200L;
    private static final int ELEVATOR_STEP_SUPPRESSION_MIN_STEPS = 3;
    private static final double STRONG_BAROMETER_TRANSITION_TOLERANCE_M = 4.0;
    private static final double ELEVATOR_ABSOLUTE_FIX_REJECT_DISTANCE_M = 5.0;
    private static final long ELEVATOR_ABSOLUTE_FIX_COOLDOWN_MS = 6_000L;
    private static final long FLOOR_ONLY_RESYNC_WINDOW_MS = 8_000L;
    private static final long DISPLAY_FLOOR_RESET_WINDOW_MS = 2_000L;
    private static final float FLOOR_ONLY_RESYNC_ABSOLUTE_FIX_STD_M = 12.0f;
    private static final int ABSOLUTE_FLOOR_STABLE_CONSENSUS_REQUIRED_CONFIRMATIONS = 4;
    private static final long ABSOLUTE_FLOOR_STABLE_CONSENSUS_REQUIRED_DURATION_MS = 8_000L;
    private static final long ABSOLUTE_FLOOR_STABLE_CONSENSUS_MAX_GAP_MS = 3_500L;
    private static final float BAROMETER_ELEVATOR_SPEED_SMOOTHING_ALPHA = 0.35f;
    private static final int ELEVATOR_PANIC_EXIT_STEP_THRESHOLD = 5;
    private static final float ELEVATOR_PANIC_EXIT_STATIONARY_SPEED_THRESHOLD_MPS = 0.05f;
    private static final long ELEVATOR_PANIC_EXIT_STATIONARY_WINDOW_MS = 5_000L;
    private static final long ELEVATOR_SESSION_TIMEOUT_MS = 60_000L;
    private static final float ELEVATOR_SESSION_FILTER_ALPHA = 0.35f;
    private static final float ELEVATOR_SESSION_MAX_VERTICAL_SPEED_MPS = 2.2f;
    private static final float ELEVATOR_SESSION_FLOOR_SNAP_BONUS_RATIO = 0.35f;
    private static final double ELEVATOR_NEAR_LIFT_TOLERANCE_M = 1.75;
    private static final long WEAK_ELEVATOR_SUPPRESSION_WINDOW_MS = 20_000L;
    private static final int CONSISTENT_SUPPRESSED_WIFI_FLOOR_REQUIRED_CONFIRMATIONS = 3;
    private static final long CONSISTENT_SUPPRESSED_WIFI_FLOOR_MAX_GAP_MS = 5_000L;
    private static final long PENDING_ELEVATOR_SUPPRESSION_ESCAPE_WINDOW_MS = 6_000L;
    private static final double WRONG_LOCK_RECOVERY_MAX_CONFIDENCE = 0.45;
    private static final long POST_LIFT_DESTINATION_FIX_WINDOW_MS = 12_000L;
    private static final long BOOTSTRAP_ABSOLUTE_FIX_MAX_AGE_MS = 20_000L;

    enum TransitionPreference {
        LIFT_ONLY,
        STAIRS_ONLY,
        ANY_AVAILABLE
    }

    enum TransitionConstraintAvailability {
        COMPLETE("constraints_ready"),
        MAP_UNAVAILABLE("map_constraints_unavailable"),
        FLOOR_DATA_UNAVAILABLE("floor_constraints_not_ready"),
        TRANSITION_GEOMETRY_UNAVAILABLE("transition_geometry_missing");

        @NonNull
        final String reasonCode;

        TransitionConstraintAvailability(@NonNull String reasonCode) {
            this.reasonCode = reasonCode;
        }
    }

    private static final class PendingConstrainedAbsoluteFix {
        @NonNull
        final AbsoluteFix absoluteFix;
        @Nullable
        final Integer initializationFloor;
        @Nullable
        final Integer floorPrior;
        @NonNull
        final String debugSource;
        final long observationAgeMs;

        PendingConstrainedAbsoluteFix(
                @NonNull AbsoluteFix absoluteFix,
                @Nullable Integer initializationFloor,
                @Nullable Integer floorPrior,
                @NonNull String debugSource,
                long observationAgeMs
        ) {
            this.absoluteFix = absoluteFix;
            this.initializationFloor = initializationFloor;
            this.floorPrior = floorPrior;
            this.debugSource = debugSource;
            this.observationAgeMs = observationAgeMs;
        }
    }

    // Temporary diagnostic snapshot for recording-screen debug UI and runtime trace logs.
    public static final class MotionDebugSnapshot {
        public final boolean stationary;
        public final int stationaryWindowSampleCount;
        public final double stationaryWindowMeanMagnitude;
        public final int stationaryWindowSpikeCount;
        public final long lastStationaryTransitionTimestampMs;
        @NonNull public final String lastStationaryTransitionReason;
        public final boolean motionResumeActive;
        public final long motionResumeWindowUntilMs;
        public final long lastStepDetectorEventTimestampMs;
        public final long lastAcceptedStepTimestampMs;
        @NonNull public final String lastStepDecision;
        public final long lastAbsoluteFixReceivedTimestampMs;
        public final long lastAcceptedAbsoluteFixTimestampMs;
        @NonNull public final String lastAbsoluteFixDecision;
        @NonNull public final String lastRejectReason;
        @NonNull public final String lastBlockReason;
        @NonNull public final String lastPoseAdvanceSource;
        public final long lastPoseAdvanceTimestampMs;
        public final long lastDisplayedFusedMarkerTimestampMs;
        public final long lastFusedPoseTimestampMs;
        public final int currentAbsoluteFloor;
        public final int currentRelativeFloor;
        @NonNull public final String floorConsensus;
        @NonNull public final String floorSource;
        @NonNull public final String elevatorGate;
        @NonNull public final String liftTransferState;
        @NonNull public final String floorAnchorState;
        @NonNull public final String postLiftState;

        MotionDebugSnapshot(
                boolean stationary,
                int stationaryWindowSampleCount,
                double stationaryWindowMeanMagnitude,
                int stationaryWindowSpikeCount,
                long lastStationaryTransitionTimestampMs,
                @NonNull String lastStationaryTransitionReason,
                boolean motionResumeActive,
                long motionResumeWindowUntilMs,
                long lastStepDetectorEventTimestampMs,
                long lastAcceptedStepTimestampMs,
                @NonNull String lastStepDecision,
                long lastAbsoluteFixReceivedTimestampMs,
                long lastAcceptedAbsoluteFixTimestampMs,
                @NonNull String lastAbsoluteFixDecision,
                @NonNull String lastRejectReason,
                @NonNull String lastBlockReason,
                @NonNull String lastPoseAdvanceSource,
                long lastPoseAdvanceTimestampMs,
                long lastDisplayedFusedMarkerTimestampMs,
                long lastFusedPoseTimestampMs,
                int currentAbsoluteFloor,
                int currentRelativeFloor,
                @NonNull String floorConsensus,
                @NonNull String floorSource,
                @NonNull String elevatorGate,
                @NonNull String liftTransferState,
                @NonNull String floorAnchorState,
                @NonNull String postLiftState
        ) {
            this.stationary = stationary;
            this.stationaryWindowSampleCount = stationaryWindowSampleCount;
            this.stationaryWindowMeanMagnitude = stationaryWindowMeanMagnitude;
            this.stationaryWindowSpikeCount = stationaryWindowSpikeCount;
            this.lastStationaryTransitionTimestampMs = lastStationaryTransitionTimestampMs;
            this.lastStationaryTransitionReason = lastStationaryTransitionReason;
            this.motionResumeActive = motionResumeActive;
            this.motionResumeWindowUntilMs = motionResumeWindowUntilMs;
            this.lastStepDetectorEventTimestampMs = lastStepDetectorEventTimestampMs;
            this.lastAcceptedStepTimestampMs = lastAcceptedStepTimestampMs;
            this.lastStepDecision = lastStepDecision;
            this.lastAbsoluteFixReceivedTimestampMs = lastAbsoluteFixReceivedTimestampMs;
            this.lastAcceptedAbsoluteFixTimestampMs = lastAcceptedAbsoluteFixTimestampMs;
            this.lastAbsoluteFixDecision = lastAbsoluteFixDecision;
            this.lastRejectReason = lastRejectReason;
            this.lastBlockReason = lastBlockReason;
            this.lastPoseAdvanceSource = lastPoseAdvanceSource;
            this.lastPoseAdvanceTimestampMs = lastPoseAdvanceTimestampMs;
            this.lastDisplayedFusedMarkerTimestampMs = lastDisplayedFusedMarkerTimestampMs;
            this.lastFusedPoseTimestampMs = lastFusedPoseTimestampMs;
            this.currentAbsoluteFloor = currentAbsoluteFloor;
            this.currentRelativeFloor = currentRelativeFloor;
            this.floorConsensus = floorConsensus;
            this.floorSource = floorSource;
            this.elevatorGate = elevatorGate;
            this.liftTransferState = liftTransferState;
            this.floorAnchorState = floorAnchorState;
            this.postLiftState = postLiftState;
        }
    }

    private static final class LiftAnchor {
        final int floor;
        final double xMeters;
        final double yMeters;
        @NonNull final LatLng latLng;

        LiftAnchor(int floor, double xMeters, double yMeters, @NonNull LatLng latLng) {
            this.floor = floor;
            this.xMeters = xMeters;
            this.yMeters = yMeters;
            this.latLng = latLng;
        }
    }

    private static final class PoseConstraintReport {
        @NonNull
        final String reasonCode;
        final boolean candidatePointLegal;
        final boolean transitionLegal;
        final boolean pointInsideWall;
        final boolean pointOutsideVenueOutline;
        final boolean pathCrossesWallOnPreviousFloor;
        final boolean pathCrossesWallOnCandidateFloor;
        final boolean pathExitsVenueOutline;
        final boolean floorTransitionRejectedByMapGate;
        final int floorForLegalityCheck;
        @Nullable
        final LatLng previousLatLng;
        @Nullable
        final LatLng candidateLatLng;

        PoseConstraintReport(
                @NonNull String reasonCode,
                boolean candidatePointLegal,
                boolean transitionLegal,
                boolean pointInsideWall,
                boolean pointOutsideVenueOutline,
                boolean pathCrossesWallOnPreviousFloor,
                boolean pathCrossesWallOnCandidateFloor,
                boolean pathExitsVenueOutline,
                boolean floorTransitionRejectedByMapGate,
                int floorForLegalityCheck,
                @Nullable LatLng previousLatLng,
                @Nullable LatLng candidateLatLng
        ) {
            this.reasonCode = reasonCode;
            this.candidatePointLegal = candidatePointLegal;
            this.transitionLegal = transitionLegal;
            this.pointInsideWall = pointInsideWall;
            this.pointOutsideVenueOutline = pointOutsideVenueOutline;
            this.pathCrossesWallOnPreviousFloor = pathCrossesWallOnPreviousFloor;
            this.pathCrossesWallOnCandidateFloor = pathCrossesWallOnCandidateFloor;
            this.pathExitsVenueOutline = pathExitsVenueOutline;
            this.floorTransitionRejectedByMapGate = floorTransitionRejectedByMapGate;
            this.floorForLegalityCheck = floorForLegalityCheck;
            this.previousLatLng = previousLatLng;
            this.candidateLatLng = candidateLatLng;
        }

        boolean isAccepted() {
            return "accepted".equals(reasonCode);
        }
    }

    private static final class FloorOnlyResyncPolicy {
        final boolean rejectAbsoluteFix;
        @Nullable
        final Integer effectiveFloorPrior;
        final float effectiveAccuracyMeters;

        FloorOnlyResyncPolicy(
                boolean rejectAbsoluteFix,
                @Nullable Integer effectiveFloorPrior,
                float effectiveAccuracyMeters
        ) {
            this.rejectAbsoluteFix = rejectAbsoluteFix;
            this.effectiveFloorPrior = effectiveFloorPrior;
            this.effectiveAccuracyMeters = effectiveAccuracyMeters;
        }
    }

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
    private static final int STATIONARY_WINDOW_SIZE = 32;
    private static final double STATIONARY_MEAN_THRESHOLD_MPS2 = 0.10;
    private static final double STATIONARY_SPIKE_THRESHOLD_MPS2 = 0.25;
    private static final int STATIONARY_ALLOWED_SPIKES = 1;
    private static final long MOTION_RESUME_WINDOW_MS = 1_500L;
    private static final long STATIONARY_LOG_INTERVAL_MS = 3_000L;
    private static final int CONSTRAINED_POSE_RECOVERY_ITERATIONS = 12;
    private static final double MIN_CONSTRAINED_POSE_PROGRESS_M = 0.05;
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
    private long lastDisplayOrientationTimestampMs = Long.MIN_VALUE;
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
    private int elevatorPositiveDetectionCount;
    private int elevatorNegativeDetectionCount;
    private long elevatorPositiveDetectionWindowStartMs = Long.MIN_VALUE;
    private long lastElevatorPositiveTimestampMs = Long.MIN_VALUE;
    private long elevatorAbsoluteFixRejectUntilMs = Long.MIN_VALUE;
    private float lastElevatorBarometerElevationMeters = Float.NaN;
    private long lastElevatorBarometerTimestampMs = Long.MIN_VALUE;
    private float smoothedBarometerVerticalSpeedMps = 0f;
    private float recentBarometerVerticalTravelMeters = 0f;
    private float recentBarometerWindowStartElevationMeters = Float.NaN;
    private float recentBarometerNetVerticalDisplacementMeters = 0f;
    private long recentBarometerVerticalWindowStartMs = Long.MIN_VALUE;
    private boolean elevatorFloorSessionActive;
    private int elevatorFloorSessionStartAbsoluteFloor;
    private float elevatorFloorSessionStartElevationMeters = Float.NaN;
    private float elevatorFloorSessionFilteredElevationMeters = Float.NaN;
    private long elevatorFloorSessionLastUpdateTimestampMs = Long.MIN_VALUE;
    private long elevatorFloorSessionStartTimestampMs = Long.MIN_VALUE;
    private int elevatorSessionStepCount;
    private long elevatorLowMotionStartTimestampMs = Long.MIN_VALUE;
    private long weakElevatorSuppressionUntilMs = Long.MIN_VALUE;
    @Nullable
    private Integer consistentSuppressedWifiFloorCandidate;
    private int consistentSuppressedWifiFloorCount;
    private long lastConsistentSuppressedWifiFloorTimestampMs = Long.MIN_VALUE;
    @Nullable
    private Integer pendingElevatorSuppressionEscapeFloor;
    private long pendingElevatorSuppressionEscapeUntilMs = Long.MIN_VALUE;
    @NonNull
    private String lastElevatorGate = "init";
    @NonNull
    private String lastLiftTransferState = "inactive";
    @NonNull
    private String lastFloorAnchorState = "unresolved";
    @NonNull
    private String lastPostLiftState = "inactive";
    @Nullable
    private LiftAnchor elevatorFloorSessionSourceLiftAnchor;
    private int postLiftExpectedAbsoluteFloor = Integer.MIN_VALUE;
    private long postLiftDestinationFixWindowUntilMs = Long.MIN_VALUE;
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
    private final ArrayDeque<Double> recentLinearAccelerationMagnitudes = new ArrayDeque<>();
    private double recentLinearAccelerationMagnitudeSum = 0.0;
    private int recentLinearAccelerationSpikeCount = 0;
    private boolean isStationary = true;
    private long lastStationaryStepLogMs = 0L;
    private long lastStationaryAbsoluteFixLogMs = 0L;
    private int lastStationaryWindowSampleCount = 0;
    private double lastStationaryWindowMeanMagnitude = Double.NaN;
    private int lastStationaryWindowSpikeCount = 0;
    private double lastLinearAccelerationMagnitude = Double.NaN;
    private long motionResumeWindowUntilMs = Long.MIN_VALUE;
    private long lastStationaryTransitionTimestampMs = Long.MIN_VALUE;
    @NonNull
    private String lastStationaryTransitionReason = "init";
    private long lastStepDetectorEventTimestampMs = Long.MIN_VALUE;
    private long lastAcceptedStepTimestampMs = Long.MIN_VALUE;
    private final ArrayDeque<Long> recentAcceptedStepTimestampsMs = new ArrayDeque<>();
    @NonNull
    private String lastStepDecision = "init";
    private long lastAbsoluteFixReceivedTimestampMs = Long.MIN_VALUE;
    private long lastAcceptedAbsoluteFixTimestampMs = Long.MIN_VALUE;
    @NonNull
    private String lastAbsoluteFixDecision = "init";
    @NonNull
    private String lastRejectReason = "none";
    @NonNull
    private String lastBlockReason = "none";
    @NonNull
    private String lastPoseAdvanceSource = "none";
    private long lastPoseAdvanceTimestampMs = Long.MIN_VALUE;

    // PDR calculation class
    private PdrProcessing pdrProcessing;
    private ParticleFilterEngine particleFilterEngine;
    private boolean pfInitialized;
    private FusedPose latestFusedPose;
    private CoordinateConverter coordinateConverter;
    @Nullable
    private PendingConstrainedAbsoluteFix pendingConstrainedAbsoluteFix;
    private boolean particleCloudTrustedUnderConstraints;
    private boolean listenersActive;
    private final AbsoluteFloorTransitionResolver absoluteFloorTransitionResolver = new AbsoluteFloorTransitionResolver();
    private boolean hasManualStartLocation;
    private float lastPredictHeadingRad;
    private float lastPredictElevation;
    @NonNull
    private String lastPredictHeadingSource = "device_raw";
    private long lastWaitingForAbsoluteFixLogMs;
    private long lastRecordedFusedPoseTimestampMs;
    // Converts barometer/PDR relative floors into absolute map floor indices.
    private int pdrFloorOffset = 0;
    private boolean isFloorOffsetInitialized = false;
    private float floorHeightOverrideForTesting = Float.NaN;
    private final CumulativeFloorTracker cumulativeFloorTracker = new CumulativeFloorTracker();
    private int floorOnlyResyncAbsoluteFloor = Integer.MIN_VALUE;
    private long floorOnlyResyncWindowUntilMs = Long.MIN_VALUE;
    private int pendingDisplayFloorResetAbsoluteFloor = Integer.MIN_VALUE;
    private long pendingDisplayFloorResetUntilMs = Long.MIN_VALUE;
    @Nullable
    private LatLng pendingDisplayFloorResetLandingLatLng;
    @NonNull
    private String pendingDisplayFloorResetAnchorSource = "none";
    private int committedDisplayFloorAbsolute = Integer.MIN_VALUE;
    private int pendingCommittedDisplayFloorAbsolute = Integer.MIN_VALUE;
    private boolean floorSwitchPending;
    @NonNull
    private String lastFloorSwitchBlockReason = "none";
    @Nullable
    private Integer pendingStableAbsoluteFloorCandidate;
    private int pendingStableAbsoluteFloorCount;
    private long pendingStableAbsoluteFloorFirstTimestampMs = Long.MIN_VALUE;
    private long pendingStableAbsoluteFloorLastTimestampMs = Long.MIN_VALUE;
    private boolean blockHistoricalPoseFloorSeedUntilTrustedFix;
    private boolean provisionalFloorBootstrapActive;
    private int provisionalFloorBootstrapFloor = Integer.MIN_VALUE;
    @NonNull
    private String provisionalFloorBootstrapSource = "none";
    @NonNull
    private String lastFloorConsensus = "init";
    @NonNull
    private String lastFloorSource = "relative_only";
    @Nullable
    private Integer pendingWifiBootstrapFloor;
    private int pendingWifiBootstrapCount;
    private long pendingWifiBootstrapFirstTimestampMs = Long.MIN_VALUE;
    private long lastWifiBootstrapCandidateTimestampMs = Long.MIN_VALUE;
    private long lastWifiScanWallClockMs = -1L;
    private boolean allowProvisionalFloorReconcileForCurrentFix;
    private int provisionalFloorReconcileTargetFloor = Integer.MIN_VALUE;
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
        resetElevatorState();
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
        resetStationaryMotionGate();
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
        this.pendingConstrainedAbsoluteFix = null;
        this.particleCloudTrustedUnderConstraints = false;
        this.hasManualStartLocation = false;
        this.lastPredictHeadingRad = 0f;
        this.lastPredictHeadingSource = "device_raw";
        this.lastPredictElevation = 0f;
        this.cumulativeFloorTracker.reset();
        this.lastRecordedFusedPoseTimestampMs = -1L;
        this.lastDisplayOrientationTimestampMs = Long.MIN_VALUE;
        resetStationaryMotionGate();

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
                    updateElevatorFloorSessionEstimate(currentTime);
                    updateElevatorBarometerSignal(currentTime);
                    updateCumulativeBarometerFloorEstimate(currentTime);
                }
                break;

            case Sensor.TYPE_GYROSCOPE:
                angularVelocity[0] = sensorEvent.values[0];
                angularVelocity[1] = sensorEvent.values[1];
                angularVelocity[2] = sensorEvent.values[2];
                break;

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
                updateStationaryMotionGate(accelMagFiltered, currentTime);

//                // Debug logging
//                Log.v("SensorFusion",
//                        "Added new linear accel magnitude: " + accelMagFiltered
//                                + "; accelMagnitude size = " + accelMagnitude.size());

                updateElevatorState(
                        pdrProcessing.estimateElevator(gravity, filteredAcc)
                                || isBarometerSuggestingElevator(currentTime),
                        currentTime
                );
                break;

            case Sensor.TYPE_GRAVITY:
                gravity[0] = sensorEvent.values[0];
                gravity[1] = sensorEvent.values[1];
                gravity[2] = sensorEvent.values[2];

                // Possibly log gravity values if needed
                //Log.v("SensorFusion", "Gravity: " + Arrays.toString(gravity));

                updateElevatorState(
                        pdrProcessing.estimateElevator(gravity, filteredAcc)
                                || isBarometerSuggestingElevator(currentTime),
                        currentTime
                );
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
                updateDisplayOrientation(rotationVectorDCM);
                lastDisplayOrientationTimestampMs = SystemClock.elapsedRealtime();
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
                long stepEventAgeMs = Math.max(
                        0L,
                        (SystemClock.elapsedRealtimeNanos() - sensorEvent.timestamp) / 1_000_000L
                );
                long sinceLastStepMs = lastStepTime <= 0L ? -1L : currentTime - lastStepTime;
                boolean stationaryBeforeStep = isStationary;
                lastStepDetectorEventTimestampMs = currentTime;
                logMotionDiagnostic(
                        "step_event received ts=" + currentTime
                                + " ageMs=" + stepEventAgeMs
                                + " sinceLastStepMs=" + sinceLastStepMs
                                + " stationaryBefore=" + stationaryBeforeStep
                                + " pfInitialized=" + pfInitialized
                                + " accelMagnitudeSamples=" + accelMagnitude.size()
                                + " windowSamples=" + lastStationaryWindowSampleCount
                                + " windowMeanAbs=" + formatDebugDouble(lastStationaryWindowMeanMagnitude)
                                + " windowSpikes=" + lastStationaryWindowSpikeCount
                );
                // Motion-resume diagnostics: a detector edge is enough to break the stationary
                // freeze immediately, even if the step is later ignored by debounce logic.
                activateMotionResumeWindow(
                        currentTime,
                        "step_detector_signal",
                        lastLinearAccelerationMagnitude
                );


                if (currentTime - lastStepTime < MIN_STEP_EVENT_INTERVAL_MS) {
                    Log.e("SensorFusion", "Ignoring step event, too soon after last step event:" + (currentTime - lastStepTime) + " ms");
                    recordStepDecision(
                            currentTime,
                            false,
                            "ignored_min_step_interval",
                            stepEventAgeMs,
                            sinceLastStepMs,
                            null
                    );
                    // Ignore rapid successive step events
                    break;
                }

                else {
                    markConfirmedStepAsMovement(currentTime);
                    lastStepTime = currentTime;
                    noteElevatorStepAndMaybePanicExit(currentTime);
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
                    float heightDeltaMeters = cumulativeFloorTracker.getPendingHeightDeltaMeters();
                    PdrDelta pdrDelta = this.pdrProcessing.buildStepDelta(
                            this.accelMagnitude,
                            deltaHeadingRad,
                            heightDeltaMeters
                    );
                    float[] newCords = this.pdrProcessing.applyStepDelta(pdrDelta, currentHeadingRad);
                    // Clear the accelMagnitude after using it
                    this.accelMagnitude.clear();
                    recordStepDecision(
                            currentTime,
                            true,
                            "accepted",
                            stepEventAgeMs,
                            sinceLastStepMs,
                            pdrDelta
                    );
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
            long requestTimestampMs = System.currentTimeMillis();
            boolean motionResumeActive = isMotionResumeWindowActive(
                    motionResumeWindowUntilMs,
                    requestTimestampMs
            );
            if (pfInitialized && isStationary && isFloorOffsetInitialized && !motionResumeActive) {
                recordPoseBlocked("WIFI:request_skipped_stationary");
                logMotionDiagnostic(
                        "wifi_request skipped reason=stationary_freeze"
                                + " ts=" + requestTimestampMs
                                + " pfInitialized=" + pfInitialized
                                + " stationary=" + isStationary
                                + " floorOffsetInitialized=" + isFloorOffsetInitialized
                );
                logStationaryAbsoluteFixSuppression("WIFI");
                return;
            }
            if (motionResumeActive) {
                logMotionDiagnostic(
                        "wifi_request allowed reason=motion_resume"
                                + " ts=" + requestTimestampMs
                                + " motionResumeUntilMs=" + motionResumeWindowUntilMs
                );
            }
            final long wifiObservationTimestampMs = lastWifiScanWallClockMs > 0L
                    ? lastWifiScanWallClockMs
                    : requestTimestampMs;
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
                public void onSuccess(LatLng wifiLocation, @Nullable Integer floor) {
                    if (wifiLocation == null) {
                        return;
                    }
                    Log.i(
                            "SensorFusion",
                            "WiFi absolute fix received lat="
                                    + wifiLocation.latitude
                                    + " lon="
                                    + wifiLocation.longitude
                                    + " floor="
                                    + (floor == null ? "unknown" : floor)
                                    + " pfInitialized="
                                    + pfInitialized
                                    + " floorOffsetInitialized="
                                    + isFloorOffsetInitialized
                    );
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
                    if (floor == null) {
                        logInfoSafely("SensorFusion", "WiFi floor missing; skipping floor bootstrap candidate");
                    }
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
    public LatLng getLatLngWifiPositioning() {
        return this.wiFiPositioning == null ? null : this.wiFiPositioning.getWifiLocation();
    }

    /**
     * Method to get current floor the user is at, obtained using WiFiPositioning
     * @see WiFiPositioning for WiFi positioning
     * @return Current floor user is at using WiFiPositioning
     */
    @Nullable
    public Integer getWifiFloor(){
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
            @Nullable Integer initializationFloor,
            @Nullable Integer floorPrior,
            long timestampMs,
            float accuracyMeters,
            @NonNull String debugSource,
            long observationAgeMs
    ) {
        recordAbsoluteFixReceived(
                debugSource,
                timestampMs,
                latitudeDeg,
                longitudeDeg,
                floorPrior,
                accuracyMeters,
                observationAgeMs
        );
        expirePostLiftDestinationFixWindowIfNeeded(timestampMs);
        Integer sanitizedFloorPrior = sanitizeReportedAbsoluteFloorPrior(floorPrior, debugSource);
        FloorOnlyResyncPolicy floorOnlyResyncPolicy = resolveFloorOnlyResyncPolicy(
                debugSource,
                sanitizedFloorPrior,
                accuracyMeters,
                timestampMs
        );
        if (floorOnlyResyncPolicy.rejectAbsoluteFix) {
            recordAbsoluteFixDecision(
                    debugSource,
                    timestampMs,
                    false,
                    "suppressed_floor_resync_mismatched_floor"
            );
            return;
        }
        sanitizedFloorPrior = floorOnlyResyncPolicy.effectiveFloorPrior;
        accuracyMeters = floorOnlyResyncPolicy.effectiveAccuracyMeters;
        Integer bootstrapReadyFloorPrior = resolveBootstrapReadyFloorPrior(
                debugSource,
                sanitizedFloorPrior,
                accuracyMeters,
                timestampMs
        );
        Integer effectiveInitializationFloor = resolveInitializationFloorForAbsoluteFix(
                initializationFloor,
                bootstrapReadyFloorPrior
        );
        if (!saveRecording) {
            recordAbsoluteFixDecision(debugSource, timestampMs, false, "ignored_not_recording");
            return;
        }
        if (shouldFreezeAbsoluteFix(debugSource, sanitizedFloorPrior)) {
            recordAbsoluteFixDecision(debugSource, timestampMs, false, "suppressed_stationary_freeze");
            logStationaryAbsoluteFixSuppression(debugSource);
            return;
        }

        if (this.particleFilterEngine == null) {
            this.particleFilterEngine = createParticleFilterEngine();
        }

        updateGeomagneticDeclination(latitudeDeg, longitudeDeg, 0f, timestampMs);
        CoordinateConverter converter = getOrCreateCoordinateConverter(latitudeDeg, longitudeDeg);
        if (converter == null) {
            recordAbsoluteFixDecision(debugSource, timestampMs, false, "ignored_no_coordinate_converter");
            return;
        }
        double[] localFix = converter.toLocalMeters(latitudeDeg, longitudeDeg);
        revalidateParticleCloudAgainstReadyConstraints();
        boolean bootstrapOverrideElevatorSuppression = shouldAllowBootstrapAbsoluteFix(
                latitudeDeg,
                longitudeDeg,
                sanitizedFloorPrior,
                timestampMs,
                observationAgeMs,
                debugSource
        );
        if (shouldRejectAbsoluteFixDuringElevatorSuppression(
                latitudeDeg,
                longitudeDeg,
                localFix,
                sanitizedFloorPrior,
                timestampMs,
                debugSource,
                bootstrapOverrideElevatorSuppression
        )) {
            recordAbsoluteFixDecision(debugSource, timestampMs, false, "suppressed_elevator_absolute_fix");
            return;
        }
        if (bootstrapOverrideElevatorSuppression) {
            logInfoSafely(
                    "SensorFusion",
                    debugSource + ":bootstrap_allowed_despite_elevator_suppression"
                            + " ts=" + timestampMs
                            + " obsAgeMs=" + observationAgeMs
                            + " floorPrior=" + (sanitizedFloorPrior == null ? "n/a" : sanitizedFloorPrior)
            );
        }
        Integer acceptedFloorPrior = resolveAcceptedAbsoluteFloorPrior(
                latitudeDeg,
                longitudeDeg,
                bootstrapReadyFloorPrior,
                timestampMs
        );
        boolean shouldReconcileProvisionalFloor = shouldApplyProvisionalFloorReconcile(
                debugSource,
                acceptedFloorPrior
        );
        allowProvisionalFloorReconcileForCurrentFix = shouldReconcileProvisionalFloor;
        provisionalFloorReconcileTargetFloor = shouldReconcileProvisionalFloor && acceptedFloorPrior != null
                ? acceptedFloorPrior
                : Integer.MIN_VALUE;
        boolean forceWifiFloorBootstrap = shouldForceWifiFloorBootstrap(
                debugSource,
                acceptedFloorPrior,
                pfInitialized,
                isFloorOffsetInitialized,
                pendingWifiBootstrapCount,
                accuracyMeters
        );
        if (forceWifiFloorBootstrap) {
            absoluteFloorTransitionResolver.reset();
            resetWifiFloorBootstrapConsensus();
        }
        maybeCalibrateFloorOffset(
                acceptedFloorPrior,
                acceptedFloorPrior != null,
                timestampMs
        );
        maybeSetInitialPositionIfAbsent(this.trajectory, latitudeDeg, longitudeDeg);
        if (!pfInitialized
                && effectiveInitializationFloor == null
                && acceptedFloorPrior == null) {
            pendingConstrainedAbsoluteFix = null;
            if ("GNSS".equals(debugSource)) {
                logInfoSafely(
                        "SensorFusion",
                        "GNSS:bootstrap_rejected_invalid_observation"
                                + " reason=no_floor_seed"
                                + " ts=" + timestampMs
                );
            }
            recordAbsoluteFixDecision(
                    debugSource,
                    timestampMs,
                    false,
                    "GNSS".equals(debugSource)
                            ? "bootstrap_rejected_invalid_observation"
                            : "ignored_unknown_floor_seed"
            );
            return;
        }
        boolean degradedBootstrapMapNotReady = false;
        if (shouldDeferConstrainedAbsoluteFix(effectiveInitializationFloor, acceptedFloorPrior)) {
            cachePendingConstrainedAbsoluteFix(
                    latitudeDeg,
                    longitudeDeg,
                    effectiveInitializationFloor,
                    sanitizedFloorPrior,
                    timestampMs,
                    accuracyMeters,
                    debugSource,
                    observationAgeMs
            );
            if (!pfInitialized
                    && ("WIFI".equals(debugSource) || "GNSS".equals(debugSource))) {
                if (canBootstrapParticleFilterWithoutReadyConstraints(
                        localFix,
                        latitudeDeg,
                        longitudeDeg,
                        effectiveInitializationFloor,
                        observationAgeMs,
                        debugSource
                )) {
                    degradedBootstrapMapNotReady = true;
                    logInfoSafely(
                            "SensorFusion",
                            "ABSOLUTE:init_degraded_bootstrap_map_not_ready"
                                    + " source=" + debugSource
                                    + " floorSeed="
                                    + (effectiveInitializationFloor == null ? "n/a" : effectiveInitializationFloor)
                                    + " acceptedFloor="
                                    + (acceptedFloorPrior == null ? "n/a" : acceptedFloorPrior)
                                    + " ts=" + timestampMs
                    );
                    if ("GNSS".equals(debugSource) && acceptedFloorPrior == null) {
                        logInfoSafely(
                                "SensorFusion",
                                "GNSS:bootstrap_unknown_floor_allowed"
                                        + " floorSeed="
                                        + (effectiveInitializationFloor == null ? "n/a" : effectiveInitializationFloor)
                                        + " ts=" + timestampMs
                        );
                        logInfoSafely(
                                "SensorFusion",
                                "GNSS:bootstrap_seed_floor_untrusted"
                                        + " floorSeed="
                                        + (effectiveInitializationFloor == null ? "n/a" : effectiveInitializationFloor)
                                        + " ts=" + timestampMs
                        );
                    }
                } else {
                    logInfoSafely(
                            "SensorFusion",
                            "ABSOLUTE:init_deferred_invalid_fix"
                                    + " source=" + debugSource
                                    + " floorSeed="
                                    + (effectiveInitializationFloor == null ? "n/a" : effectiveInitializationFloor)
                                    + " acceptedFloor="
                                    + (acceptedFloorPrior == null ? "n/a" : acceptedFloorPrior)
                                    + " obsAgeMs=" + observationAgeMs
                                    + " ts=" + timestampMs
                    );
                    if ("GNSS".equals(debugSource)) {
                        logInfoSafely(
                                "SensorFusion",
                                "GNSS:bootstrap_rejected_invalid_observation"
                                        + " floorSeed="
                                        + (effectiveInitializationFloor == null ? "n/a" : effectiveInitializationFloor)
                                        + " obsAgeMs=" + observationAgeMs
                                        + " ts=" + timestampMs
                        );
                    }
                    recordAbsoluteFixDecision(debugSource, timestampMs, false, "init_deferred_invalid_fix");
                    return;
                }
            } else {
                recordAbsoluteFixDecision(debugSource, timestampMs, false, "deferred_constraints_not_ready");
                return;
            }
        }
        if (!degradedBootstrapMapNotReady) {
            pendingConstrainedAbsoluteFix = null;
        }
        ParticleFilterEngine.StateSnapshot previousParticleSnapshot = this.particleFilterEngine == null
                ? null
                : this.particleFilterEngine.captureStateSnapshot();
        boolean previousParticleCloudTrustedUnderConstraints = this.particleCloudTrustedUnderConstraints;
        boolean previousPfInitialized = this.pfInitialized;
        FusedPose previousPublishedPose = this.latestFusedPose;
        if (degradedBootstrapMapNotReady) {
            particleCloudTrustedUnderConstraints = false;
        }
        boolean bootstrapInitializedParticleCloud = false;
        FusedPose candidateFusedPose;
        if ((forceWifiFloorBootstrap || shouldReconcileProvisionalFloor) && acceptedFloorPrior != null) {
            // Modified: GNSS may initialize PF on floor 0 before WiFi returns the real floor.
            // Re-anchor the cloud to the first trusted WiFi floor instead of treating it as an
            // impossible cross-floor jump that must pass stairs/lift gating.
            logInfoSafely(
                    "SensorFusion",
                    (shouldReconcileProvisionalFloor
                            ? "Applying provisional floor reconcile floor="
                            : "Forcing initial WiFi floor bootstrap floor=")
                            + acceptedFloorPrior
                            + " previousFloor="
                            + (latestFusedPose == null ? "n/a" : latestFusedPose.getFloor())
            );
            this.particleFilterEngine.initialize(
                    localFix[0],
                    localFix[1],
                    acceptedFloorPrior,
                    timestampMs,
                    getCurrentHeadingRad(),
                    Math.max(MIN_ABSOLUTE_FIX_START_STD_M, accuracyMeters)
            );
            bootstrapInitializedParticleCloud = true;
            if (!previousPfInitialized) {
                noteBootstrapFloorSeed(acceptedFloorPrior, isFloorOffsetInitialized, debugSource, timestampMs);
            }
            candidateFusedPose = this.particleFilterEngine.estimatePose();
        } else {
            if (!previousPfInitialized && effectiveInitializationFloor != null) {
                noteBootstrapFloorSeed(
                        effectiveInitializationFloor,
                        isFloorOffsetInitialized && acceptedFloorPrior != null,
                        acceptedFloorPrior != null ? debugSource : "fallback_seed",
                        timestampMs
                );
            }
            candidateFusedPose = applyAbsoluteFixForStep1(
                    this.particleFilterEngine,
                    converter,
                    this.pfInitialized,
                    latitudeDeg,
                    longitudeDeg,
                    effectiveInitializationFloor,
                    acceptedFloorPrior,
                    timestampMs,
                    accuracyMeters,
                    getCurrentHeadingRad()
            );
        }
        if (particleFilterEngine != null) {
            logMotionDiagnostic(
                    "absolute_fix pf_result source=" + debugSource
                            + " reanchored=" + particleFilterEngine.wasLastAbsoluteFixReanchored()
                            + " rejectedByConstraints=" + particleFilterEngine.wasLastAbsoluteFixRejectedByConstraints()
                            + " candidate=" + formatFusedPose(candidateFusedPose)
            );
        }
        this.latestFusedPose = resolveConstrainedFusedPose(
                this.latestFusedPose,
                candidateFusedPose,
                debugSource,
                null
        );
        boolean absoluteFixReanchored = particleFilterEngine != null
                && particleFilterEngine.wasLastAbsoluteFixReanchored();
        boolean absoluteFixRejectedByParticleConstraints = particleFilterEngine != null
                && particleFilterEngine.wasLastAbsoluteFixRejectedByConstraints();
        boolean poseAdvanced = this.latestFusedPose != null
                && this.latestFusedPose.getTimestampMs() == timestampMs;
        boolean shouldRollbackUnpublishedParticleChange =
                ((absoluteFixReanchored && !absoluteFixRejectedByParticleConstraints)
                        || bootstrapInitializedParticleCloud)
                && !poseAdvanced;
        if (shouldRollbackUnpublishedParticleChange) {
            restoreParticleCloudSnapshot(previousParticleSnapshot, timestampMs);
            particleCloudTrustedUnderConstraints = previousParticleCloudTrustedUnderConstraints;
            this.pfInitialized = previousPfInitialized
                    && this.particleFilterEngine != null
                    && this.particleFilterEngine.hasParticles();
            logMotionDiagnostic(
                    "absolute_fix rollback_particle_change_kept_previous_pose=true"
                            + " source=" + debugSource
                            + " ts=" + timestampMs
                            + " bootstrapInit=" + bootstrapInitializedParticleCloud
                            + " reanchored=" + absoluteFixReanchored
                            + " previousFused=" + formatFusedPose(this.latestFusedPose)
                            + " candidate=" + formatFusedPose(candidateFusedPose)
            );
        } else {
            if (particleFilterEngine != null
                    && particleFilterEngine.hasParticles()
                    && !absoluteFixRejectedByParticleConstraints
                    && !degradedBootstrapMapNotReady) {
                particleCloudTrustedUnderConstraints = true;
            }
            this.pfInitialized = this.latestFusedPose != null;
        }
        this.lastPredictHeadingRad = getCurrentHeadingRad();
        this.lastPredictElevation = this.elevation;
        if (poseAdvanced) {
            String acceptedReason = "fused";
            if (!previousPfInitialized
                    && previousPublishedPose == null
                    && ("WIFI".equals(debugSource) || "GNSS".equals(debugSource))) {
                acceptedReason = degradedBootstrapMapNotReady
                        ? "init_degraded_bootstrap_map_not_ready"
                        : "init_full_constraints_ready";
                logInfoSafely(
                        "SensorFusion",
                        "ABSOLUTE:" + acceptedReason
                                + " source=" + debugSource
                                + " floor="
                                + (latestFusedPose == null ? "n/a" : latestFusedPose.getFloor())
                                + " ts=" + timestampMs
                );
                if ("GNSS".equals(debugSource)
                        && acceptedFloorPrior == null
                        && effectiveInitializationFloor != null) {
                    logInfoSafely(
                            "SensorFusion",
                            "GNSS:bootstrap_unknown_floor_allowed"
                                    + " floorSeed=" + effectiveInitializationFloor
                                    + " ts=" + timestampMs
                    );
                    logInfoSafely(
                            "SensorFusion",
                            "GNSS:bootstrap_seed_floor_untrusted"
                                    + " floorSeed=" + effectiveInitializationFloor
                                    + " ts=" + timestampMs
                    );
                }
            }
            recordAbsoluteFixDecision(debugSource, timestampMs, true, acceptedReason);
            recordPoseAdvance(debugSource, timestampMs);
            if (isPostLiftDestinationFixWindowActive(timestampMs)
                    && postLiftExpectedAbsoluteFloor == getAbsoluteCurrentFloor()) {
                lastLiftTransferState = "cleared_completed";
                clearPostLiftDestinationFixWindow("cleared_after_accept");
            }
        } else if (absoluteFixRejectedByParticleConstraints) {
            recordAbsoluteFixDecision(debugSource, timestampMs, false, "rejected_particle_constraints");
        } else {
            recordAbsoluteFixDecision(debugSource, timestampMs, false, "kept_previous_pose");
        }
        if (poseAdvanced
                && shouldReconcileProvisionalFloor
                && acceptedFloorPrior != null
                && latestFusedPose != null
                && latestFusedPose.getFloor() == acceptedFloorPrior) {
            noteProvisionalFloorReconcileApplied(acceptedFloorPrior, debugSource, timestampMs);
        }
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
        allowProvisionalFloorReconcileForCurrentFix = false;
        provisionalFloorReconcileTargetFloor = Integer.MIN_VALUE;
    }

    @Nullable
    private Integer resolveAbsoluteFixConstraintFloor(
            @Nullable Integer initializationFloor,
            @Nullable Integer floorPrior
    ) {
        Integer constraintFloor = floorPrior != null ? floorPrior : initializationFloor;
        return constraintFloor == null ? null : clampAbsoluteFloorToVenue(constraintFloor);
    }

    private boolean areParticleConstraintsReadyForFloor(@Nullable Integer floor) {
        return MapConstraintReadiness.hasFloorLevelParticleWallReadiness(
                coordinateConverter != null,
                floor
        );
    }

    private boolean shouldDeferConstrainedAbsoluteFix(
            @Nullable Integer initializationFloor,
            @Nullable Integer floorPrior
    ) {
        return !areParticleConstraintsReadyForFloor(
                resolveAbsoluteFixConstraintFloor(initializationFloor, floorPrior)
        );
    }

    private boolean canBootstrapParticleFilterWithoutReadyConstraints(
            @Nullable double[] localFix,
            double latitudeDeg,
            double longitudeDeg,
            @Nullable Integer initializationFloor,
            long observationAgeMs,
            @NonNull String debugSource
    ) {
        if (pfInitialized
                || initializationFloor == null
                || (!"WIFI".equals(debugSource) && !"GNSS".equals(debugSource))) {
            return false;
        }
        if (localFix == null
                || localFix.length < 2
                || !Double.isFinite(localFix[0])
                || !Double.isFinite(localFix[1])
                || !isValidLatLng(latitudeDeg, longitudeDeg)
                || observationAgeMs < 0L
                || observationAgeMs > BOOTSTRAP_ABSOLUTE_FIX_MAX_AGE_MS) {
            return false;
        }
        LatLng absoluteFixLatLng = new LatLng(latitudeDeg, longitudeDeg);
        if (MapConstraintRepository.hasVenueOutline()
                && !MapConstraintRepository.isPointInsideVenueOutline(absoluteFixLatLng)) {
            return false;
        }
        return isValidParticlePrediction(localFix[0], localFix[1], initializationFloor);
    }

    private void cachePendingConstrainedAbsoluteFix(
            double latitudeDeg,
            double longitudeDeg,
            @Nullable Integer initializationFloor,
            @Nullable Integer floorPrior,
            long timestampMs,
            float accuracyMeters,
            @NonNull String debugSource,
            long observationAgeMs
    ) {
        pendingConstrainedAbsoluteFix = new PendingConstrainedAbsoluteFix(
                new AbsoluteFix(timestampMs, latitudeDeg, longitudeDeg, accuracyMeters),
                initializationFloor,
                floorPrior,
                debugSource,
                observationAgeMs
        );
    }

    private int revalidateParticleCloudAgainstReadyConstraints() {
        if (particleFilterEngine == null || coordinateConverter == null) {
            return 0;
        }
        if (!MapConstraintRepository.hasAnyConstraints() && !MapConstraintRepository.hasVenueOutline()) {
            return 0;
        }
        int rejectedParticles = particleFilterEngine.revalidateParticlesAgainstConstraints();
        logMotionDiagnostic(
                "map_constraints_revalidated"
                        + " rejectedParticles=" + rejectedParticles
                        + " remainingParticles="
                        + particleFilterEngine.snapshotParticlesForTesting().size()
                        + " trustedCloud=" + particleCloudTrustedUnderConstraints
                        + " publishedPose=false"
        );
        return rejectedParticles;
    }

    private boolean maybeReplayPendingConstrainedAbsoluteFix() {
        PendingConstrainedAbsoluteFix pendingFix = pendingConstrainedAbsoluteFix;
        if (pendingFix == null) {
            return false;
        }
        if (!areParticleConstraintsReadyForFloor(resolveAbsoluteFixConstraintFloor(
                pendingFix.initializationFloor,
                pendingFix.floorPrior
        ))) {
            return false;
        }
        logMotionDiagnostic(
                "map_constraints_ready replay_pending_fix=true"
                        + " fixTs=" + pendingFix.absoluteFix.getTimestampMs()
                        + " floor=" + resolveAbsoluteFixConstraintFloor(
                        pendingFix.initializationFloor,
                        pendingFix.floorPrior
                )
        );
        if (particleFilterEngine != null) {
            particleFilterEngine.clearParticles(pendingFix.absoluteFix.getTimestampMs());
        }
        particleCloudTrustedUnderConstraints = false;
        latestFusedPose = null;
        pfInitialized = false;
        pendingConstrainedAbsoluteFix = null;
        handleAbsoluteFixInternal(
                pendingFix.absoluteFix.getLatitudeDeg(),
                pendingFix.absoluteFix.getLongitudeDeg(),
                pendingFix.initializationFloor,
                pendingFix.floorPrior,
                pendingFix.absoluteFix.getTimestampMs(),
                pendingFix.absoluteFix.getAccuracyMeters(),
                pendingFix.debugSource,
                pendingFix.observationAgeMs
        );
        return true;
    }

    public synchronized void onMapMatchingConstraintsUpdated() {
        if (maybeReplayPendingConstrainedAbsoluteFix()) {
            return;
        }

        int rejectedParticles = revalidateParticleCloudAgainstReadyConstraints();
        if (!particleCloudTrustedUnderConstraints) {
            long clearTimestampMs = latestFusedPose != null
                    ? latestFusedPose.getTimestampMs()
                    : System.currentTimeMillis();
            if (particleFilterEngine != null && particleFilterEngine.hasParticles()) {
                particleFilterEngine.clearParticles(clearTimestampMs);
            }
            pfInitialized = false;
            logMotionDiagnostic(
                    "map_constraints_ready hold_untrusted_cloud=true"
                            + " rejectedParticles=" + rejectedParticles
                            + " keptFused=" + formatFusedPose(latestFusedPose)
            );
            return;
        }

        if (rejectedParticles <= 0) {
            return;
        }

        latestFusedPose = particleFilterEngine == null ? null : particleFilterEngine.estimatePose();
        pfInitialized = latestFusedPose != null;
        if (latestFusedPose != null) {
            recordLatestFusedPoseIfNeeded();
        }
        logMotionDiagnostic(
                "map_constraints_ready published_revalidated_pose=true"
                        + " rejectedParticles=" + rejectedParticles
                        + " fused=" + formatFusedPose(latestFusedPose)
        );
    }

    private void handleAbsoluteFixInternal(@NonNull AbsoluteFix absoluteFix,
                                           @Nullable Integer initializationFloor,
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
        logMotionDiagnostic(
                "handlePdrPredict called"
                        + " ts=" + timestampMs
                        + " floor=" + floor
                        + " obsAgeMs=" + observationAgeMs
                        + " pfInitialized=" + pfInitialized
                        + " stationary=" + isStationary
                        + " stepLenM=" + formatDebugDouble(pdrDelta == null ? Double.NaN : pdrDelta.getStepLengthMeters())
                        + " deltaHeadingRad=" + formatDebugDouble(pdrDelta == null ? Double.NaN : pdrDelta.getDeltaHeadingRad())
                        + " heightDeltaM=" + formatDebugDouble(pdrDelta == null ? Double.NaN : pdrDelta.getHeightDeltaMeters())
        );
        if (!saveRecording) {
            recordPoseBlocked("PDR:ignored_not_recording");
            return;
        }
        maybeReplayPendingConstrainedAbsoluteFix();

        // Formal fusion flow must wait for the first real absolute fix.
        // PDR-only prediction is ignored until GNSS/WiFi/other absolute positioning arrives.
        if (!pfInitialized || this.particleFilterEngine == null || pdrDelta == null) {
            String reason;
            if (!pfInitialized) {
                reason = "PDR:waiting_first_absolute_fix";
            } else if (this.particleFilterEngine == null) {
                reason = "PDR:missing_particle_filter";
            } else {
                reason = "PDR:missing_delta";
            }
            recordPoseBlocked(reason);
            logMotionDiagnostic("handlePdrPredict skipped reason=" + reason + " ts=" + timestampMs);
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
        String invalidDeltaReason = getInvalidPdrDeltaReason(constrainedDelta);
        if (invalidDeltaReason != null) {
            String blockReason = "PDR:" + invalidDeltaReason;
            recordPoseBlocked(blockReason);
            logMotionDiagnostic(
                    "handlePdrPredict skipped reason=" + blockReason
                            + " ts=" + timestampMs
                            + " stepLenM=" + formatDebugDouble(constrainedDelta.getStepLengthMeters())
                            + " deltaHeadingRad=" + formatDebugDouble(constrainedDelta.getDeltaHeadingRad())
                            + " heightDeltaM=" + formatDebugDouble(constrainedDelta.getHeightDeltaMeters())
            );
            return;
        }
        FusedPose candidateFusedPose = applyPdrPredictionForStep1(
                this.particleFilterEngine,
                this.pfInitialized,
                constrainedDelta,
                floor,
                timestampMs
        );
        if (particleFilterEngine != null) {
            logMotionDiagnostic(
                    "pdr_predict pf_result"
                            + " ts=" + timestampMs
                            + " wallReject=" + particleFilterEngine.getLastPredictWallRejectCount()
                            + " floorReject=" + particleFilterEngine.getLastPredictFloorConstraintRejectCount()
                            + " candidate=" + formatFusedPose(candidateFusedPose)
            );
        }
        FusedPose previousFusedPose = this.latestFusedPose;
        this.latestFusedPose = resolveConstrainedFusedPose(
                previousFusedPose,
                candidateFusedPose,
                "PDR",
                constrainedDelta
        );
        if (this.latestFusedPose != null && this.latestFusedPose.getTimestampMs() == timestampMs) {
            recordPoseAdvance("PDR", timestampMs);
        } else {
            if (!hasSpecificBlockReason("PDR")) {
                recordPoseBlocked("PDR:kept_previous_pose");
            }
            logMotionDiagnostic(
                    "pdr_kept_previous_pose"
                            + " reason=" + lastBlockReason
                            + " previous=" + formatFusedPose(previousFusedPose)
                            + " candidate=" + formatFusedPose(candidateFusedPose)
                            + " stepLenM=" + formatDebugDouble(constrainedDelta.getStepLengthMeters())
                            + " deltaHeadingRad=" + formatDebugDouble(constrainedDelta.getDeltaHeadingRad())
                            + " floor=" + floor
            );
        }
        recordLatestFusedPoseIfNeeded();
        logPdrFusionTrace(constrainedDelta, floor, timestampMs, observationAgeMs);
    }

    public FusedPose getLatestFusedPose() {
        return latestFusedPose;
    }

    @Nullable
    private FusedPose resolveConstrainedFusedPose(
            @Nullable FusedPose previousPose,
            @Nullable FusedPose candidatePose,
            @NonNull String debugSource,
            @Nullable PdrDelta pdrDelta
    ) {
        PoseConstraintReport constraintReport = analyzePoseConstraint(previousPose, candidatePose, debugSource);
        if (allowProvisionalFloorReconcileForCurrentFix
                && constraintReport.floorTransitionRejectedByMapGate
                && candidatePose != null
                && candidatePose.getFloor() == provisionalFloorReconcileTargetFloor
                && constraintReport.candidatePointLegal) {
            logInfoSafely(
                    "SensorFusion",
                    "FLOOR:provisional_reconcile_applied floor=" + candidatePose.getFloor()
                            + " source=" + debugSource
                            + " strategy=bypass_map_gate_for_provisional_bootstrap"
            );
            return candidatePose;
        }
        if (constraintReport.isAccepted()) {
            return candidatePose;
        }
        FusedPose recoveredPose = maybeRecoverConstrainedPose(
                previousPose,
                candidatePose,
                debugSource,
                pdrDelta,
                constraintReport
        );
        if (recoveredPose != null) {
            logInfoSafely(
                    "SensorFusion",
                    "POSE:clamp_to_last_legal_point source=" + debugSource
                            + " distanceM="
                            + formatDebugDouble(Math.hypot(
                            recoveredPose.getX() - previousPose.getX(),
                            recoveredPose.getY() - previousPose.getY()
                    ))
            );
            return recoveredPose;
        }
        recordPoseBlocked(debugSource + ":" + constraintReport.reasonCode);
        logInfoSafely(
                "SensorFusion",
                "POSE:hold_previous reason=" + constraintReport.reasonCode
                        + " source=" + debugSource
        );
        logPoseConstraintRejection(
                debugSource,
                previousPose,
                candidatePose,
                null,
                pdrDelta,
                constraintReport,
                "kept_previous_pose"
        );
        if (constraintReport.candidatePointLegal) {
            logWarnSafely("SensorFusion", "Rejecting " + debugSource + " fused transition that would cross map constraints.");
        } else {
            logWarnSafely("SensorFusion", "Rejecting illegal " + debugSource + " fused pose; keeping last legal pose.");
        }
        return previousPose;
    }

    private void restoreParticleCloudSnapshot(
            @Nullable ParticleFilterEngine.StateSnapshot particleSnapshot,
            long timestampMs
    ) {
        if (this.particleFilterEngine == null) {
            return;
        }
        if (particleSnapshot == null) {
            this.particleFilterEngine.clearParticles(timestampMs);
            return;
        }
        this.particleFilterEngine.restoreStateSnapshot(particleSnapshot);
    }

    private boolean hasRenderableMapConstraints() {
        return MapConstraintRepository.hasAnyConstraints() || MapConstraintRepository.hasVenueOutline();
    }

    private boolean isFusedPoseLegal(@Nullable FusedPose fusedPose) {
        if (fusedPose == null) {
            return false;
        }
        LatLng fusedLatLng = getLatLngForFusedPose(fusedPose);
        if (fusedLatLng == null) {
            return true;
        }
        return MapConstraintRepository.isPointLegal(fusedLatLng, fusedPose.getFloor());
    }

    private boolean isFusedTransitionLegal(@NonNull FusedPose previousPose, @NonNull FusedPose candidatePose) {
        LatLng previousLatLng = getLatLngForFusedPose(previousPose);
        LatLng candidateLatLng = getLatLngForFusedPose(candidatePose);
        if (previousLatLng == null || candidateLatLng == null) {
            return true;
        }
        if (previousPose.getFloor() == candidatePose.getFloor()) {
            return MapConstraintRepository.isPathLegal(
                    previousLatLng,
                    candidateLatLng,
                    candidatePose.getFloor()
            );
        }
        return allowsMapBasedFloorTransition(
                previousPose.getX(),
                previousPose.getY(),
                candidatePose.getX(),
                candidatePose.getY(),
                previousPose.getFloor(),
                candidatePose.getFloor()
        );
    }

    @NonNull
    private PoseConstraintReport analyzePoseConstraint(
            @Nullable FusedPose previousPose,
            @Nullable FusedPose candidatePose,
            @NonNull String debugSource
    ) {
        if (candidatePose == null) {
            return new PoseConstraintReport(
                    "PDR".equals(debugSource) ? "pf_result_empty_or_unstable" : "candidate_pose_null",
                    false,
                    false,
                    false,
                    false,
                    false,
                    false,
                    false,
                    false,
                    Integer.MIN_VALUE,
                    previousPose == null ? null : getLatLngForFusedPose(previousPose),
                    null
            );
        }
        LatLng previousLatLng = previousPose == null ? null : getLatLngForFusedPose(previousPose);
        LatLng candidateLatLng = getLatLngForFusedPose(candidatePose);
        if (!hasRenderableMapConstraints()) {
            return new PoseConstraintReport(
                    "accepted",
                    true,
                    true,
                    false,
                    false,
                    false,
                    false,
                    false,
                    false,
                    candidatePose.getFloor(),
                    previousLatLng,
                    candidateLatLng
            );
        }

        boolean pointInsideWall = candidateLatLng != null
                && MapConstraintRepository.hasWallConstraints(candidatePose.getFloor())
                && MapConstraintRepository.isPointInsideWall(candidateLatLng, candidatePose.getFloor());
        boolean pointOutsideVenueOutline = candidateLatLng != null
                && MapConstraintRepository.hasVenueOutline()
                && !MapConstraintRepository.isPointInsideVenueOutline(candidateLatLng);
        boolean candidatePointLegal = candidateLatLng == null || (!pointInsideWall && !pointOutsideVenueOutline);
        if (!candidatePointLegal) {
            return new PoseConstraintReport(
                    pointInsideWall
                            ? "illegal_pose_inside_wall"
                            : "illegal_pose_outside_venue_outline",
                    false,
                    false,
                    pointInsideWall,
                    pointOutsideVenueOutline,
                    false,
                    false,
                    false,
                    false,
                    candidatePose.getFloor(),
                    previousLatLng,
                    candidateLatLng
            );
        }
        if (previousPose == null || previousLatLng == null || candidateLatLng == null) {
            return new PoseConstraintReport(
                    "accepted",
                    true,
                    true,
                    false,
                    false,
                    false,
                    false,
                    false,
                    false,
                    candidatePose.getFloor(),
                    previousLatLng,
                    candidateLatLng
            );
        }

        boolean pathExitsVenueOutline = MapConstraintRepository.hasVenueOutline()
                && MapConstraintRepository.doesPathExitVenueOutline(previousLatLng, candidateLatLng);
        boolean pathCrossesWallOnPreviousFloor = MapConstraintRepository.hasWallConstraints(previousPose.getFloor())
                && MapConstraintRepository.doesPathIntersectWall(
                previousLatLng,
                candidateLatLng,
                previousPose.getFloor()
        );
        boolean pathCrossesWallOnCandidateFloor = MapConstraintRepository.hasWallConstraints(candidatePose.getFloor())
                && MapConstraintRepository.doesPathIntersectWall(
                previousLatLng,
                candidateLatLng,
                candidatePose.getFloor()
        );
        boolean floorTransitionRejectedByMapGate = previousPose.getFloor() != candidatePose.getFloor()
                && !allowsMapBasedFloorTransition(
                previousPose.getX(),
                previousPose.getY(),
                candidatePose.getX(),
                candidatePose.getY(),
                previousPose.getFloor(),
                candidatePose.getFloor()
        );

        if (previousPose.getFloor() == candidatePose.getFloor()) {
            if (pathExitsVenueOutline) {
                return new PoseConstraintReport(
                        "illegal_transition_exits_venue_outline",
                        true,
                        false,
                        false,
                        false,
                        pathCrossesWallOnPreviousFloor,
                        pathCrossesWallOnCandidateFloor,
                        true,
                        false,
                        candidatePose.getFloor(),
                        previousLatLng,
                        candidateLatLng
                );
            }
            if (pathCrossesWallOnCandidateFloor) {
                return new PoseConstraintReport(
                        "illegal_transition_crosses_wall",
                        true,
                        false,
                        false,
                        false,
                        pathCrossesWallOnPreviousFloor,
                        true,
                        false,
                        false,
                        candidatePose.getFloor(),
                        previousLatLng,
                        candidateLatLng
                );
            }
        } else if (floorTransitionRejectedByMapGate) {
            return new PoseConstraintReport(
                    "floor_transition_rejected_by_map_gate",
                    true,
                    false,
                    false,
                    false,
                    pathCrossesWallOnPreviousFloor,
                    pathCrossesWallOnCandidateFloor,
                    pathExitsVenueOutline,
                    true,
                    candidatePose.getFloor(),
                    previousLatLng,
                    candidateLatLng
            );
        }

        return new PoseConstraintReport(
                "accepted",
                true,
                true,
                false,
                false,
                pathCrossesWallOnPreviousFloor,
                pathCrossesWallOnCandidateFloor,
                pathExitsVenueOutline,
                false,
                candidatePose.getFloor(),
                previousLatLng,
                candidateLatLng
        );
    }

    @Nullable
    private FusedPose maybeRecoverConstrainedPose(
            @Nullable FusedPose previousPose,
            @Nullable FusedPose candidatePose,
            @NonNull String debugSource,
            @Nullable PdrDelta pdrDelta,
            @NonNull PoseConstraintReport constraintReport
    ) {
        if (!"PDR".equals(debugSource)
                || previousPose == null
                || candidatePose == null
                || coordinateConverter == null) {
            return null;
        }

        FusedPose particleFallback = findClosestLegalParticleFallback(previousPose, candidatePose);
        if (particleFallback != null) {
            logPoseConstraintRejection(
                    debugSource,
                    previousPose,
                    candidatePose,
                    particleFallback,
                    pdrDelta,
                    constraintReport,
                    "recovered_by_particle_fallback"
            );
            return particleFallback;
        }

        FusedPose clampedPose = clampCandidateToLegalSegmentPrefix(
                previousPose,
                candidatePose,
                coordinateConverter
        );
        if (clampedPose != null) {
            logPoseConstraintRejection(
                    debugSource,
                    previousPose,
                    candidatePose,
                    clampedPose,
                    pdrDelta,
                    constraintReport,
                    "recovered_by_segment_prefix_clamp"
            );
        }
        return clampedPose;
    }

    @Nullable
    private FusedPose findClosestLegalParticleFallback(
            @NonNull FusedPose previousPose,
            @NonNull FusedPose candidatePose
    ) {
        if (particleFilterEngine == null) {
            return null;
        }
        List<Particle> particles = particleFilterEngine.snapshotParticlesForTesting();
        if (particles.isEmpty()) {
            return null;
        }

        Particle bestParticle = null;
        double bestDistanceSq = Double.POSITIVE_INFINITY;
        double bestWeight = -1.0;
        double bestProgressMeters = -1.0;
        for (Particle particle : particles) {
            FusedPose particlePose = new FusedPose(
                    particle.getX(),
                    particle.getY(),
                    particle.getFloor(),
                    candidatePose.getConfidence(),
                    candidatePose.getTimestampMs()
            );
            PoseConstraintReport particleReport = analyzePoseConstraint(previousPose, particlePose, "PDR");
            if (!particleReport.isAccepted()) {
                continue;
            }
            double progressMeters = Math.hypot(
                    particlePose.getX() - previousPose.getX(),
                    particlePose.getY() - previousPose.getY()
            );
            if (progressMeters < MIN_CONSTRAINED_POSE_PROGRESS_M) {
                continue;
            }
            double distanceSqToCandidate = Math.pow(particlePose.getX() - candidatePose.getX(), 2)
                    + Math.pow(particlePose.getY() - candidatePose.getY(), 2)
                    + (particlePose.getFloor() == candidatePose.getFloor() ? 0.0 : 1000.0);
            double particleWeight = particle.getWeight();
            if (distanceSqToCandidate < bestDistanceSq
                    || (Math.abs(distanceSqToCandidate - bestDistanceSq) <= 1e-9
                    && particleWeight > bestWeight)
                    || (Math.abs(distanceSqToCandidate - bestDistanceSq) <= 1e-9
                    && Math.abs(particleWeight - bestWeight) <= 1e-9
                    && progressMeters > bestProgressMeters)) {
                bestParticle = particle;
                bestDistanceSq = distanceSqToCandidate;
                bestWeight = particleWeight;
                bestProgressMeters = progressMeters;
            }
        }
        if (bestParticle == null) {
            return null;
        }
        return new FusedPose(
                bestParticle.getX(),
                bestParticle.getY(),
                bestParticle.getFloor(),
                candidatePose.getConfidence(),
                candidatePose.getTimestampMs()
        );
    }

    private void logPoseConstraintRejection(
            @NonNull String debugSource,
            @Nullable FusedPose previousPose,
            @Nullable FusedPose candidatePose,
            @Nullable FusedPose resolvedPose,
            @Nullable PdrDelta pdrDelta,
            @NonNull PoseConstraintReport constraintReport,
            @NonNull String outcome
    ) {
        logMotionDiagnostic(
                "pose_constraint outcome=" + outcome
                        + " source=" + debugSource
                        + " reason=" + constraintReport.reasonCode
                        + " previous=" + formatFusedPose(previousPose)
                        + " candidate=" + formatFusedPose(candidatePose)
                        + " resolved=" + formatFusedPose(resolvedPose)
                        + " previousLatLng=" + formatLatLng(constraintReport.previousLatLng)
                        + " candidateLatLng=" + formatLatLng(constraintReport.candidateLatLng)
                        + " stepLenM=" + formatDebugDouble(pdrDelta == null ? Double.NaN : pdrDelta.getStepLengthMeters())
                        + " deltaHeadingRad=" + formatDebugDouble(pdrDelta == null ? Double.NaN : pdrDelta.getDeltaHeadingRad())
                        + " floorForLegality=" + constraintReport.floorForLegalityCheck
                        + " pointLegal=" + constraintReport.candidatePointLegal
                        + " transitionLegal=" + constraintReport.transitionLegal
                        + " insideWall=" + constraintReport.pointInsideWall
                        + " outsideVenue=" + constraintReport.pointOutsideVenueOutline
                        + " crossWallPrev=" + constraintReport.pathCrossesWallOnPreviousFloor
                        + " crossWallCandidate=" + constraintReport.pathCrossesWallOnCandidateFloor
                        + " exitVenue=" + constraintReport.pathExitsVenueOutline
                        + " floorTransitionRejected=" + constraintReport.floorTransitionRejectedByMapGate
                        + " pfWallReject=" + (particleFilterEngine == null
                        ? -1
                        : particleFilterEngine.getLastPredictWallRejectCount())
                        + " pfFloorReject=" + (particleFilterEngine == null
                        ? -1
                        : particleFilterEngine.getLastPredictFloorConstraintRejectCount())
        );
    }

    @Nullable
    private String getInvalidPdrDeltaReason(@Nullable PdrDelta pdrDelta) {
        if (pdrDelta == null) {
            return "missing_delta";
        }
        if (!Float.isFinite(pdrDelta.getStepLengthMeters()) || pdrDelta.getStepLengthMeters() < 0f) {
            return "invalid_step_displacement";
        }
        if (!Float.isFinite(pdrDelta.getDeltaHeadingRad())) {
            return "invalid_heading_projection";
        }
        if (!Float.isFinite(pdrDelta.getHeightDeltaMeters())) {
            return "invalid_height_delta";
        }
        return null;
    }

    private boolean hasSpecificBlockReason(@NonNull String debugSource) {
        return lastBlockReason.startsWith(debugSource + ":")
                && !(debugSource + ":kept_previous_pose").equals(lastBlockReason);
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
    public LatLng getRecordedTrajectoryOriginLatLng() {
        if (trajectory != null && trajectory.hasInitialPosition()) {
            Traj.GNSSPosition initialPosition = trajectory.getInitialPosition();
            return new LatLng(initialPosition.getLatitude(), initialPosition.getLongitude());
        }
        if (coordinateConverter != null) {
            return new LatLng(
                    coordinateConverter.getOriginLatitudeDeg(),
                    coordinateConverter.getOriginLongitudeDeg()
            );
        }
        if (startLocation != null
                && startLocation.length >= 2
                && isValidLatLng(startLocation[0], startLocation[1])) {
            return new LatLng(startLocation[0], startLocation[1]);
        }
        if (isValidLatLng(latitude, longitude)) {
            return new LatLng(latitude, longitude);
        }
        return null;
    }

    @NonNull
    public List<LatLng> getRecordedTrajectoryLatLngs() {
        return getRecordedTrajectoryLatLngs(1f);
    }

    @NonNull
    public List<LatLng> getRecordedTrajectoryLatLngs(float scaleFactor) {
        if (trajectory == null) {
            return new ArrayList<>();
        }

        LatLng origin = getRecordedTrajectoryOriginLatLng();
        CoordinateConverter trajectoryConverter = getRecordedTrajectoryCoordinateConverter(origin);
        if (trajectoryConverter == null) {
            return new ArrayList<>();
        }

        float effectiveScaleFactor = Float.isFinite(scaleFactor) && scaleFactor > 0f
                ? scaleFactor
                : 1f;
        boolean useScaledPdrPath = Math.abs(effectiveScaleFactor - 1f) > 1e-3f;
        int estimatedPointCount = useScaledPdrPath
                ? trajectory.getPdrDataCount() + (origin != null ? 1 : 0)
                : trajectory.getFusedPoseCount();
        List<LatLng> recordedPoints = new ArrayList<>(Math.max(estimatedPointCount, 0));

        if (!useScaledPdrPath) {
            appendRecordedFusedTrajectory(recordedPoints, trajectoryConverter);
            if (!recordedPoints.isEmpty()) {
                return recordedPoints;
            }
        }

        if (origin != null) {
            recordedPoints.add(origin);
        }
        appendRecordedPdrTrajectory(recordedPoints, trajectoryConverter, effectiveScaleFactor);
        return recordedPoints;
    }

    @Nullable
    private CoordinateConverter getRecordedTrajectoryCoordinateConverter(@Nullable LatLng origin) {
        if (origin != null) {
            return new CoordinateConverter(origin.latitude, origin.longitude);
        }
        return coordinateConverter;
    }

    private void appendRecordedFusedTrajectory(
            @NonNull List<LatLng> recordedPoints,
            @NonNull CoordinateConverter trajectoryConverter
    ) {
        for (Traj.FusedPoseRecord fusedPoseRecord : trajectory.getFusedPoseList()) {
            recordedPoints.add(
                    trajectoryConverter.toLatLng(fusedPoseRecord.getX(), fusedPoseRecord.getY())
            );
        }
    }

    private void appendRecordedPdrTrajectory(
            @NonNull List<LatLng> recordedPoints,
            @NonNull CoordinateConverter trajectoryConverter,
            float scaleFactor
    ) {
        for (Traj.RelativePosition relativePosition : trajectory.getPdrDataList()) {
            recordedPoints.add(
                    trajectoryConverter.toLatLng(
                            relativePosition.getX() * scaleFactor,
                            relativePosition.getY() * scaleFactor
                    )
            );
        }
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
            long timestampMs,
            boolean displayPoseApplied,
            int displayFloor
    ) {
        lastDisplayedFusedMarkerRawLatLng = rawLocation;
        lastDisplayedFusedMarkerLatLng = displayedLocation;
        lastDisplayedFusedMarkerTimestampMs = timestampMs;
        if (!displayPoseApplied) {
            return;
        }
        committedDisplayFloorAbsolute = clampAbsoluteFloorToVenue(displayFloor);
        if (pendingCommittedDisplayFloorAbsolute == committedDisplayFloorAbsolute) {
            floorSwitchPending = false;
            pendingCommittedDisplayFloorAbsolute = Integer.MIN_VALUE;
            lastFloorSwitchBlockReason = "none";
        }
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
        return cumulativeFloorTracker.getRelativeFloor();
    }

    private float getActiveFloorHeightMeters() {
        if (Float.isFinite(floorHeightOverrideForTesting) && floorHeightOverrideForTesting > 0f) {
            return floorHeightOverrideForTesting;
        }
        return this.pdrProcessing == null ? 0f : this.pdrProcessing.getFloorHeightMeters();
    }

    void setFloorHeightOverrideForTesting(float floorHeightMeters) {
        floorHeightOverrideForTesting = floorHeightMeters > 0f ? floorHeightMeters : Float.NaN;
    }

    private void updateCumulativeBarometerFloorEstimate(long timestampMs) {
        if (this.pdrProcessing == null) {
            return;
        }
        if (elevatorFloorSessionActive) {
            return;
        }
        int previousRelativeFloor = cumulativeFloorTracker.getRelativeFloor();
        cumulativeFloorTracker.update(
                this.elevation,
                getActiveFloorHeightMeters(),
                timestampMs
        );
        boolean clampedToVenueBounds = clampCumulativeFloorTrackerToVenueBounds();
        int updatedRelativeFloor = cumulativeFloorTracker.getRelativeFloor();
        if (clampedToVenueBounds) {
            Log.w(
                    "SensorFusion",
                    "Clamped cumulative floor tracker to venue bounds absFloor="
                            + getAbsoluteCurrentFloor()
                            + " relFloor="
                            + updatedRelativeFloor
            );
        }
        if (updatedRelativeFloor != previousRelativeFloor) {
            int previousAbsoluteFloor = clampAbsoluteFloorToVenue(previousRelativeFloor + pdrFloorOffset);
            int updatedAbsoluteFloor = clampAbsoluteFloorToVenue(updatedRelativeFloor + pdrFloorOffset);
            Log.i(
                    "SensorFusion",
                    "Barometer committed floor transition relFloor="
                            + updatedRelativeFloor
                            + " absFloor="
                            + getAbsoluteCurrentFloor()
                            + " elevation="
                            + this.elevation
                            + " floorHeight="
                            + getActiveFloorHeightMeters()
                            + " requiredTravel="
                            + CumulativeFloorTracker.requiredTravelForFloorChange(
                                    getActiveFloorHeightMeters()
                            )
            );
            syncFusedFloorFromBarometer(previousAbsoluteFloor, updatedAbsoluteFloor, timestampMs);
        }
    }

    private void syncFusedFloorFromBarometer(
            int previousAbsoluteFloor,
            int updatedAbsoluteFloor,
            long timestampMs
    ) {
        if (previousAbsoluteFloor == updatedAbsoluteFloor || latestFusedPose == null) {
            return;
        }

        boolean strongEvidence = hasRecentStrongBarometerFloorTransitionEvidence(
                previousAbsoluteFloor,
                updatedAbsoluteFloor
        );
        boolean failOpenTransition = shouldFailOpenBarometerFloorTransition(
                previousAbsoluteFloor,
                updatedAbsoluteFloor
        );
        boolean transitionAllowed = failOpenTransition || allowsMapBasedFloorTransition(
                latestFusedPose.getX(),
                latestFusedPose.getY(),
                latestFusedPose.getX(),
                latestFusedPose.getY(),
                latestFusedPose.getFloor(),
                updatedAbsoluteFloor
        );
        if (!transitionAllowed) {
            logInfoSafely(
                    "SensorFusion",
                    "FLOOR:transition_rejected_insufficient_evidence"
                            + " prevFloor=" + previousAbsoluteFloor
                            + " newFloor=" + updatedAbsoluteFloor
                            + " strongEvidence=" + strongEvidence
                            + " failOpen=" + failOpenTransition
            );
            Log.w(
                    "SensorFusion",
                "Rejected barometer-only fused floor sync prevFloor="
                            + previousAbsoluteFloor
                            + " newFloor="
                            + updatedAbsoluteFloor
            );
            return;
        }
        logInfoSafely(
                "SensorFusion",
                (failOpenTransition
                        ? "FLOOR:transition_fail_open_limited"
                        : "FLOOR:transition_strict_map_gate_pass")
                        + " prevFloor=" + previousAbsoluteFloor
                        + " newFloor=" + updatedAbsoluteFloor
                        + " strongEvidence=" + strongEvidence
        );

        if (!applyAbsoluteFloorKeepingCurrentXy(updatedAbsoluteFloor, timestampMs)) {
            return;
        }
        activateFloorOnlyResyncWindow(updatedAbsoluteFloor, timestampMs);
        noteFloorConsensusDecision("promoted_by_barometer", timestampMs);
        recordLatestFusedPoseIfNeeded();
        Log.i(
                "SensorFusion",
                "Synced fused floor from barometer prevFloor="
                        + previousAbsoluteFloor
                        + " newFloor="
                        + updatedAbsoluteFloor
                        + " strongEvidence="
                        + strongEvidence
                        + " failOpen="
                        + failOpenTransition
        );
    }

    private boolean hasRecentStrongBarometerFloorTransitionEvidence(int previousFloor, int newFloor) {
        return cumulativeFloorTracker.hasRecentStrongTransitionEvidence(
                previousFloor,
                newFloor,
                System.currentTimeMillis()
        );
    }

    private boolean hasRecentBarometerFloorTransitionEvidence(int previousFloor, int newFloor) {
        return cumulativeFloorTracker.hasRecentTransition(
                previousFloor,
                newFloor,
                System.currentTimeMillis()
        );
    }

    private boolean hasBarometerEvidenceForFloorTransitionFallback(int previousFloor, int newFloor) {
        if (hasRecentStrongBarometerFloorTransitionEvidence(previousFloor, newFloor)) {
            return true;
        }
        return newFloor < previousFloor
                && hasRecentBarometerFloorTransitionEvidence(previousFloor, newFloor);
    }

    @NonNull
    private TransitionConstraintAvailability resolveTransitionConstraintAvailability(
            int previousFloor,
            int newFloor
    ) {
        boolean hasMapConstraints = MapConstraintRepository.hasAnyConstraints()
                || MapConstraintRepository.hasVenueOutline();
        if (!hasMapConstraints) {
            return TransitionConstraintAvailability.MAP_UNAVAILABLE;
        }
        if (!MapConstraintRepository.hasUsableFloorTransitionConstraints(previousFloor, newFloor)) {
            if (!MapConstraintRepository.hasKnownFloor(previousFloor)
                    || !MapConstraintRepository.hasKnownFloor(newFloor)) {
                return TransitionConstraintAvailability.FLOOR_DATA_UNAVAILABLE;
            }
            return TransitionConstraintAvailability.TRANSITION_GEOMETRY_UNAVAILABLE;
        }
        return TransitionConstraintAvailability.COMPLETE;
    }

    private boolean shouldFailOpenBarometerFloorTransition(int previousFloor, int newFloor) {
        if (Math.abs(newFloor - previousFloor) != 1) {
            return false;
        }
        TransitionConstraintAvailability availability = resolveTransitionConstraintAvailability(
                previousFloor,
                newFloor
        );
        if (availability == TransitionConstraintAvailability.COMPLETE
                || availability == TransitionConstraintAvailability.MAP_UNAVAILABLE
                || availability == TransitionConstraintAvailability.FLOOR_DATA_UNAVAILABLE) {
            return false;
        }
        if (!hasBarometerEvidenceForFloorTransitionFallback(previousFloor, newFloor)) {
            return false;
        }
        long timestampMs = System.currentTimeMillis();
        LatLng contextLatLng = resolveLiftContextLatLng(timestampMs);
        if (contextLatLng == null) {
            return false;
        }
        return isLiftTransitionSatisfiedWithTolerance(
                contextLatLng,
                contextLatLng,
                previousFloor,
                newFloor,
                STRONG_BAROMETER_TRANSITION_TOLERANCE_M
        ) || isStairTransitionSatisfiedWithTolerance(
                contextLatLng,
                contextLatLng,
                previousFloor,
                newFloor,
                STRONG_BAROMETER_TRANSITION_TOLERANCE_M
        );
    }

    /**
     * Get the current absolute floor aligned to the map floor index space.
     */
    private int getAbsoluteCurrentFloor() {
        return clampAbsoluteFloorToVenue(getRelativeCurrentFloor() + pdrFloorOffset);
    }

    private int clampAbsoluteFloorToVenue(int absoluteFloor) {
        try {
            return IndoorMapManager.clampFloorToVenue(collectionVenue, absoluteFloor);
        } catch (ExceptionInInitializerError | NoClassDefFoundError ignored) {
            return absoluteFloor;
        }
    }

    private boolean clampCumulativeFloorTrackerToVenueBounds() {
        try {
            IndoorMapManager.FloorBounds floorBounds = IndoorMapManager.resolveFloorBounds(collectionVenue);
            if (floorBounds == null) {
                return false;
            }
            return cumulativeFloorTracker.clampRelativeFloor(
                    floorBounds.minFloor - pdrFloorOffset,
                    floorBounds.maxFloor - pdrFloorOffset
            );
        } catch (ExceptionInInitializerError | NoClassDefFoundError ignored) {
            return false;
        }
    }

    private boolean applyAbsoluteFloorKeepingCurrentXy(int absoluteFloor, long timestampMs) {
        if (!canPreserveCurrentXyOnFloor(absoluteFloor)) {
            try {
                Log.w(
                        "SensorFusion",
                        "Rejected floor-only sync because current XY is not legal on target floor floor="
                                + absoluteFloor
                                + " fused=" + formatFusedPose(latestFusedPose)
                );
            } catch (RuntimeException ignored) {
            }
            return false;
        }
        if (particleFilterEngine != null && pfInitialized && !particleFilterEngine.forceFloor(absoluteFloor, timestampMs)) {
            try {
                Log.w(
                        "SensorFusion",
                        "Rejected floor-only particle sync because particle cloud is not legal on target floor floor="
                                + absoluteFloor
                );
            } catch (RuntimeException ignored) {
            }
            return false;
        }
        if (latestFusedPose != null) {
            latestFusedPose = new FusedPose(
                    latestFusedPose.getX(),
                    latestFusedPose.getY(),
                    absoluteFloor,
                    latestFusedPose.getConfidence(),
                    timestampMs
            );
        }
        return true;
    }

    private boolean canPreserveCurrentXyOnFloor(int absoluteFloor) {
        if (!areParticleConstraintsReadyForFloor(absoluteFloor)) {
            return false;
        }
        if (latestFusedPose != null) {
            LatLng latestPoseLatLng = getLatLngForFusedPose(latestFusedPose);
            if (latestPoseLatLng == null || !MapConstraintRepository.isPointLegal(latestPoseLatLng, absoluteFloor)) {
                return false;
            }
        }
        return particleFilterEngine == null
                || !pfInitialized
                || particleFilterEngine.canForceFloor(absoluteFloor);
    }

    private void activateFloorOnlyResyncWindow(int absoluteFloor, long timestampMs) {
        activateFloorOnlyResyncWindow(absoluteFloor, timestampMs, null, "none");
    }

    private void activateFloorOnlyResyncWindow(
            int absoluteFloor,
            long timestampMs,
            @Nullable LatLng landingLatLng,
            @NonNull String anchorSource
    ) {
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        int boundedAbsoluteFloor = clampAbsoluteFloorToVenue(absoluteFloor);
        floorOnlyResyncAbsoluteFloor = boundedAbsoluteFloor;
        floorOnlyResyncWindowUntilMs = safeTimestampMs + FLOOR_ONLY_RESYNC_WINDOW_MS;
        pendingDisplayFloorResetAbsoluteFloor = boundedAbsoluteFloor;
        pendingDisplayFloorResetUntilMs = safeTimestampMs + DISPLAY_FLOOR_RESET_WINDOW_MS;
        pendingDisplayFloorResetLandingLatLng = landingLatLng;
        pendingDisplayFloorResetAnchorSource = anchorSource;
        pendingCommittedDisplayFloorAbsolute = boundedAbsoluteFloor;
        floorSwitchPending = true;
        lastFloorSwitchBlockReason = "awaiting_display_commit";
        absoluteFloorTransitionResolver.reset();
        resetStableAbsoluteFloorConsensus();
        resetWifiFloorBootstrapConsensus();
        logFloorSwitchTrace(
                "DISPLAY_RESET_TOKEN_ISSUED=true",
                boundedAbsoluteFloor,
                anchorSource,
                landingLatLng,
                landingLatLng != null && MapConstraintRepository.isPointLegal(landingLatLng, boundedAbsoluteFloor),
                true,
                false,
                "awaiting_display_commit"
        );
    }

    private void resetFloorOnlyResyncWindow() {
        floorOnlyResyncAbsoluteFloor = Integer.MIN_VALUE;
        floorOnlyResyncWindowUntilMs = Long.MIN_VALUE;
        clearPendingDisplayFloorResetToken();
        floorSwitchPending = false;
        pendingCommittedDisplayFloorAbsolute = Integer.MIN_VALUE;
        lastFloorSwitchBlockReason = "none";
    }

    @Nullable
    private Integer getActiveFloorOnlyResyncFloor(long timestampMs) {
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        if (floorOnlyResyncAbsoluteFloor == Integer.MIN_VALUE
                || safeTimestampMs > floorOnlyResyncWindowUntilMs) {
            return null;
        }
        return floorOnlyResyncAbsoluteFloor;
    }

    private FloorOnlyResyncPolicy resolveFloorOnlyResyncPolicy(
            @NonNull String debugSource,
            @Nullable Integer sanitizedFloorPrior,
            float accuracyMeters,
            long timestampMs
    ) {
        if (!"WIFI".equals(debugSource) && !"GNSS".equals(debugSource)) {
            return new FloorOnlyResyncPolicy(false, sanitizedFloorPrior, accuracyMeters);
        }
        Integer activeResyncFloor = getActiveFloorOnlyResyncFloor(timestampMs);
        if (activeResyncFloor == null) {
            return new FloorOnlyResyncPolicy(false, sanitizedFloorPrior, accuracyMeters);
        }
        if (sanitizedFloorPrior != null && sanitizedFloorPrior != activeResyncFloor) {
            return new FloorOnlyResyncPolicy(true, sanitizedFloorPrior, accuracyMeters);
        }
        return new FloorOnlyResyncPolicy(
                false,
                activeResyncFloor,
                Math.max(accuracyMeters, FLOOR_ONLY_RESYNC_ABSOLUTE_FIX_STD_M)
        );
    }

    private void clearPendingDisplayFloorResetToken() {
        pendingDisplayFloorResetAbsoluteFloor = Integer.MIN_VALUE;
        pendingDisplayFloorResetUntilMs = Long.MIN_VALUE;
        pendingDisplayFloorResetLandingLatLng = null;
        pendingDisplayFloorResetAnchorSource = "none";
    }

    private void expirePendingDisplayFloorResetIfNeeded(long timestampMs) {
        if (pendingDisplayFloorResetAbsoluteFloor == Integer.MIN_VALUE
                || timestampMs <= pendingDisplayFloorResetUntilMs) {
            return;
        }
        int expiredFloor = pendingDisplayFloorResetAbsoluteFloor;
        LatLng expiredLanding = pendingDisplayFloorResetLandingLatLng;
        String expiredAnchorSource = pendingDisplayFloorResetAnchorSource;
        clearPendingDisplayFloorResetToken();
        floorSwitchPending = false;
        pendingCommittedDisplayFloorAbsolute = Integer.MIN_VALUE;
        lastFloorSwitchBlockReason = "display_reset_token_expired";
        logFloorSwitchTrace(
                "DISPLAY_RESET_TOKEN_ISSUED=false",
                expiredFloor,
                expiredAnchorSource,
                expiredLanding,
                expiredLanding != null && MapConstraintRepository.isPointLegal(expiredLanding, expiredFloor),
                false,
                false,
                "display_reset_token_expired"
        );
    }

    public synchronized boolean hasPendingDisplayFloorReset(int absoluteFloor, long timestampMs) {
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        expirePendingDisplayFloorResetIfNeeded(safeTimestampMs);
        return pendingDisplayFloorResetAbsoluteFloor == absoluteFloor
                && safeTimestampMs <= pendingDisplayFloorResetUntilMs;
    }

    @Nullable
    public synchronized LatLng peekPendingDisplayFloorResetLanding(int absoluteFloor, long timestampMs) {
        if (!hasPendingDisplayFloorReset(absoluteFloor, timestampMs)) {
            return null;
        }
        return pendingDisplayFloorResetLandingLatLng;
    }

    public synchronized boolean commitPendingDisplayFloorReset(
            int absoluteFloor,
            long timestampMs,
            @Nullable LatLng displayedLocation
    ) {
        String anchorSource = pendingDisplayFloorResetAnchorSource;
        boolean consumed = consumePendingDisplayFloorReset(absoluteFloor, timestampMs);
        if (!consumed) {
            return false;
        }
        committedDisplayFloorAbsolute = clampAbsoluteFloorToVenue(absoluteFloor);
        floorSwitchPending = false;
        pendingCommittedDisplayFloorAbsolute = Integer.MIN_VALUE;
        lastFloorSwitchBlockReason = "none";
        logFloorSwitchTrace(
                "DISPLAY_RESET_TOKEN_CONSUMED=true",
                committedDisplayFloorAbsolute,
                anchorSource,
                displayedLocation,
                displayedLocation != null && MapConstraintRepository.isPointLegal(displayedLocation, committedDisplayFloorAbsolute),
                false,
                true,
                "none"
        );
        return true;
    }

    public synchronized boolean consumePendingDisplayFloorReset(int absoluteFloor, long timestampMs) {
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        expirePendingDisplayFloorResetIfNeeded(safeTimestampMs);
        if (pendingDisplayFloorResetAbsoluteFloor != absoluteFloor
                || safeTimestampMs > pendingDisplayFloorResetUntilMs) {
            return false;
        }
        clearPendingDisplayFloorResetToken();
        return true;
    }

    private void clampLatestFusedPoseFloorToVenueIfNeeded(long timestampMs) {
        if (latestFusedPose == null) {
            return;
        }
        int clampedFloor = clampAbsoluteFloorToVenue(latestFusedPose.getFloor());
        if (clampedFloor == latestFusedPose.getFloor()) {
            return;
        }
        if (applyAbsoluteFloorKeepingCurrentXy(clampedFloor, timestampMs)) {
            recordLatestFusedPoseIfNeeded();
        }
    }

    private void maybeCalibrateFloorOffset(
            @Nullable Integer absoluteFloor,
            boolean trustedAbsoluteFloor,
            long timestampMs
    ) {
        if (!trustedAbsoluteFloor || absoluteFloor == null || this.pdrProcessing == null) {
            return;
        }
        boolean wasFloorInitialized = isFloorOffsetInitialized;
        int boundedAbsoluteFloor = clampAbsoluteFloorToVenue(absoluteFloor);
        int previousAbsoluteFloor = getAbsoluteCurrentFloor();
        int desiredOffset = boundedAbsoluteFloor - getRelativeCurrentFloor();
        if (!isFloorOffsetInitialized || getAbsoluteCurrentFloor() != boundedAbsoluteFloor) {
            this.pdrFloorOffset = desiredOffset;
            this.isFloorOffsetInitialized = true;
            this.blockHistoricalPoseFloorSeedUntilTrustedFix = false;
            clampCumulativeFloorTrackerToVenueBounds();
            resetWifiFloorBootstrapConsensus();
            if (previousAbsoluteFloor != boundedAbsoluteFloor) {
                activateFloorOnlyResyncWindow(boundedAbsoluteFloor, timestampMs);
            }
            if (!wasFloorInitialized) {
                noteFloorConsensusDecision("current_floor_locked", timestampMs);
            }
            lastFloorAnchorState = wasFloorInitialized
                    ? "corrected_floor_anchor"
                    : "locked_absolute_floor";
            logFloorDiagnostic(
                    "floor_calibration_initialized",
                    boundedAbsoluteFloor,
                    lastFloorAnchorState,
                    trustedAbsoluteFloor ? "accepted_floor_prior" : "untrusted"
            );
        }
    }

    private void clearProvisionalFloorBootstrapState() {
        provisionalFloorBootstrapActive = false;
        provisionalFloorBootstrapFloor = Integer.MIN_VALUE;
        provisionalFloorBootstrapSource = "none";
    }

    private void noteBootstrapFloorSeed(
            int floor,
            boolean trustedFloorSeed,
            @NonNull String source,
            long timestampMs
    ) {
        if (trustedFloorSeed) {
            clearProvisionalFloorBootstrapState();
            logInfoSafely(
                    "SensorFusion",
                    "BOOTSTRAP:trusted_floor_seed floor=" + floor
                            + " source=" + source
                            + " ts=" + timestampMs
            );
            return;
        }
        provisionalFloorBootstrapActive = true;
        provisionalFloorBootstrapFloor = floor;
        provisionalFloorBootstrapSource = source;
        logInfoSafely(
                "SensorFusion",
                "BOOTSTRAP:provisional_floor_seed floor=" + floor
                        + " source=" + source
                        + " ts=" + timestampMs
        );
    }

    private static boolean isPlausibleProvisionalFloorCorrection(
            int provisionalFloor,
            int reportedFloor
    ) {
        return Math.abs(reportedFloor - provisionalFloor) <= 2;
    }

    private boolean shouldApplyProvisionalFloorReconcile(
            @NonNull String debugSource,
            @Nullable Integer acceptedFloorPrior
    ) {
        if (!provisionalFloorBootstrapActive
                || acceptedFloorPrior == null
                || latestFusedPose == null
                || provisionalFloorBootstrapFloor == Integer.MIN_VALUE) {
            return false;
        }
        if (!"WIFI".equals(debugSource) && !"GNSS".equals(debugSource)) {
            return false;
        }
        if (latestFusedPose.getFloor() == acceptedFloorPrior
                || provisionalFloorBootstrapFloor == acceptedFloorPrior) {
            return false;
        }
        return isPlausibleProvisionalFloorCorrection(
                provisionalFloorBootstrapFloor,
                acceptedFloorPrior
        );
    }

    private void noteProvisionalFloorReconcileApplied(
            int floor,
            @NonNull String source,
            long timestampMs
    ) {
        logInfoSafely(
                "SensorFusion",
                "BOOTSTRAP:provisional_floor_replaced_by_absolute_floor oldFloor="
                        + provisionalFloorBootstrapFloor
                        + " newFloor=" + floor
                        + " source=" + source
                        + " ts=" + timestampMs
        );
        logInfoSafely(
                "SensorFusion",
                "FLOOR:provisional_reconcile_applied floor=" + floor
                        + " source=" + source
                        + " ts=" + timestampMs
        );
        clearProvisionalFloorBootstrapState();
    }

    @Nullable
    private Integer resolveBootstrapReadyFloorPrior(
            @NonNull String debugSource,
            @Nullable Integer sanitizedFloorPrior,
            float accuracyMeters,
            long timestampMs
    ) {
        if (!"WIFI".equals(debugSource)) {
            return sanitizedFloorPrior;
        }
        if (sanitizedFloorPrior == null) {
            resetWifiFloorBootstrapConsensus();
            return null;
        }
        if (isFloorOffsetInitialized) {
            resetWifiFloorBootstrapConsensus();
            return sanitizedFloorPrior;
        }
        if (sanitizedFloorPrior == 0
                && !isGroundFloorWifiBootstrapContextReady()) {
            logInfoSafely(
                    "SensorFusion",
                    "WiFi floor 0 skipped from bootstrap; venue context does not support trusted GF lock"
            );
            resetWifiFloorBootstrapConsensus();
            return null;
        }
        if (shouldAcceptWifiBootstrapConsensus(sanitizedFloorPrior, accuracyMeters, timestampMs)) {
            if (sanitizedFloorPrior == 0) {
                logInfoSafely(
                        "SensorFusion",
                        "Accepted ground-floor WiFi bootstrap candidate floor=0 with strengthened evidence"
                );
            }
            return sanitizedFloorPrior;
        }
        return null;
    }

    // Visible for tests.
    boolean shouldAcceptWifiBootstrapConsensus(
            int reportedFloor,
            float accuracyMeters,
            long timestampMs
    ) {
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        if (lastWifiBootstrapCandidateTimestampMs != Long.MIN_VALUE
                && safeTimestampMs - lastWifiBootstrapCandidateTimestampMs > WIFI_BOOTSTRAP_CONSENSUS_WINDOW_MS) {
            resetWifiFloorBootstrapConsensus();
        }

        if (pendingWifiBootstrapFloor != null && pendingWifiBootstrapFloor == reportedFloor) {
            pendingWifiBootstrapCount++;
        } else {
            pendingWifiBootstrapFloor = reportedFloor;
            pendingWifiBootstrapCount = 1;
            pendingWifiBootstrapFirstTimestampMs = safeTimestampMs;
        }
        lastWifiBootstrapCandidateTimestampMs = safeTimestampMs;
        long consensusDurationMs = pendingWifiBootstrapFirstTimestampMs == Long.MIN_VALUE
                ? 0L
                : Math.max(0L, safeTimestampMs - pendingWifiBootstrapFirstTimestampMs);
        int requiredConfirmations = resolveWifiBootstrapRequiredConfirmations(reportedFloor);
        long requiredDurationMs = resolveWifiBootstrapRequiredDurationMs(reportedFloor);
        return pendingWifiBootstrapCount >= requiredConfirmations
                && consensusDurationMs >= requiredDurationMs;
    }

    private void resetWifiFloorBootstrapConsensus() {
        pendingWifiBootstrapFloor = null;
        pendingWifiBootstrapCount = 0;
        pendingWifiBootstrapFirstTimestampMs = Long.MIN_VALUE;
        lastWifiBootstrapCandidateTimestampMs = Long.MIN_VALUE;
    }

    // Visible for tests.
    void resetWifiFloorBootstrapConsensusForTesting() {
        resetWifiFloorBootstrapConsensus();
    }

    static int resolveWifiBootstrapRequiredConfirmations(int reportedFloor) {
        return reportedFloor == 0
                ? GROUND_FLOOR_WIFI_BOOTSTRAP_REQUIRED_CONFIRMATIONS
                : WIFI_BOOTSTRAP_REQUIRED_CONFIRMATIONS;
    }

    static long resolveWifiBootstrapRequiredDurationMs(int reportedFloor) {
        return reportedFloor == 0
                ? GROUND_FLOOR_WIFI_BOOTSTRAP_REQUIRED_DURATION_MS
                : 0L;
    }

    boolean isGroundFloorWifiBootstrapContextReady() {
        return isFloorReadyForConsensus(0);
    }

    @Nullable
    private Integer resolveInitializationFloorForAbsoluteFix(
            @Nullable Integer initializationFloor,
            @Nullable Integer sanitizedFloorPrior
    ) {
        if (sanitizedFloorPrior != null) {
            return sanitizedFloorPrior;
        }
        return initializationFloor == null ? null : clampAbsoluteFloorToVenue(initializationFloor);
    }

    @Nullable
    private Integer sanitizeReportedAbsoluteFloorPrior(
            @Nullable Integer reportedFloor,
            @NonNull String debugSource
    ) {
        if (reportedFloor == null) {
            return null;
        }
        int boundedFloor = clampAbsoluteFloorToVenue(reportedFloor);
        if (boundedFloor != reportedFloor) {
            Log.w(
                    "SensorFusion",
                    "Clamping reported absolute floor source="
                            + debugSource
                            + " reportedFloor="
                            + reportedFloor
                            + " boundedFloor="
                            + boundedFloor
            );
        }
        if ("WIFI".equals(debugSource)) {
            return sanitizeWifiFloorPrior(boundedFloor);
        }
        return boundedFloor;
    }

    @Nullable
    private Integer sanitizeWifiFloorPrior(int reportedFloor) {
        if (!pfInitialized || !isFloorOffsetInitialized || !cumulativeFloorTracker.isInitialized()) {
            return reportedFloor;
        }
        int currentAbsoluteFloor = getAbsoluteCurrentFloor();
        if (reportedFloor == currentAbsoluteFloor) {
            return reportedFloor;
        }
        if (Math.abs(reportedFloor - currentAbsoluteFloor) < WIFI_MULTI_FLOOR_SANITY_GAP) {
            return reportedFloor;
        }
        if (hasRecentStrongBarometerFloorTransitionEvidence(currentAbsoluteFloor, reportedFloor)) {
            return reportedFloor;
        }
        if (isWifiFloorPlausibleWithBarometer(
                reportedFloor,
                pdrFloorOffset,
                cumulativeFloorTracker.getFilteredRelativeElevationMeters(),
                getActiveFloorHeightMeters(),
                cumulativeFloorTracker.isInitialized()
        )) {
            return reportedFloor;
        }
        Log.w(
                "SensorFusion",
                "Rejecting implausible WiFi floor jump currentFloor="
                        + currentAbsoluteFloor
                        + " reportedFloor="
                        + reportedFloor
                        + " filteredElevation="
                        + cumulativeFloorTracker.getFilteredRelativeElevationMeters()
                        + " floorHeight="
                        + getActiveFloorHeightMeters()
        );
        return null;
    }

    static boolean isWifiFloorPlausibleWithBarometer(
            int reportedAbsoluteFloor,
            int pdrFloorOffset,
            float filteredRelativeElevationMeters,
            float floorHeightMeters,
            boolean barometerInitialized
    ) {
        if (!barometerInitialized || !Float.isFinite(filteredRelativeElevationMeters)) {
            return true;
        }
        if (!Float.isFinite(floorHeightMeters) || floorHeightMeters <= 0f) {
            return false;
        }
        int targetRelativeFloor = reportedAbsoluteFloor - pdrFloorOffset;
        float targetRelativeElevationMeters = targetRelativeFloor * floorHeightMeters;
        float allowedElevationErrorMeters = CumulativeFloorTracker.requiredTravelForFloorChange(
                floorHeightMeters
        );
        return Math.abs(filteredRelativeElevationMeters - targetRelativeElevationMeters)
                <= allowedElevationErrorMeters;
    }

    @Nullable
    private Integer resolveAcceptedAbsoluteFloorPrior(
            double latitudeDeg,
            double longitudeDeg,
            @Nullable Integer reportedFloor,
            long timestampMs
    ) {
        if (reportedFloor == null) {
            clearPendingElevatorSuppressionEscapeFloor();
            absoluteFloorTransitionResolver.reset();
            resetStableAbsoluteFloorConsensus();
            if (!isFloorOffsetInitialized) {
                lastFloorAnchorState = "unresolved";
            }
            noteFloorConsensusDecision(
                    isFloorOffsetInitialized ? "current_floor_locked" : "relative_only",
                    timestampMs
            );
            return null;
        }
        if (consumePendingElevatorSuppressionEscapeFloor(reportedFloor, timestampMs)) {
            absoluteFloorTransitionResolver.reset();
            resetStableAbsoluteFloorConsensus();
            noteFloorConsensusDecision("recovered_from_elevator_escape", timestampMs);
            logFloorDiagnostic(
                    "accepted_absolute_floor_prior",
                    reportedFloor,
                    "recovered_from_elevator_escape",
                    "accepted_floor_prior"
            );
            return reportedFloor;
        }
        int currentFloor = getAbsoluteCurrentFloor();
        boolean transitionAllowed = reportedFloor == currentFloor
                || allowsAbsoluteFixFloorTransition(
                new LatLng(latitudeDeg, longitudeDeg),
                currentFloor,
                reportedFloor
        );
        boolean recoveryCandidate = isWrongLockedFloorRecoveryCandidate(
                currentFloor,
                reportedFloor
        );
        if (!isFloorOffsetInitialized) {
            absoluteFloorTransitionResolver.reset();
            boolean provisionalReconcileCandidate = provisionalFloorBootstrapActive
                    && provisionalFloorBootstrapFloor != Integer.MIN_VALUE
                    && reportedFloor != provisionalFloorBootstrapFloor;
            if (provisionalReconcileCandidate
                    && hasConflictingBarometerFloorEvidence(currentFloor, reportedFloor)) {
                logInfoSafely(
                        "SensorFusion",
                        "FLOOR:provisional_reconcile_rejected_barometer_conflict"
                                + " currentFloor=" + currentFloor
                                + " reportedFloor=" + reportedFloor
                                + " provisionalFloor=" + provisionalFloorBootstrapFloor
                );
            } else if (provisionalReconcileCandidate
                    && isPlausibleProvisionalFloorCorrection(provisionalFloorBootstrapFloor, reportedFloor)) {
                logInfoSafely(
                        "SensorFusion",
                        "FLOOR:provisional_reconcile_pending"
                                + " provisionalFloor=" + provisionalFloorBootstrapFloor
                                + " reportedFloor=" + reportedFloor
                                + " source=" + provisionalFloorBootstrapSource
                );
            }
            return resolveInitialAbsoluteFloorLock(
                    currentFloor,
                    reportedFloor,
                    timestampMs,
                    provisionalReconcileCandidate
            );
        }
        if (reportedFloor == currentFloor) {
            absoluteFloorTransitionResolver.reset();
            resetStableAbsoluteFloorConsensus();
            noteFloorConsensusDecision("current_floor_locked", timestampMs);
            logFloorDiagnostic(
                    "accepted_absolute_floor_prior",
                    reportedFloor,
                    "current_floor_locked",
                    "accepted_floor_prior"
            );
            return reportedFloor;
        }
        if (shouldRecoverFromWrongLockedFloor(currentFloor, reportedFloor, timestampMs)) {
            absoluteFloorTransitionResolver.reset();
            noteFloorConsensusDecision("recovered_from_wrong_lock", timestampMs);
            logFloorDiagnostic(
                    "accepted_absolute_floor_prior",
                    reportedFloor,
                    "recovered_from_wrong_lock",
                    "accepted_floor_prior"
            );
            return reportedFloor;
        }
        if (!transitionAllowed) {
            absoluteFloorTransitionResolver.reset();
            if (recoveryCandidate && "initial_lock_pending".equals(lastFloorConsensus)) {
                return null;
            }
            resetStableAbsoluteFloorConsensus();
            lastFloorAnchorState = "blocked_by_inconsistent_floor_map";
            noteFloorConsensusDecision("blocked_by_map_transition", timestampMs);
            return null;
        }
        if (hasRecentStrongBarometerFloorTransitionEvidence(currentFloor, reportedFloor)) {
            absoluteFloorTransitionResolver.reset();
            resetStableAbsoluteFloorConsensus();
            noteFloorConsensusDecision("promoted_by_barometer", timestampMs);
            logFloorDiagnostic(
                    "accepted_absolute_floor_prior",
                    reportedFloor,
                    "promoted_by_barometer",
                    "accepted_floor_prior"
            );
            return reportedFloor;
        }
        Integer stableConsensusFloor = resolveStableAbsoluteFloorConsensus(
                currentFloor,
                reportedFloor,
                timestampMs
        );
        if (stableConsensusFloor != null) {
            absoluteFloorTransitionResolver.reset();
            noteFloorConsensusDecision("promoted_by_stable_wifi_consensus", timestampMs);
            logFloorDiagnostic(
                    "accepted_absolute_floor_prior",
                    stableConsensusFloor,
                    "promoted_by_stable_wifi_consensus",
                    "stable_candidate"
            );
            return stableConsensusFloor;
        }
        noteFloorConsensusDecision("blocked_by_no_barometer", timestampMs);
        return null;
    }

    private void resetStableAbsoluteFloorConsensus() {
        pendingStableAbsoluteFloorCandidate = null;
        pendingStableAbsoluteFloorCount = 0;
        pendingStableAbsoluteFloorFirstTimestampMs = Long.MIN_VALUE;
        pendingStableAbsoluteFloorLastTimestampMs = Long.MIN_VALUE;
    }

    private void noteFloorConsensusDecision(@NonNull String decision, long timestampMs) {
        lastFloorConsensus = decision;
        lastFloorSource = resolveFloorSourceDebug(timestampMs);
    }

    @NonNull
    private String resolveFloorSourceDebug(long timestampMs) {
        if (!isFloorOffsetInitialized) {
            return "relative_only";
        }
        return getActiveFloorOnlyResyncFloor(timestampMs) != null ? "venue_synced" : "absolute";
    }

    @Nullable
    private Integer resolveInitialAbsoluteFloorLock(
            int currentFloor,
            int reportedFloor,
            long timestampMs,
            boolean provisionalReconcileCandidate
    ) {
        if (!isFloorReadyForConsensus(reportedFloor)) {
            resetStableAbsoluteFloorConsensus();
            lastFloorAnchorState = "unresolved";
            noteFloorConsensusDecision("initializing", timestampMs);
            logInfoSafely(
                    "SensorFusion",
                    "FLOOR:initial_lock_pending reason="
                            + (provisionalReconcileCandidate
                            ? "unknown_floor_from_provisional_bootstrap"
                            : "unknown_floor")
                            + " reportedFloor=" + reportedFloor
            );
            return null;
        }
        if (hasConflictingBarometerFloorEvidence(currentFloor, reportedFloor)) {
            resetStableAbsoluteFloorConsensus();
            lastFloorAnchorState = "unresolved";
            noteFloorConsensusDecision("initializing", timestampMs);
            logInfoSafely(
                    "SensorFusion",
                    "FLOOR:initial_lock_rejected_barometer_conflict"
                            + " currentFloor=" + currentFloor
                            + " reportedFloor=" + reportedFloor
                            + " provisionalBootstrap=" + provisionalReconcileCandidate
            );
            return null;
        }
        int requiredConfirmations = INITIAL_ABSOLUTE_FLOOR_LOCK_REQUIRED_CONFIRMATIONS;
        long requiredDurationMs = INITIAL_ABSOLUTE_FLOOR_LOCK_REQUIRED_DURATION_MS;
        if (shouldAccelerateInitialAbsoluteFloorLock(currentFloor, reportedFloor, timestampMs)) {
            requiredConfirmations = FAST_INITIAL_ABSOLUTE_FLOOR_LOCK_REQUIRED_CONFIRMATIONS;
            requiredDurationMs = FAST_INITIAL_ABSOLUTE_FLOOR_LOCK_REQUIRED_DURATION_MS;
        }
        Integer acceptedFloor = resolvePendingAbsoluteFloorConsensus(
                reportedFloor,
                timestampMs,
                requiredConfirmations,
                requiredDurationMs
        );
        if (acceptedFloor == null) {
            lastFloorAnchorState = "bootstrap_pending";
            noteFloorConsensusDecision("initial_lock_pending", timestampMs);
            logInfoSafely(
                    "SensorFusion",
                    "FLOOR:initial_lock_pending reason="
                            + (provisionalReconcileCandidate
                            ? "awaiting_consensus_from_provisional_bootstrap"
                            : "awaiting_consensus")
                            + " reportedFloor=" + reportedFloor
                            + " confirmations=" + pendingStableAbsoluteFloorCount
                            + " requiredConfirmations=" + requiredConfirmations
                            + " elapsedMs=" + resolvePendingAbsoluteFloorConsensusElapsedMs(timestampMs)
                            + " requiredDurationMs=" + requiredDurationMs
            );
        } else {
            noteFloorConsensusDecision("initial_lock_acquired", timestampMs);
            logInfoSafely(
                    "SensorFusion",
                    "FLOOR:initial_lock_acquired source=consistent_absolute_floor"
                            + " floor=" + acceptedFloor
                            + " confirmations=" + requiredConfirmations
                            + " durationMs=" + requiredDurationMs
            );
            logFloorDiagnostic(
                    "accepted_absolute_floor_prior",
                    acceptedFloor,
                    "initial_absolute_floor_lock",
                    "stable_candidate"
            );
        }
        return acceptedFloor;
    }

    private boolean shouldAccelerateInitialAbsoluteFloorLock(
            int currentFloor,
            int reportedFloor,
            long timestampMs
    ) {
        if (elevator || elevatorFloorSessionActive) {
            return false;
        }
        if (reportedFloor == currentFloor) {
            return true;
        }
        if (pendingWifiBootstrapFloor != null && pendingWifiBootstrapFloor == reportedFloor) {
            return true;
        }
        int activeMapFloor = MapConstraintRepository.getActiveFloor();
        if (cumulativeFloorTracker.isInitialized()
                && activeMapFloor != -1
                && activeMapFloor == reportedFloor) {
            return true;
        }
        return pendingStableAbsoluteFloorCandidate != null
                && pendingStableAbsoluteFloorCandidate == reportedFloor
                && pendingStableAbsoluteFloorLastTimestampMs != Long.MIN_VALUE
                && Math.max(0L, timestampMs - pendingStableAbsoluteFloorLastTimestampMs)
                <= ABSOLUTE_FLOOR_STABLE_CONSENSUS_MAX_GAP_MS;
    }

    private long resolvePendingAbsoluteFloorConsensusElapsedMs(long timestampMs) {
        if (pendingStableAbsoluteFloorFirstTimestampMs == Long.MIN_VALUE) {
            return 0L;
        }
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        return Math.max(0L, safeTimestampMs - pendingStableAbsoluteFloorFirstTimestampMs);
    }

    private boolean shouldRecoverFromWrongLockedFloor(
            int currentFloor,
            int reportedFloor,
            long timestampMs
    ) {
        if (!isWrongLockedFloorRecoveryCandidate(currentFloor, reportedFloor)) {
            resetStableAbsoluteFloorConsensus();
            return false;
        }
        Integer recoveredFloor = resolvePendingAbsoluteFloorConsensus(
                reportedFloor,
                timestampMs,
                ABSOLUTE_FLOOR_STABLE_CONSENSUS_REQUIRED_CONFIRMATIONS,
                ABSOLUTE_FLOOR_STABLE_CONSENSUS_REQUIRED_DURATION_MS
        );
        if (recoveredFloor == null) {
            noteFloorConsensusDecision("initial_lock_pending", timestampMs);
            return false;
        }
        logFloorDiagnostic(
                "wrong_floor_recovery_accepted",
                recoveredFloor,
                "recovered_from_wrong_lock",
                "stable_candidate"
        );
        return true;
    }

    private boolean isWrongLockedFloorRecoveryCandidate(int currentFloor, int reportedFloor) {
        return reportedFloor != currentFloor
                && isFloorReadyForConsensus(reportedFloor)
                && !hasConflictingBarometerFloorEvidence(currentFloor, reportedFloor);
    }

    private boolean isFloorReadyForConsensus(int floor) {
        if (MapConstraintRepository.hasKnownFloor(floor)) {
            return true;
        }
        int activeMapFloor = MapConstraintRepository.getActiveFloor();
        if (activeMapFloor != -1 && activeMapFloor == floor) {
            return true;
        }
        try {
            IndoorMapManager.FloorBounds floorBounds = IndoorMapManager.resolveFloorBounds(collectionVenue);
            return floorBounds != null
                    && floor >= floorBounds.minFloor
                    && floor <= floorBounds.maxFloor;
        } catch (ExceptionInInitializerError | NoClassDefFoundError ignored) {
            return false;
        }
    }

    private boolean hasConflictingBarometerFloorEvidence(int currentFloor, int reportedFloor) {
        if (currentFloor == reportedFloor || !cumulativeFloorTracker.isInitialized()) {
            return false;
        }
        return hasRecentStrongBarometerFloorTransitionEvidence(reportedFloor, currentFloor)
                && !hasRecentStrongBarometerFloorTransitionEvidence(currentFloor, reportedFloor);
    }

    private boolean isLowConfidenceFloorEstimate() {
        return latestFusedPose == null
                || !Double.isFinite(latestFusedPose.getConfidence())
                || latestFusedPose.getConfidence() <= WRONG_LOCK_RECOVERY_MAX_CONFIDENCE;
    }

    @Nullable
    private Integer resolvePendingAbsoluteFloorConsensus(
            int reportedFloor,
            long timestampMs,
            int requiredConfirmations,
            long requiredDurationMs
    ) {
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        if (pendingStableAbsoluteFloorLastTimestampMs != Long.MIN_VALUE
                && safeTimestampMs - pendingStableAbsoluteFloorLastTimestampMs
                > ABSOLUTE_FLOOR_STABLE_CONSENSUS_MAX_GAP_MS) {
            resetStableAbsoluteFloorConsensus();
        }
        if (pendingStableAbsoluteFloorCandidate != null
                && pendingStableAbsoluteFloorCandidate == reportedFloor) {
            pendingStableAbsoluteFloorCount++;
        } else {
            pendingStableAbsoluteFloorCandidate = reportedFloor;
            pendingStableAbsoluteFloorCount = 1;
            pendingStableAbsoluteFloorFirstTimestampMs = safeTimestampMs;
        }
        pendingStableAbsoluteFloorLastTimestampMs = safeTimestampMs;
        long consensusDurationMs = pendingStableAbsoluteFloorFirstTimestampMs == Long.MIN_VALUE
                ? 0L
                : Math.max(0L, safeTimestampMs - pendingStableAbsoluteFloorFirstTimestampMs);
        if (pendingStableAbsoluteFloorCount < requiredConfirmations) {
            return null;
        }
        if (consensusDurationMs < requiredDurationMs) {
            return null;
        }
        resetStableAbsoluteFloorConsensus();
        return reportedFloor;
    }

    @Nullable
    private Integer resolveStableAbsoluteFloorConsensus(
            int currentFloor,
            int reportedFloor,
            long timestampMs
    ) {
        if (reportedFloor == currentFloor) {
            resetStableAbsoluteFloorConsensus();
            return reportedFloor;
        }
        return resolvePendingAbsoluteFloorConsensus(
                reportedFloor,
                timestampMs,
                ABSOLUTE_FLOOR_STABLE_CONSENSUS_REQUIRED_CONFIRMATIONS,
                ABSOLUTE_FLOOR_STABLE_CONSENSUS_REQUIRED_DURATION_MS
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
        if (shouldFailOpenBarometerFloorTransition(currentFloor, newFloor)) {
            return true;
        }
        LatLng previousLatLng = getLatLngForFusedPose(latestFusedPose);
        if (previousLatLng == null) {
            previousLatLng = absoluteFixLatLng;
        }
        boolean liftAvailable = MapConstraintRepository.hasLiftConstraints(currentFloor)
                || MapConstraintRepository.hasLiftConstraints(newFloor);
        boolean stairsAvailable = MapConstraintRepository.hasStairsConstraints(currentFloor)
                || MapConstraintRepository.hasStairsConstraints(newFloor);
        TransitionPreference transitionPreference = resolveTransitionPreference(
                getElevator(),
                UtilFunctions.distanceBetweenPoints(previousLatLng, absoluteFixLatLng),
                liftAvailable,
                stairsAvailable
        );
        boolean strongBarometerEvidence = hasRecentStrongBarometerFloorTransitionEvidence(
                currentFloor,
                newFloor
        );
        return isTransitionSatisfied(
                transitionPreference,
                previousLatLng,
                absoluteFixLatLng,
                currentFloor,
                newFloor,
                strongBarometerEvidence
        );
    }

    @Nullable
    private Integer resolveStep1InitializationFloor(@Nullable Integer preferredFloor) {
        if (preferredFloor != null) {
            Integer resolvedFloor = clampAbsoluteFloorToVenue(preferredFloor);
            logFloorDiagnostic(
                    "step1_initialization_floor_resolved",
                    resolvedFloor,
                    "preferred_floor",
                    "accepted_floor_prior"
            );
            return resolvedFloor;
        }
        if (isFloorOffsetInitialized) {
            Integer resolvedFloor = getAbsoluteCurrentFloor();
            logFloorDiagnostic(
                    "step1_initialization_floor_resolved",
                    resolvedFloor,
                    "trusted_current_absolute_floor",
                    "trusted_current_floor"
            );
            return resolvedFloor;
        }
        Integer recoveryFloorCandidate = resolveBestAvailableUntrustedFloorSeed();
        if (recoveryFloorCandidate != null) {
            String resolutionSource = "fallback";
            String reason = "untrusted_seed";
            if (pendingStableAbsoluteFloorCandidate != null
                    && isFloorReadyForConsensus(pendingStableAbsoluteFloorCandidate)
                    && clampAbsoluteFloorToVenue(pendingStableAbsoluteFloorCandidate) == recoveryFloorCandidate) {
                resolutionSource = "stable_candidate";
                reason = "pending_stable_candidate";
            } else if (pendingWifiBootstrapFloor != null
                    && isFloorReadyForConsensus(pendingWifiBootstrapFloor)
                    && clampAbsoluteFloorToVenue(pendingWifiBootstrapFloor) == recoveryFloorCandidate) {
                resolutionSource = "wifi_bootstrap";
                reason = "pending_wifi_bootstrap";
            } else if (!blockHistoricalPoseFloorSeedUntilTrustedFix && latestFusedPose != null) {
                resolutionSource = "fused_pose";
                reason = "latest_fused_pose";
            }
            logFloorDiagnostic(
                    "step1_initialization_floor_resolved",
                    recoveryFloorCandidate,
                    reason,
                    resolutionSource
            );
            return recoveryFloorCandidate;
        }
        logFloorDiagnostic(
                "step1_initialization_floor_resolved",
                null,
                "no_floor_seed",
                "null"
        );
        return null;
    }

    @Nullable
    private Integer resolveBestAvailableUntrustedFloorSeed() {
        if (!blockHistoricalPoseFloorSeedUntilTrustedFix && cumulativeFloorTracker.isInitialized()) {
            int relativeFloor = getRelativeCurrentFloor();
            int candidateFloor = clampAbsoluteFloorToVenue(relativeFloor + pdrFloorOffset);
            if (relativeFloor != 0 || pdrFloorOffset != 0 || candidateFloor != 0) {
                return candidateFloor;
            }
        }
        int activeMapFloor = MapConstraintRepository.getActiveFloor();
        if (activeMapFloor != -1 && isFloorReadyForConsensus(activeMapFloor)) {
            return clampAbsoluteFloorToVenue(activeMapFloor);
        }
        if (pendingStableAbsoluteFloorCandidate != null
                && isFloorReadyForConsensus(pendingStableAbsoluteFloorCandidate)) {
            return clampAbsoluteFloorToVenue(pendingStableAbsoluteFloorCandidate);
        }
        if (pendingWifiBootstrapFloor != null
                && isFloorReadyForConsensus(pendingWifiBootstrapFloor)) {
            return clampAbsoluteFloorToVenue(pendingWifiBootstrapFloor);
        }
        if (!blockHistoricalPoseFloorSeedUntilTrustedFix && latestFusedPose != null) {
            return clampAbsoluteFloorToVenue(latestFusedPose.getFloor());
        }
        return null;
    }

    static FusedPose applyAbsoluteFixForStep1(
            @NonNull ParticleFilterEngine particleFilterEngine,
            @NonNull CoordinateConverter coordinateConverter,
            boolean pfInitialized,
            double latitudeDeg,
            double longitudeDeg,
            @Nullable Integer initializationFloor,
            @Nullable Integer floorPrior,
            long timestampMs,
            float accuracyMeters,
            float headingRad
    ) {
        double[] localFix = coordinateConverter.toLocalMeters(latitudeDeg, longitudeDeg);
        if (!pfInitialized) {
            if (initializationFloor == null) {
                return particleFilterEngine.estimatePose();
            }
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

    @Nullable
    static FusedPose clampCandidateToLegalSegmentPrefix(
            @NonNull FusedPose previousPose,
            @NonNull FusedPose candidatePose,
            @NonNull CoordinateConverter coordinateConverter
    ) {
        if (previousPose.getFloor() != candidatePose.getFloor()) {
            return null;
        }
        double rawProgressMeters = Math.hypot(
                candidatePose.getX() - previousPose.getX(),
                candidatePose.getY() - previousPose.getY()
        );
        if (rawProgressMeters < MIN_CONSTRAINED_POSE_PROGRESS_M) {
            return null;
        }

        LatLng previousLatLng = coordinateConverter.toLatLng(previousPose.getX(), previousPose.getY());
        if (previousLatLng == null) {
            return null;
        }

        double low = 0.0;
        double high = 1.0;
        FusedPose bestPose = null;
        for (int i = 0; i < CONSTRAINED_POSE_RECOVERY_ITERATIONS; i++) {
            double fraction = (low + high) * 0.5;
            FusedPose probePose = interpolatePose(previousPose, candidatePose, fraction);
            LatLng probeLatLng = coordinateConverter.toLatLng(probePose.getX(), probePose.getY());
            boolean isLegalPrefix = probeLatLng != null
                    && MapConstraintRepository.isPathLegal(
                    previousLatLng,
                    probeLatLng,
                    previousPose.getFloor(),
                    probePose.getFloor()
            );
            if (isLegalPrefix) {
                bestPose = probePose;
                low = fraction;
            } else {
                high = fraction;
            }
        }
        if (bestPose == null) {
            return null;
        }
        double recoveredProgressMeters = Math.hypot(
                bestPose.getX() - previousPose.getX(),
                bestPose.getY() - previousPose.getY()
        );
        return recoveredProgressMeters >= MIN_CONSTRAINED_POSE_PROGRESS_M ? bestPose : null;
    }

    @NonNull
    private static FusedPose interpolatePose(
            @NonNull FusedPose previousPose,
            @NonNull FusedPose candidatePose,
            double fraction
    ) {
        double clampedFraction = Math.max(0.0, Math.min(1.0, fraction));
        return new FusedPose(
                previousPose.getX() + ((candidatePose.getX() - previousPose.getX()) * clampedFraction),
                previousPose.getY() + ((candidatePose.getY() - previousPose.getY()) * clampedFraction),
                candidatePose.getFloor(),
                candidatePose.getConfidence(),
                candidatePose.getTimestampMs()
        );
    }

    private float getCurrentHeadingRad() {
        if (displayOrientation != null
                && displayOrientation.length > 0
                && !Float.isNaN(displayOrientation[0])) {
            lastPredictHeadingSource = "device_display";
            return displayOrientation[0];
        }
        lastPredictHeadingSource = "device_raw";
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
     * of those zones. Fall back only when floor-transition constraints are unavailable or incomplete.
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
        if (shouldFailOpenBarometerFloorTransition(previousFloor, newFloor)) {
            return true;
        }
        boolean hasAnyMapConstraints = MapConstraintRepository.hasAnyConstraints();
        if (!hasAnyMapConstraints) {
            return false;
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
        boolean liftAvailable = MapConstraintRepository.hasLiftConstraints(previousFloor)
                || MapConstraintRepository.hasLiftConstraints(newFloor);
        boolean stairsAvailable = MapConstraintRepository.hasStairsConstraints(previousFloor)
                || MapConstraintRepository.hasStairsConstraints(newFloor);
        TransitionPreference transitionPreference = resolveTransitionPreference(
                getElevator(),
                Math.hypot(predictedXMeters - previousXMeters, predictedYMeters - previousYMeters),
                liftAvailable,
                stairsAvailable
        );
        boolean strongBarometerEvidence = hasRecentStrongBarometerFloorTransitionEvidence(
                previousFloor,
                newFloor
        );
        return isTransitionSatisfied(
                transitionPreference,
                previousLatLng,
                predictedLatLng,
                previousFloor,
                newFloor,
                strongBarometerEvidence
        );
    }

    static TransitionPreference resolveTransitionPreference(
            boolean elevatorSignal,
            double horizontalDistanceMeters,
            boolean liftAvailable,
            boolean stairsAvailable
    ) {
        if (liftAvailable && !stairsAvailable) {
            return TransitionPreference.LIFT_ONLY;
        }
        if (stairsAvailable && !liftAvailable) {
            return TransitionPreference.STAIRS_ONLY;
        }
        if (!liftAvailable && !stairsAvailable) {
            return TransitionPreference.ANY_AVAILABLE;
        }
        if (elevatorSignal && horizontalDistanceMeters <= MAX_LIFT_HORIZONTAL_TRAVEL_M) {
            return TransitionPreference.LIFT_ONLY;
        }
        if (!elevatorSignal && horizontalDistanceMeters >= MIN_STAIRS_HORIZONTAL_TRAVEL_M) {
            return TransitionPreference.STAIRS_ONLY;
        }
        // When motion evidence is ambiguous, keep the floor change constrained to known
        // transition zones but do not pretend we can confidently distinguish lift vs stairs.
        return TransitionPreference.ANY_AVAILABLE;
    }

    private boolean isTransitionSatisfied(
            @NonNull TransitionPreference transitionPreference,
            @NonNull LatLng previousLatLng,
            @NonNull LatLng nextLatLng,
            int previousFloor,
            int newFloor,
            boolean strongBarometerEvidence
    ) {
        boolean liftSatisfied = isLiftTransitionSatisfied(previousLatLng, nextLatLng, previousFloor, newFloor);
        boolean stairsSatisfied = isStairTransitionSatisfied(previousLatLng, nextLatLng, previousFloor, newFloor);
        boolean liftSatisfiedByTolerance = false;
        boolean stairsSatisfiedByTolerance = false;
        if (strongBarometerEvidence) {
            if (!liftSatisfied) {
                liftSatisfiedByTolerance = isLiftTransitionSatisfiedWithTolerance(
                        previousLatLng,
                        nextLatLng,
                        previousFloor,
                        newFloor,
                        STRONG_BAROMETER_TRANSITION_TOLERANCE_M
                );
                liftSatisfied = liftSatisfiedByTolerance;
            }
            if (!stairsSatisfied) {
                stairsSatisfiedByTolerance = isStairTransitionSatisfiedWithTolerance(
                        previousLatLng,
                        nextLatLng,
                        previousFloor,
                        newFloor,
                        STRONG_BAROMETER_TRANSITION_TOLERANCE_M
                );
                stairsSatisfied = stairsSatisfiedByTolerance;
            }
        }
        if ((liftSatisfiedByTolerance || stairsSatisfiedByTolerance) && appContext != null) {
            Log.i(
                    "SensorFusion",
                    "Allowing strong-barometer floor transition via tolerance prevFloor="
                            + previousFloor
                            + " newFloor="
                            + newFloor
                            + " toleranceM="
                            + STRONG_BAROMETER_TRANSITION_TOLERANCE_M
                            + " liftTolerance="
                            + liftSatisfiedByTolerance
                            + " stairsTolerance="
                            + stairsSatisfiedByTolerance
            );
        }
        switch (transitionPreference) {
            case LIFT_ONLY:
                return liftSatisfied;
            case STAIRS_ONLY:
                return stairsSatisfied;
            case ANY_AVAILABLE:
            default:
                return liftSatisfied || stairsSatisfied;
        }
    }

    private boolean isLiftTransitionSatisfied(
            @NonNull LatLng previousLatLng,
            @NonNull LatLng nextLatLng,
            int previousFloor,
            int newFloor
    ) {
        return MapConstraintRepository.isPointInsideLift(previousLatLng, previousFloor)
                || MapConstraintRepository.isPointInsideLift(nextLatLng, previousFloor)
                || MapConstraintRepository.isPointInsideLift(nextLatLng, newFloor)
                || MapConstraintRepository.doesPathIntersectLift(previousLatLng, nextLatLng, previousFloor)
                || MapConstraintRepository.doesPathIntersectLift(previousLatLng, nextLatLng, newFloor);
    }

    private boolean isLiftTransitionSatisfiedWithTolerance(
            @NonNull LatLng previousLatLng,
            @NonNull LatLng nextLatLng,
            int previousFloor,
            int newFloor,
            double toleranceMeters
    ) {
        return MapConstraintRepository.isPointInsideOrNearLift(previousLatLng, previousFloor, toleranceMeters)
                || MapConstraintRepository.isPointInsideOrNearLift(previousLatLng, newFloor, toleranceMeters)
                || MapConstraintRepository.isPointInsideOrNearLift(nextLatLng, previousFloor, toleranceMeters)
                || MapConstraintRepository.isPointInsideOrNearLift(nextLatLng, newFloor, toleranceMeters);
    }

    private boolean isStairTransitionSatisfied(
            @NonNull LatLng previousLatLng,
            @NonNull LatLng nextLatLng,
            int previousFloor,
            int newFloor
    ) {
        return MapConstraintRepository.isPointInsideStairs(previousLatLng, previousFloor)
                || MapConstraintRepository.isPointInsideStairs(nextLatLng, previousFloor)
                || MapConstraintRepository.isPointInsideStairs(nextLatLng, newFloor)
                || MapConstraintRepository.doesPathIntersectStairs(previousLatLng, nextLatLng, previousFloor)
                || MapConstraintRepository.doesPathIntersectStairs(previousLatLng, nextLatLng, newFloor);
    }

    private boolean isStairTransitionSatisfiedWithTolerance(
            @NonNull LatLng previousLatLng,
            @NonNull LatLng nextLatLng,
            int previousFloor,
            int newFloor,
            double toleranceMeters
    ) {
        return MapConstraintRepository.isPointInsideOrNearStairs(previousLatLng, previousFloor, toleranceMeters)
                || MapConstraintRepository.isPointInsideOrNearStairs(previousLatLng, newFloor, toleranceMeters)
                || MapConstraintRepository.isPointInsideOrNearStairs(nextLatLng, previousFloor, toleranceMeters)
                || MapConstraintRepository.isPointInsideOrNearStairs(nextLatLng, newFloor, toleranceMeters);
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
        if (MapConstraintRepository.hasVenueOutline()
                && MapConstraintRepository.doesPathExitVenueOutline(previousLatLng, predictedLatLng)) {
            return false;
        }
        if (hasBlockingWallOnMotionPath(previousLatLng, predictedLatLng, previousFloor, predictedFloor)) {
            return false;
        }
        if (previousFloor != predictedFloor
                && shouldFailOpenBarometerFloorTransition(previousFloor, predictedFloor)) {
            return true;
        }
        return true;
    }

    static boolean hasBlockingWallOnMotionPath(
            @NonNull LatLng previousLatLng,
            @NonNull LatLng predictedLatLng,
            int previousFloor,
            int predictedFloor
    ) {
        boolean crossesPredictedFloorWall = MapConstraintRepository.hasWallConstraints(predictedFloor)
                && MapConstraintRepository.doesPathIntersectWall(
                previousLatLng,
                predictedLatLng,
                predictedFloor
        );
        if (crossesPredictedFloorWall) {
            return true;
        }
        return previousFloor != predictedFloor
                && MapConstraintRepository.hasWallConstraints(previousFloor)
                && MapConstraintRepository.doesPathIntersectWall(
                previousLatLng,
                predictedLatLng,
                previousFloor
        );
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
        String sanitizedVenueId = sanitiseVenueTagStatic(venueId);
        boolean venueChanged = !Objects.equals(this.collectionVenue, sanitizedVenueId);
        String previousVenueId = this.collectionVenue;
        this.collectionVenue = sanitizedVenueId;
        if (venueChanged) {
            safeDebugLog(
                    SENSOR_FLOOR_DIAG_TAG,
                    "event=venue_switch venue=" + sanitizedVenueId
                            + " previousVenue=" + previousVenueId
                            + " floor=" + getAbsoluteCurrentFloor()
                            + " trusted=" + isFloorOffsetInitialized
                            + " floorOffsetInitialized=" + isFloorOffsetInitialized
                            + " pdrFloorOffset=" + pdrFloorOffset
                            + " reason=venue_changed source=fallback"
            );
            invalidateVenueScopedFloorState();
        }
        resetWifiFloorBootstrapConsensus();
        clampCumulativeFloorTrackerToVenueBounds();
        clampLatestFusedPoseFloorToVenueIfNeeded(System.currentTimeMillis());
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

    private void invalidateVenueScopedFloorState() {
        int previousFloor = getAbsoluteCurrentFloor();
        boolean wasFloorInitialized = isFloorOffsetInitialized;
        pdrFloorOffset = 0;
        isFloorOffsetInitialized = false;
        blockHistoricalPoseFloorSeedUntilTrustedFix = true;
        resetFloorOnlyResyncWindow();
        resetStableAbsoluteFloorConsensus();
        resetWifiFloorBootstrapConsensus();
        clearProvisionalFloorBootstrapState();
        absoluteFloorTransitionResolver.reset();
        clearPendingElevatorSuppressionEscapeFloor();
        clearPostLiftDestinationFixWindow("inactive");
        committedDisplayFloorAbsolute = Integer.MIN_VALUE;
        pendingCommittedDisplayFloorAbsolute = Integer.MIN_VALUE;
        floorSwitchPending = false;
        lastFloorSwitchBlockReason = "none";
        lastFloorConsensus = "reset";
        lastFloorSource = "relative_only";
        lastFloorAnchorState = "unresolved";
        safeDebugLog(
                SENSOR_FLOOR_DIAG_TAG,
                "event=floor_calibration_invalidated venue=" + collectionVenue
                        + " floor=" + previousFloor
                        + " trustedBefore=" + wasFloorInitialized
                        + " floorOffsetInitialized=" + isFloorOffsetInitialized
                        + " pdrFloorOffset=" + pdrFloorOffset
                        + " reason=venue_scoped_state_invalidated source=fallback"
        );
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

    public long getDisplayOrientationTimestampMs() {
        return lastDisplayOrientationTimestampMs;
    }

    // Derive a map-facing heading that tracks horizontal turning while remaining stable under pitch/roll.
    private void updateDisplayOrientation(@NonNull float[] rotationMatrix) {
        System.arraycopy(this.orientation, 0, this.displayOrientation, 0, this.orientation.length);
        float displayHeadingRad = extractDisplayHeadingRad(rotationMatrix, getDisplayRotation());
        if (!Float.isNaN(displayHeadingRad)) {
            this.displayOrientation[0] = toTrueNorthHeadingRad(displayHeadingRad);
        }
    }

    static float extractDisplayHeadingRad(@NonNull float[] rotationMatrix, int displayRotation) {
        if (rotationMatrix.length < 9) {
            return Float.NaN;
        }

        float[] screenTopAxis = resolveScreenTopAxis(displayRotation);
        float[] screenTopWorld = projectDeviceAxisToWorld(
                rotationMatrix,
                screenTopAxis[0],
                screenTopAxis[1],
                screenTopAxis[2]
        );
        float[] backOfDeviceWorld = projectDeviceAxisToWorld(rotationMatrix, 0f, 0f, -1f);

        float[] headingVector = horizontalMagnitudeSquared(backOfDeviceWorld)
                >= horizontalMagnitudeSquared(screenTopWorld)
                ? backOfDeviceWorld
                : screenTopWorld;

        double headingRad = Math.atan2(headingVector[0], headingVector[1]);
        if (!Double.isFinite(headingRad)) {
            return Float.NaN;
        }
        if (headingRad < 0.0) {
            headingRad += Math.PI * 2.0;
        }
        return (float) headingRad;
    }

    @NonNull
    private static float[] resolveScreenTopAxis(int displayRotation) {
        switch (displayRotation) {
            case Surface.ROTATION_90:
                return new float[]{-1f, 0f, 0f};
            case Surface.ROTATION_180:
                return new float[]{0f, -1f, 0f};
            case Surface.ROTATION_270:
                return new float[]{1f, 0f, 0f};
            case Surface.ROTATION_0:
            default:
                return new float[]{0f, 1f, 0f};
        }
    }

    @NonNull
    private static float[] projectDeviceAxisToWorld(
            @NonNull float[] rotationMatrix,
            float axisX,
            float axisY,
            float axisZ
    ) {
        return new float[]{
                (rotationMatrix[0] * axisX) + (rotationMatrix[1] * axisY) + (rotationMatrix[2] * axisZ),
                (rotationMatrix[3] * axisX) + (rotationMatrix[4] * axisY) + (rotationMatrix[5] * axisZ),
                (rotationMatrix[6] * axisX) + (rotationMatrix[7] * axisY) + (rotationMatrix[8] * axisZ)
        };
    }

    private static float horizontalMagnitudeSquared(@NonNull float[] worldVector) {
        return (worldVector[0] * worldVector[0]) + (worldVector[1] * worldVector[1]);
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

    static boolean isStationaryFromLinearAccelerationWindow(@Nullable List<Double> magnitudes) {
        if (magnitudes == null || magnitudes.size() < STATIONARY_WINDOW_SIZE) {
            return false;
        }
        int startIndex = Math.max(0, magnitudes.size() - STATIONARY_WINDOW_SIZE);
        double sum = 0.0;
        int sampleCount = 0;
        int spikeCount = 0;
        for (int i = startIndex; i < magnitudes.size(); i++) {
            Double magnitude = magnitudes.get(i);
            if (magnitude == null || !Double.isFinite(magnitude)) {
                continue;
            }
            double absoluteMagnitude = Math.abs(magnitude);
            sum += absoluteMagnitude;
            sampleCount++;
            if (absoluteMagnitude >= STATIONARY_SPIKE_THRESHOLD_MPS2) {
                spikeCount++;
            }
        }
        return isStationaryFromMotionSummary(
                sampleCount,
                sampleCount > 0 ? sum / sampleCount : Double.POSITIVE_INFINITY,
                spikeCount
        );
    }

    static boolean isStationaryFromMotionSummary(int sampleCount, double meanMagnitude, int spikeCount) {
        return sampleCount >= STATIONARY_WINDOW_SIZE
                && Double.isFinite(meanMagnitude)
                && meanMagnitude <= STATIONARY_MEAN_THRESHOLD_MPS2
                && spikeCount <= STATIONARY_ALLOWED_SPIKES;
    }

    static boolean shouldForceMotionResume(
            double absoluteMagnitude,
            double meanMagnitude,
            int spikeCount
    ) {
        return (Double.isFinite(absoluteMagnitude) && absoluteMagnitude >= STATIONARY_MEAN_THRESHOLD_MPS2)
                || (Double.isFinite(meanMagnitude) && meanMagnitude > STATIONARY_MEAN_THRESHOLD_MPS2)
                || spikeCount > STATIONARY_ALLOWED_SPIKES;
    }

    static boolean isMotionResumeWindowActive(long motionResumeWindowUntilMs, long timestampMs) {
        return motionResumeWindowUntilMs != Long.MIN_VALUE
                && timestampMs > 0L
                && timestampMs <= motionResumeWindowUntilMs;
    }

    private void updateStationaryMotionGate(double linearAccelerationMagnitude, long timestampMs) {
        if (!Double.isFinite(linearAccelerationMagnitude)) {
            return;
        }
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        double absoluteMagnitude = Math.abs(linearAccelerationMagnitude);
        lastLinearAccelerationMagnitude = absoluteMagnitude;
        recentLinearAccelerationMagnitudes.addLast(absoluteMagnitude);
        recentLinearAccelerationMagnitudeSum += absoluteMagnitude;
        if (absoluteMagnitude >= STATIONARY_SPIKE_THRESHOLD_MPS2) {
            recentLinearAccelerationSpikeCount++;
        }
        while (recentLinearAccelerationMagnitudes.size() > STATIONARY_WINDOW_SIZE) {
            double removedMagnitude = recentLinearAccelerationMagnitudes.removeFirst();
            recentLinearAccelerationMagnitudeSum -= removedMagnitude;
            if (removedMagnitude >= STATIONARY_SPIKE_THRESHOLD_MPS2) {
                recentLinearAccelerationSpikeCount--;
            }
        }
        lastStationaryWindowSampleCount = recentLinearAccelerationMagnitudes.size();
        lastStationaryWindowSpikeCount = recentLinearAccelerationSpikeCount;
        lastStationaryWindowMeanMagnitude = recentLinearAccelerationMagnitudes.isEmpty()
                ? Double.NaN
                : recentLinearAccelerationMagnitudeSum / recentLinearAccelerationMagnitudes.size();
        if (shouldForceMotionResume(
                absoluteMagnitude,
                lastStationaryWindowMeanMagnitude,
                lastStationaryWindowSpikeCount
        )) {
            activateMotionResumeWindow(
                    safeTimestampMs,
                    "linear_accel_motion",
                    absoluteMagnitude
            );
        }
        if (isMotionResumeWindowActive(motionResumeWindowUntilMs, safeTimestampMs)) {
            isStationary = false;
            return;
        }
        if (recentLinearAccelerationMagnitudes.size() < STATIONARY_WINDOW_SIZE) {
            return;
        }
        boolean previousStationary = isStationary;
        boolean nextStationary = isStationaryFromMotionSummary(
                recentLinearAccelerationMagnitudes.size(),
                recentLinearAccelerationMagnitudeSum / recentLinearAccelerationMagnitudes.size(),
                recentLinearAccelerationSpikeCount
        );
        isStationary = nextStationary;
        if (previousStationary != nextStationary) {
            recordStationaryTransition(
                    previousStationary,
                    nextStationary,
                    "linear_accel_window",
                    absoluteMagnitude
            );
        }
    }

    private void markConfirmedStepAsMovement(long timestampMs) {
        recentLinearAccelerationMagnitudes.clear();
        recentLinearAccelerationMagnitudeSum = 0.0;
        recentLinearAccelerationSpikeCount = 0;
        lastStationaryWindowSampleCount = 0;
        lastStationaryWindowMeanMagnitude = Double.NaN;
        lastStationaryWindowSpikeCount = 0;
        activateMotionResumeWindow(timestampMs, "confirmed_step", lastLinearAccelerationMagnitude);
    }

    private void updateElevatorState(boolean rawElevatorDetected, long timestampMs) {
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        expirePostLiftDestinationFixWindowIfNeeded(safeTimestampMs);
        if (maybePanicExitElevatorFromTimeout(safeTimestampMs)) {
            return;
        }
        boolean wasElevator = elevator;
        boolean strongBarometerMotion = isStrongBarometerElevatorMotionPresent(safeTimestampMs);
        boolean nearLiftContext = isCurrentPositionNearLiftZone(safeTimestampMs);
        boolean passedElevatorGate = hasStrongEnoughElevatorEvidence(
                rawElevatorDetected,
                strongBarometerMotion,
                nearLiftContext,
                safeTimestampMs
        );
        if (passedElevatorGate) {
            lastElevatorGate = "passed_barometer_and_lift_zone";
            if (elevatorPositiveDetectionWindowStartMs == Long.MIN_VALUE) {
                elevatorPositiveDetectionWindowStartMs = safeTimestampMs;
            }
            elevatorPositiveDetectionCount = Math.min(
                    ELEVATOR_ON_CONFIRMATION_SAMPLES,
                    elevatorPositiveDetectionCount + 1
            );
            elevatorNegativeDetectionCount = 0;
            lastElevatorPositiveTimestampMs = safeTimestampMs;
            long sustainedPositiveDurationMs = Math.max(
                    0L,
                    safeTimestampMs - elevatorPositiveDetectionWindowStartMs
            );
            if (!elevator) {
                lastLiftTransferState = "enter_pending";
            }
            if (elevator || shouldConfirmElevatorEntry(
                    elevatorPositiveDetectionCount,
                    sustainedPositiveDurationMs
            )) {
                elevator = true;
            }
            if (strongBarometerMotion || elevator) {
                extendElevatorAbsoluteFixSuppression(safeTimestampMs);
            }
            if (!wasElevator && elevator) {
                beginElevatorFloorSession(safeTimestampMs);
                logInfoSafely(
                        "SensorFusion",
                        "ELEVATOR:enter_suppression"
                                + " ts=" + safeTimestampMs
                                + " raw=" + rawElevatorDetected
                                + " strongBarometer=" + strongBarometerMotion
                                + " nearLift=" + nearLiftContext
                );
            }
            return;
        }

        if ((rawElevatorDetected || strongBarometerMotion) && !elevator) {
            logInfoSafely(
                    "SensorFusion",
                    "ELEVATOR:false_trigger_rejected"
                            + " ts=" + safeTimestampMs
                            + " raw=" + rawElevatorDetected
                            + " strongBarometer=" + strongBarometerMotion
                            + " nearLift=" + nearLiftContext
                            + " weakSuppression=" + isWeakElevatorSuppressionState(safeTimestampMs)
            );
        }

        if (rawElevatorDetected && !nearLiftContext) {
            lastElevatorGate = "blocked_by_not_near_lift";
        }
        elevatorPositiveDetectionCount = 0;
        elevatorPositiveDetectionWindowStartMs = Long.MIN_VALUE;
        elevatorNegativeDetectionCount = Math.min(
                ELEVATOR_OFF_CONFIRMATION_SAMPLES,
                elevatorNegativeDetectionCount + 1
        );
        if (!elevator) {
            return;
        }

        boolean withinHoldWindow = lastElevatorPositiveTimestampMs != Long.MIN_VALUE
                && safeTimestampMs - lastElevatorPositiveTimestampMs <= ELEVATOR_HOLD_MS;
        int requiredOffSamples = !strongBarometerMotion && !nearLiftContext
                ? ELEVATOR_OFF_NO_LIFT_CONFIRMATION_SAMPLES
                : ELEVATOR_OFF_CONFIRMATION_SAMPLES;
        if (withinHoldWindow || elevatorNegativeDetectionCount < requiredOffSamples) {
            return;
        }
        elevator = false;
        if (!strongBarometerMotion && !nearLiftContext) {
            lastElevatorGate = "cleared_by_no_motion_or_no_lift_context";
        }
        finishElevatorFloorSession(safeTimestampMs);
        logInfoSafely(
                "SensorFusion",
                "ELEVATOR:exit_suppression_recovered"
                        + " ts=" + safeTimestampMs
                        + " strongBarometer=" + strongBarometerMotion
                        + " nearLift=" + nearLiftContext
        );
    }

    @Nullable
    private LatLng resolveLiftContextLatLng(long timestampMs) {
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        LatLng contextLatLng = null;
        if (isLiftContextPoseFreshEnough(latestFusedPose, safeTimestampMs)) {
            contextLatLng = getLatLngForFusedPose(latestFusedPose);
        } else if (latestFusedPose != null) {
            logMotionDiagnostic(
                    "lift_context skipping_stale_fused_pose ts="
                            + latestFusedPose.getTimestampMs()
                            + " confidence=" + formatDebugDouble(latestFusedPose.getConfidence())
                            + " weakSuppression=" + isWeakElevatorSuppressionState(safeTimestampMs)
            );
        }
        if (contextLatLng == null && hasRecentWifiLiftContext(safeTimestampMs)) {
            contextLatLng = getLatLngWifiPositioning();
        }
        if (contextLatLng == null) {
            contextLatLng = getCurrentGnssLatLng();
        }
        return contextLatLng;
    }

    private boolean isLiftContextPoseFreshEnough(@Nullable FusedPose pose, long timestampMs) {
        if (pose == null) {
            return false;
        }
        if (!Double.isFinite(pose.getConfidence())
                || pose.getConfidence() < LIFT_CONTEXT_MIN_CONFIDENCE) {
            return false;
        }
        if (pose.getTimestampMs() == Long.MIN_VALUE) {
            return false;
        }
        long ageMs = Math.max(0L, timestampMs - pose.getTimestampMs());
        if (ageMs > LIFT_CONTEXT_FUSED_POSE_MAX_AGE_MS) {
            return false;
        }
        return !isWeakElevatorSuppressionState(timestampMs);
    }

    private boolean hasRecentWifiLiftContext(long timestampMs) {
        return getLatLngWifiPositioning() != null
                && lastWifiScanWallClockMs > 0L
                && Math.max(0L, timestampMs - lastWifiScanWallClockMs)
                <= LIFT_CONTEXT_WIFI_FIX_MAX_AGE_MS;
    }

    @Nullable
    private Integer resolveLiftTransferSourceFloorCandidate(long timestampMs) {
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        if (pendingStableAbsoluteFloorCandidate != null
                && pendingStableAbsoluteFloorCount >= INITIAL_ABSOLUTE_FLOOR_LOCK_REQUIRED_CONFIRMATIONS
                && pendingStableAbsoluteFloorLastTimestampMs != Long.MIN_VALUE
                && safeTimestampMs - pendingStableAbsoluteFloorLastTimestampMs
                <= ABSOLUTE_FLOOR_STABLE_CONSENSUS_MAX_GAP_MS
                && isFloorReadyForConsensus(pendingStableAbsoluteFloorCandidate)) {
            return clampAbsoluteFloorToVenue(pendingStableAbsoluteFloorCandidate);
        }
        if (pendingWifiBootstrapFloor != null
                && pendingWifiBootstrapCount >= WIFI_BOOTSTRAP_REQUIRED_CONFIRMATIONS
                && lastWifiBootstrapCandidateTimestampMs != Long.MIN_VALUE
                && safeTimestampMs - lastWifiBootstrapCandidateTimestampMs
                <= WIFI_BOOTSTRAP_CONSENSUS_WINDOW_MS
                && isFloorReadyForConsensus(pendingWifiBootstrapFloor)) {
            return clampAbsoluteFloorToVenue(pendingWifiBootstrapFloor);
        }
        return null;
    }

    private int resolveElevatorSessionStartAbsoluteFloor(long timestampMs) {
        int currentFloor = getAbsoluteCurrentFloor();
        Integer candidateFloor = resolveLiftTransferSourceFloorCandidate(timestampMs);
        if (!isFloorOffsetInitialized) {
            return candidateFloor != null ? candidateFloor : currentFloor;
        }
        if (candidateFloor != null
                && candidateFloor != currentFloor
                && isLowConfidenceFloorEstimate()
                && !hasConflictingBarometerFloorEvidence(currentFloor, candidateFloor)) {
            return candidateFloor;
        }
        return currentFloor;
    }

    @Nullable
    private LiftAnchor resolveNearestLiftAnchor(int floor, @Nullable LatLng referenceLatLng) {
        List<List<LatLng>> liftPolygons = MapConstraintRepository.getLiftsForFloor(floor);
        if (liftPolygons.isEmpty()) {
            return null;
        }
        CoordinateConverter activeConverter = coordinateConverter;
        if (activeConverter == null && referenceLatLng != null) {
            activeConverter = getOrCreateCoordinateConverter(
                    referenceLatLng.latitude,
                    referenceLatLng.longitude
            );
        }
        if (activeConverter == null) {
            return null;
        }
        LiftAnchor bestAnchor = null;
        double bestDistanceMeters = Double.POSITIVE_INFINITY;
        for (List<LatLng> liftPolygon : liftPolygons) {
            LatLng centroid = resolvePolygonCentroid(liftPolygon);
            if (centroid == null) {
                continue;
            }
            double[] localMeters = activeConverter.toLocalMeters(centroid.latitude, centroid.longitude);
            double distanceMeters = referenceLatLng == null
                    ? 0.0
                    : UtilFunctions.distanceBetweenPoints(referenceLatLng, centroid);
            if (distanceMeters < bestDistanceMeters) {
                bestDistanceMeters = distanceMeters;
                bestAnchor = new LiftAnchor(floor, localMeters[0], localMeters[1], centroid);
            }
        }
        return bestAnchor;
    }

    @Nullable
    private LiftAnchor resolveDestinationLiftAnchor(
            int targetFloor,
            @Nullable LiftAnchor sourceLiftAnchor,
            @Nullable LatLng fallbackReference
    ) {
        LatLng referenceLatLng = sourceLiftAnchor != null ? sourceLiftAnchor.latLng : fallbackReference;
        return resolveNearestLiftAnchor(targetFloor, referenceLatLng);
    }

    @Nullable
    private LatLng resolvePolygonCentroid(@Nullable List<LatLng> polygon) {
        if (polygon == null || polygon.isEmpty()) {
            return null;
        }
        double latitudeSum = 0.0;
        double longitudeSum = 0.0;
        int count = 0;
        for (LatLng point : polygon) {
            if (point == null) {
                continue;
            }
            latitudeSum += point.latitude;
            longitudeSum += point.longitude;
            count++;
        }
        if (count == 0) {
            return null;
        }
        return new LatLng(latitudeSum / count, longitudeSum / count);
    }

    private boolean isCurrentPositionNearLiftZone(long timestampMs) {
        LatLng contextLatLng = resolveLiftContextLatLng(timestampMs);
        if (contextLatLng == null) {
            return false;
        }

        LinkedHashSet<Integer> candidateFloors = new LinkedHashSet<>();
        candidateFloors.add(getAbsoluteCurrentFloor());
        if (isLiftContextPoseFreshEnough(latestFusedPose, timestampMs)) {
            candidateFloors.add(clampAbsoluteFloorToVenue(latestFusedPose.getFloor()));
        }
        Integer activeResyncFloor = getActiveFloorOnlyResyncFloor(timestampMs);
        if (activeResyncFloor != null) {
            candidateFloors.add(clampAbsoluteFloorToVenue(activeResyncFloor));
        }
        Integer liftTransferSourceFloor = resolveLiftTransferSourceFloorCandidate(timestampMs);
        if (liftTransferSourceFloor != null) {
            candidateFloors.add(clampAbsoluteFloorToVenue(liftTransferSourceFloor));
        }
        for (int candidateFloor : candidateFloors) {
            if (MapConstraintRepository.hasLiftConstraints(candidateFloor)
                    && MapConstraintRepository.isPointInsideOrNearLift(
                    contextLatLng,
                    candidateFloor,
                    ELEVATOR_NEAR_LIFT_TOLERANCE_M
            )) {
                return true;
            }
        }
        return false;
    }

    private void beginElevatorFloorSession(long timestampMs) {
        elevatorFloorSessionActive = true;
        lastLiftTransferState = "active";
        extendElevatorAbsoluteFixSuppression(timestampMs);
        clearPostLiftDestinationFixWindow("inactive");
        int startAbsoluteFloor = resolveElevatorSessionStartAbsoluteFloor(timestampMs);
        elevatorFloorSessionStartAbsoluteFloor = startAbsoluteFloor;
        clearWeakElevatorSuppressionState();
        resetConsistentSuppressedWifiFloorStreak();
        clearPendingElevatorSuppressionEscapeFloor();
        LatLng liftContextLatLng = resolveLiftContextLatLng(timestampMs);
        elevatorFloorSessionSourceLiftAnchor = resolveNearestLiftAnchor(
                startAbsoluteFloor,
                liftContextLatLng
        );
        int currentAbsoluteFloor = getAbsoluteCurrentFloor();
        if (!isFloorOffsetInitialized || currentAbsoluteFloor != startAbsoluteFloor) {
            recalibrateAbsoluteFloorBaselineInternal(
                    startAbsoluteFloor,
                    timestampMs,
                    elevatorFloorSessionSourceLiftAnchor,
                    "corrected_floor_anchor"
            );
        } else if (elevatorFloorSessionSourceLiftAnchor != null) {
            reseedPoseToLiftAnchor(elevatorFloorSessionSourceLiftAnchor, timestampMs);
            recordLatestFusedPoseIfNeeded();
        }
        float sessionStartElevation = cumulativeFloorTracker.isInitialized()
                ? cumulativeFloorTracker.getFilteredRelativeElevationMeters()
                : elevation;
        if (!Float.isFinite(sessionStartElevation)) {
            sessionStartElevation = elevation;
        }
        elevatorFloorSessionStartElevationMeters = sessionStartElevation;
        elevatorFloorSessionFilteredElevationMeters = sessionStartElevation;
        elevatorFloorSessionLastUpdateTimestampMs = timestampMs;
        elevatorFloorSessionStartTimestampMs = timestampMs;
        elevatorSessionStepCount = 0;
        elevatorLowMotionStartTimestampMs = Long.MIN_VALUE;
    }

    private void updateElevatorFloorSessionEstimate(long timestampMs) {
        if (!elevatorFloorSessionActive || !Float.isFinite(elevation)) {
            return;
        }
        if (!Float.isFinite(elevatorFloorSessionFilteredElevationMeters)) {
            elevatorFloorSessionFilteredElevationMeters = elevation;
            elevatorFloorSessionLastUpdateTimestampMs = timestampMs;
            return;
        }
        long dtMs = elevatorFloorSessionLastUpdateTimestampMs == Long.MIN_VALUE
                ? 0L
                : Math.max(1L, timestampMs - elevatorFloorSessionLastUpdateTimestampMs);
        float desiredDeltaMeters = elevation - elevatorFloorSessionFilteredElevationMeters;
        float smoothedDeltaMeters = desiredDeltaMeters * ELEVATOR_SESSION_FILTER_ALPHA;
        float maxDeltaMeters = dtMs <= 0L
                ? Math.abs(smoothedDeltaMeters)
                : ELEVATOR_SESSION_MAX_VERTICAL_SPEED_MPS * dtMs / 1000f;
        float boundedDeltaMeters = Math.max(
                -maxDeltaMeters,
                Math.min(maxDeltaMeters, smoothedDeltaMeters)
        );
        elevatorFloorSessionFilteredElevationMeters += boundedDeltaMeters;
        elevatorFloorSessionLastUpdateTimestampMs = timestampMs;
    }

    private void finishElevatorFloorSession(long timestampMs) {
        if (!elevatorFloorSessionActive) {
            return;
        }
        int targetAbsoluteFloor = elevatorFloorSessionStartAbsoluteFloor;
        int elevatorFloorDelta = 0;
        float floorHeightMeters = getActiveFloorHeightMeters();
        if (Float.isFinite(floorHeightMeters) && floorHeightMeters > 0f
                && Float.isFinite(elevatorFloorSessionStartElevationMeters)
                && Float.isFinite(elevatorFloorSessionFilteredElevationMeters)) {
            float netElevationDeltaMeters = elevatorFloorSessionFilteredElevationMeters
                    - elevatorFloorSessionStartElevationMeters;
            elevatorFloorDelta = resolveElevatorSessionFloorDelta(
                    netElevationDeltaMeters,
                    floorHeightMeters
            );
            targetAbsoluteFloor = clampAbsoluteFloorToVenue(
                    elevatorFloorSessionStartAbsoluteFloor + elevatorFloorDelta
            );
            logInfoSafely(
                    "SensorFusion",
                    "Elevator session finished startFloor="
                            + elevatorFloorSessionStartAbsoluteFloor
                            + " targetFloor="
                            + targetAbsoluteFloor
                            + " netDeltaM="
                            + netElevationDeltaMeters
            );
        }
        if (elevatorFloorDelta == 0) {
            lastLiftTransferState = "cleared_aborted";
            markWeakElevatorSuppressionState(timestampMs, "no_floor_delta");
            clearPostLiftDestinationFixWindow("inactive");
            resetElevatorFloorSession();
            return;
        }
        lastLiftTransferState = "target_floor_candidate";
        clearWeakElevatorSuppressionState();
        resetConsistentSuppressedWifiFloorStreak();
        clearPendingElevatorSuppressionEscapeFloor();
        extendElevatorAbsoluteFixSuppression(timestampMs);
        LiftAnchor destinationLiftAnchor = resolveDestinationLiftAnchor(
                targetAbsoluteFloor,
                elevatorFloorSessionSourceLiftAnchor,
                resolveLiftContextLatLng(timestampMs)
        );
        boolean destinationAnchorFound = destinationLiftAnchor != null;
        boolean landingPoseLegal = destinationLiftAnchor != null
                && MapConstraintRepository.isPointLegal(destinationLiftAnchor.latLng, targetAbsoluteFloor);
        String destinationAnchorSource = destinationAnchorFound
                ? "destination_lift_anchor"
                : "none";
        pendingDisplayFloorResetAnchorSource = destinationAnchorSource;
        logFloorSwitchTrace(
                "ELEVATOR_TRANSFER_START",
                targetAbsoluteFloor,
                destinationAnchorSource,
                destinationLiftAnchor == null ? null : destinationLiftAnchor.latLng,
                landingPoseLegal,
                false,
                false,
                destinationAnchorFound ? "awaiting_floor_commit" : "missing_destination_lift_anchor"
        );
        if (!destinationAnchorFound || !landingPoseLegal) {
            lastLiftTransferState = destinationAnchorFound
                    ? "blocked_illegal_destination_landing"
                    : "blocked_missing_destination_lift_anchor";
            lastFloorAnchorState = destinationAnchorFound
                    ? "blocked_illegal_destination_landing"
                    : "blocked_missing_destination_lift_anchor";
            lastPostLiftState = "landing_blocked";
            clearPendingDisplayFloorResetToken();
            floorSwitchPending = false;
            pendingCommittedDisplayFloorAbsolute = Integer.MIN_VALUE;
            lastFloorSwitchBlockReason = destinationAnchorFound
                    ? "illegal_destination_landing"
                    : "missing_destination_lift_anchor";
            logFloorSwitchTrace(
                    "FLOOR_SWITCH_COMMITTED=false",
                    targetAbsoluteFloor,
                    destinationAnchorSource,
                    destinationLiftAnchor == null ? null : destinationLiftAnchor.latLng,
                    landingPoseLegal,
                    false,
                    false,
                    lastFloorSwitchBlockReason
            );
            clearPostLiftDestinationFixWindow("landing_blocked");
            resetElevatorFloorSession();
            return;
        }
        recalibrateAbsoluteFloorBaselineInternal(
                targetAbsoluteFloor,
                timestampMs,
                destinationLiftAnchor,
                "resolved_from_lift_transfer"
        );
        lastLiftTransferState = "reseeded_destination_lift";
        armPostLiftDestinationFixWindow(targetAbsoluteFloor, timestampMs);
        resetElevatorFloorSession();
    }

    private void armPostLiftDestinationFixWindow(int absoluteFloor, long timestampMs) {
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        postLiftExpectedAbsoluteFloor = clampAbsoluteFloorToVenue(absoluteFloor);
        postLiftDestinationFixWindowUntilMs = safeTimestampMs + POST_LIFT_DESTINATION_FIX_WINDOW_MS;
        lastPostLiftState = "awaiting_destination_fix";
    }

    private void clearPostLiftDestinationFixWindow(@NonNull String state) {
        postLiftExpectedAbsoluteFloor = Integer.MIN_VALUE;
        postLiftDestinationFixWindowUntilMs = Long.MIN_VALUE;
        lastPostLiftState = state;
    }

    private void expirePostLiftDestinationFixWindowIfNeeded(long timestampMs) {
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        if (postLiftExpectedAbsoluteFloor == Integer.MIN_VALUE
                || postLiftDestinationFixWindowUntilMs == Long.MIN_VALUE
                || safeTimestampMs <= postLiftDestinationFixWindowUntilMs) {
            return;
        }
        clearPostLiftDestinationFixWindow("stuck_guard_triggered");
    }

    private boolean isPostLiftDestinationFixWindowActive(long timestampMs) {
        expirePostLiftDestinationFixWindowIfNeeded(timestampMs);
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        return postLiftExpectedAbsoluteFloor != Integer.MIN_VALUE
                && postLiftDestinationFixWindowUntilMs != Long.MIN_VALUE
                && safeTimestampMs <= postLiftDestinationFixWindowUntilMs;
    }

    private void reseedPoseToLiftAnchor(@NonNull LiftAnchor liftAnchor, long timestampMs) {
        if (particleFilterEngine == null) {
            particleFilterEngine = createParticleFilterEngine();
        }
        if (particleFilterEngine != null) {
            particleFilterEngine.initialize(
                    liftAnchor.xMeters,
                    liftAnchor.yMeters,
                    liftAnchor.floor,
                    timestampMs,
                    getCurrentHeadingRad(),
                    MIN_ABSOLUTE_FIX_START_STD_M
            );
            latestFusedPose = particleFilterEngine.estimatePose();
            pfInitialized = latestFusedPose != null;
            particleCloudTrustedUnderConstraints = latestFusedPose != null;
        }
        if (latestFusedPose == null) {
            latestFusedPose = new FusedPose(
                    liftAnchor.xMeters,
                    liftAnchor.yMeters,
                    liftAnchor.floor,
                    1.0,
                    timestampMs
            );
            pfInitialized = false;
            particleCloudTrustedUnderConstraints = false;
        }
    }

    static boolean shouldForceExitElevatorFromSteps(int elevatorStepCount) {
        return elevatorStepCount > ELEVATOR_PANIC_EXIT_STEP_THRESHOLD;
    }

    static boolean shouldConfirmElevatorEntry(
            int positiveDetectionCount,
            long sustainedPositiveDurationMs
    ) {
        return positiveDetectionCount >= ELEVATOR_ON_CONFIRMATION_SAMPLES
                && sustainedPositiveDurationMs >= ELEVATOR_ON_CONFIRMATION_WINDOW_MS;
    }

    static boolean shouldForceExitElevatorFromBarometerStillness(
            float verticalSpeedMps,
            long lowMotionDurationMs
    ) {
        return Float.isFinite(verticalSpeedMps)
                && verticalSpeedMps < ELEVATOR_PANIC_EXIT_STATIONARY_SPEED_THRESHOLD_MPS
                && lowMotionDurationMs >= ELEVATOR_PANIC_EXIT_STATIONARY_WINDOW_MS;
    }

    static boolean shouldForceExitElevatorFromTimeout(long sessionDurationMs) {
        return sessionDurationMs >= ELEVATOR_SESSION_TIMEOUT_MS;
    }

    private boolean hasStrongEnoughElevatorEvidence(
            boolean rawElevatorDetected,
            boolean strongBarometerMotion,
            boolean nearLiftContext,
            long timestampMs
    ) {
        if (!rawElevatorDetected || !strongBarometerMotion || !nearLiftContext) {
            return false;
        }
        if (!elevator && isWeakElevatorSuppressionState(timestampMs)) {
            lastElevatorGate = "blocked_by_recent_weak_exit";
            return false;
        }
        return true;
    }

    private void markWeakElevatorSuppressionState(long timestampMs, @NonNull String reason) {
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        weakElevatorSuppressionUntilMs = safeTimestampMs + WEAK_ELEVATOR_SUPPRESSION_WINDOW_MS;
        resetConsistentSuppressedWifiFloorStreak();
        clearPendingElevatorSuppressionEscapeFloor();
        logMotionDiagnostic(
                "elevator weak_suppression armed reason=" + reason
                        + " untilMs=" + weakElevatorSuppressionUntilMs
        );
    }

    private void clearWeakElevatorSuppressionState() {
        weakElevatorSuppressionUntilMs = Long.MIN_VALUE;
    }

    private boolean isWeakElevatorSuppressionState(long timestampMs) {
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        return weakElevatorSuppressionUntilMs != Long.MIN_VALUE
                && safeTimestampMs <= weakElevatorSuppressionUntilMs;
    }

    private void resetConsistentSuppressedWifiFloorStreak() {
        consistentSuppressedWifiFloorCandidate = null;
        consistentSuppressedWifiFloorCount = 0;
        lastConsistentSuppressedWifiFloorTimestampMs = Long.MIN_VALUE;
    }

    private void noteConsistentSuppressedWifiFloor(int floor, long timestampMs) {
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        if (consistentSuppressedWifiFloorCandidate != null
                && consistentSuppressedWifiFloorCandidate == floor
                && lastConsistentSuppressedWifiFloorTimestampMs != Long.MIN_VALUE
                && safeTimestampMs - lastConsistentSuppressedWifiFloorTimestampMs
                <= CONSISTENT_SUPPRESSED_WIFI_FLOOR_MAX_GAP_MS) {
            consistentSuppressedWifiFloorCount++;
        } else {
            consistentSuppressedWifiFloorCandidate = floor;
            consistentSuppressedWifiFloorCount = 1;
        }
        lastConsistentSuppressedWifiFloorTimestampMs = safeTimestampMs;
    }

    private boolean shouldAllowConsistentSuppressedWifiFloorEscape(int floor, long timestampMs) {
        if (!isWeakElevatorSuppressionState(timestampMs)) {
            return false;
        }
        if (elevator || elevatorFloorSessionActive || isStrongBarometerElevatorMotionPresent(timestampMs)) {
            return false;
        }
        noteConsistentSuppressedWifiFloor(floor, timestampMs);
        return consistentSuppressedWifiFloorCandidate != null
                && consistentSuppressedWifiFloorCandidate == floor
                && consistentSuppressedWifiFloorCount
                >= CONSISTENT_SUPPRESSED_WIFI_FLOOR_REQUIRED_CONFIRMATIONS
                && floor != getAbsoluteCurrentFloor();
    }

    private void armPendingElevatorSuppressionEscapeFloor(int floor, long timestampMs) {
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        pendingElevatorSuppressionEscapeFloor = clampAbsoluteFloorToVenue(floor);
        pendingElevatorSuppressionEscapeUntilMs =
                safeTimestampMs + PENDING_ELEVATOR_SUPPRESSION_ESCAPE_WINDOW_MS;
        logMotionDiagnostic(
                "elevator escape_floor armed floor=" + pendingElevatorSuppressionEscapeFloor
                        + " count=" + consistentSuppressedWifiFloorCount
        );
    }

    private boolean consumePendingElevatorSuppressionEscapeFloor(int floor, long timestampMs) {
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        if (pendingElevatorSuppressionEscapeFloor == null
                || pendingElevatorSuppressionEscapeUntilMs == Long.MIN_VALUE
                || safeTimestampMs > pendingElevatorSuppressionEscapeUntilMs
                || pendingElevatorSuppressionEscapeFloor != floor) {
            return false;
        }
        clearPendingElevatorSuppressionEscapeFloor();
        clearWeakElevatorSuppressionState();
        resetConsistentSuppressedWifiFloorStreak();
        return true;
    }

    private void clearPendingElevatorSuppressionEscapeFloor() {
        pendingElevatorSuppressionEscapeFloor = null;
        pendingElevatorSuppressionEscapeUntilMs = Long.MIN_VALUE;
    }

    static int resolveElevatorSessionFloorDelta(float netElevationDeltaMeters, float floorHeightMeters) {
        if (!Float.isFinite(netElevationDeltaMeters)
                || !Float.isFinite(floorHeightMeters)
                || floorHeightMeters <= 0f) {
            return 0;
        }
        float absoluteDeltaMeters = Math.abs(netElevationDeltaMeters);
        float minimumFloorChangeMeters = CumulativeFloorTracker.requiredTravelForFloorChange(
                floorHeightMeters
        );
        if (absoluteDeltaMeters < minimumFloorChangeMeters) {
            return 0;
        }
        float maxSnapBonusMeters = Math.max(0f, floorHeightMeters - minimumFloorChangeMeters);
        float snapBonusMeters = Math.min(
                floorHeightMeters * ELEVATOR_SESSION_FLOOR_SNAP_BONUS_RATIO,
                maxSnapBonusMeters
        );
        int floorDelta = (int) Math.floor((absoluteDeltaMeters + snapBonusMeters) / floorHeightMeters);
        return floorDelta * (netElevationDeltaMeters > 0f ? 1 : -1);
    }

    private void resetElevatorFloorSession() {
        elevatorFloorSessionActive = false;
        elevatorFloorSessionStartAbsoluteFloor = 0;
        elevatorFloorSessionStartElevationMeters = Float.NaN;
        elevatorFloorSessionFilteredElevationMeters = Float.NaN;
        elevatorFloorSessionLastUpdateTimestampMs = Long.MIN_VALUE;
        elevatorFloorSessionStartTimestampMs = Long.MIN_VALUE;
        elevatorSessionStepCount = 0;
        elevatorLowMotionStartTimestampMs = Long.MIN_VALUE;
        elevatorFloorSessionSourceLiftAnchor = null;
    }

    private void updateElevatorBarometerSignal(long timestampMs) {
        if (!Float.isFinite(elevation)) {
            return;
        }
        if (!Float.isFinite(lastElevatorBarometerElevationMeters)
                || lastElevatorBarometerTimestampMs == Long.MIN_VALUE) {
            lastElevatorBarometerElevationMeters = elevation;
            lastElevatorBarometerTimestampMs = timestampMs;
            recentBarometerVerticalTravelMeters = 0f;
            recentBarometerWindowStartElevationMeters = elevation;
            recentBarometerNetVerticalDisplacementMeters = 0f;
            recentBarometerVerticalWindowStartMs = timestampMs;
            smoothedBarometerVerticalSpeedMps = 0f;
            return;
        }
        long dtMs = Math.max(1L, timestampMs - lastElevatorBarometerTimestampMs);
        float elevationDeltaMeters = elevation - lastElevatorBarometerElevationMeters;
        float instantaneousSpeedMps = Math.abs(elevationDeltaMeters) * 1000f / dtMs;
        smoothedBarometerVerticalSpeedMps = smoothedBarometerVerticalSpeedMps
                + BAROMETER_ELEVATOR_SPEED_SMOOTHING_ALPHA
                * (instantaneousSpeedMps - smoothedBarometerVerticalSpeedMps);

        if (recentBarometerVerticalWindowStartMs == Long.MIN_VALUE
                || timestampMs - recentBarometerVerticalWindowStartMs > ELEVATOR_BAROMETER_WINDOW_MS) {
            recentBarometerVerticalWindowStartMs = timestampMs;
            recentBarometerVerticalTravelMeters = 0f;
            recentBarometerWindowStartElevationMeters = lastElevatorBarometerElevationMeters;
            recentBarometerNetVerticalDisplacementMeters = 0f;
        }
        recentBarometerVerticalTravelMeters += Math.abs(elevationDeltaMeters);
        if (!Float.isFinite(recentBarometerWindowStartElevationMeters)) {
            recentBarometerWindowStartElevationMeters = lastElevatorBarometerElevationMeters;
        }
        recentBarometerNetVerticalDisplacementMeters = Float.isFinite(
                recentBarometerWindowStartElevationMeters
        )
                ? Math.abs(elevation - recentBarometerWindowStartElevationMeters)
                : 0f;
        lastElevatorBarometerElevationMeters = elevation;
        lastElevatorBarometerTimestampMs = timestampMs;
        updateElevatorLowMotionWindow(timestampMs);
        maybePanicExitElevatorFromBarometerStillness(timestampMs);
        maybePanicExitElevatorFromTimeout(timestampMs);
    }

    private boolean isStrongBarometerElevatorMotionPresent(long timestampMs) {
        if (lastElevatorBarometerTimestampMs == Long.MIN_VALUE
                || timestampMs - lastElevatorBarometerTimestampMs > ELEVATOR_BAROMETER_SIGNAL_MAX_AGE_MS) {
            return false;
        }
        if (hasRecentWalkingStepActivity(timestampMs)) {
            return false;
        }
        boolean hasRecentWindow = recentBarometerVerticalWindowStartMs != Long.MIN_VALUE
                && timestampMs - recentBarometerVerticalWindowStartMs <= ELEVATOR_BAROMETER_WINDOW_MS;
        boolean sufficientlyLongWindow = hasRecentWindow
                && timestampMs - recentBarometerVerticalWindowStartMs >= ELEVATOR_BAROMETER_MIN_WINDOW_MS;
        boolean verticalSpeedDetected = smoothedBarometerVerticalSpeedMps
                >= ELEVATOR_BAROMETER_VERTICAL_SPEED_THRESHOLD_MPS;
        boolean netVerticalDisplacementDetected = sufficientlyLongWindow
                && recentBarometerNetVerticalDisplacementMeters
                >= ELEVATOR_BAROMETER_NET_VERTICAL_DISPLACEMENT_THRESHOLD_M;
        return hasRecentWindow && verticalSpeedDetected && netVerticalDisplacementDetected;
    }

    private boolean isBarometerSuggestingElevator(long timestampMs) {
        if (lastElevatorBarometerTimestampMs == Long.MIN_VALUE
                || timestampMs - lastElevatorBarometerTimestampMs > ELEVATOR_BAROMETER_SIGNAL_MAX_AGE_MS) {
            return false;
        }
        if (hasRecentWalkingStepActivity(timestampMs)) {
            return false;
        }
        if (isStrongBarometerElevatorMotionPresent(timestampMs)) {
            return true;
        }
        boolean hasRecentWindow = recentBarometerVerticalWindowStartMs != Long.MIN_VALUE
                && timestampMs - recentBarometerVerticalWindowStartMs <= ELEVATOR_BAROMETER_WINDOW_MS;
        boolean sufficientlyLongWindow = hasRecentWindow
                && timestampMs - recentBarometerVerticalWindowStartMs >= ELEVATOR_BAROMETER_MIN_WINDOW_MS;
        boolean verticalSpeedDetected = smoothedBarometerVerticalSpeedMps
                >= ELEVATOR_BAROMETER_VERTICAL_SPEED_THRESHOLD_MPS;
        boolean verticalTravelDetected = sufficientlyLongWindow
                && recentBarometerVerticalTravelMeters >= ELEVATOR_BAROMETER_VERTICAL_TRAVEL_THRESHOLD_M;
        boolean netVerticalDisplacementDetected = sufficientlyLongWindow
                && recentBarometerNetVerticalDisplacementMeters
                >= ELEVATOR_BAROMETER_NET_VERTICAL_DISPLACEMENT_THRESHOLD_M;
        return isStationary
                && hasRecentWindow
                && netVerticalDisplacementDetected
                && (verticalSpeedDetected || verticalTravelDetected);
    }

    private boolean hasRecentWalkingStepActivity(long timestampMs) {
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        pruneRecentAcceptedStepTimestamps(safeTimestampMs);
        long sinceLastAcceptedStepMs = lastAcceptedStepTimestampMs == Long.MIN_VALUE
                ? Long.MAX_VALUE
                : Math.max(0L, safeTimestampMs - lastAcceptedStepTimestampMs);
        return shouldSuppressElevatorForRecentSteps(
                recentAcceptedStepTimestampsMs.size(),
                sinceLastAcceptedStepMs
        );
    }

    static boolean shouldSuppressElevatorForRecentSteps(
            int recentAcceptedStepCount,
            long sinceLastAcceptedStepMs
    ) {
        if (recentAcceptedStepCount >= ELEVATOR_STEP_SUPPRESSION_MIN_STEPS) {
            return true;
        }
        return recentAcceptedStepCount >= 2
                && sinceLastAcceptedStepMs >= 0L
                && sinceLastAcceptedStepMs <= ELEVATOR_STEP_SUPPRESSION_RECENT_STEP_MAX_AGE_MS;
    }

    private void noteAcceptedStepForElevatorSuppression(long timestampMs) {
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        recentAcceptedStepTimestampsMs.addLast(safeTimestampMs);
        pruneRecentAcceptedStepTimestamps(safeTimestampMs);
    }

    private void pruneRecentAcceptedStepTimestamps(long timestampMs) {
        long minTimestampMs = timestampMs - ELEVATOR_STEP_SUPPRESSION_WINDOW_MS;
        while (!recentAcceptedStepTimestampsMs.isEmpty()
                && recentAcceptedStepTimestampsMs.peekFirst() < minTimestampMs) {
            recentAcceptedStepTimestampsMs.removeFirst();
        }
    }

    private void extendElevatorAbsoluteFixSuppression(long timestampMs) {
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        elevatorAbsoluteFixRejectUntilMs = Math.max(
                elevatorAbsoluteFixRejectUntilMs,
                safeTimestampMs + ELEVATOR_ABSOLUTE_FIX_COOLDOWN_MS
        );
    }

    private boolean isElevatorAbsoluteFixSuppressionActive(long timestampMs) {
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        return elevator
                || elevatorFloorSessionActive
                || isStrongBarometerElevatorMotionPresent(safeTimestampMs)
                || safeTimestampMs <= elevatorAbsoluteFixRejectUntilMs;
    }

    private boolean shouldRejectAbsoluteFixDuringElevatorSuppression(
            double latitudeDeg,
            double longitudeDeg,
            @NonNull double[] localFix,
            @Nullable Integer floorPrior,
            long timestampMs,
            @NonNull String debugSource,
            boolean bootstrapOverrideElevatorSuppression
    ) {
        if (!"WIFI".equals(debugSource) && !"GNSS".equals(debugSource)) {
            return false;
        }
        expirePostLiftDestinationFixWindowIfNeeded(timestampMs);
        if (!isElevatorAbsoluteFixSuppressionActive(timestampMs)) {
            resetConsistentSuppressedWifiFloorStreak();
            return false;
        }
        if (bootstrapOverrideElevatorSuppression) {
            return false;
        }
        extendElevatorAbsoluteFixSuppression(timestampMs);
        if ("WIFI".equals(debugSource)
                && floorPrior != null
                && shouldAllowConsistentSuppressedWifiFloorEscape(floorPrior, timestampMs)) {
            armPendingElevatorSuppressionEscapeFloor(floorPrior, timestampMs);
            clearWeakElevatorSuppressionState();
            logMotionDiagnostic(
                    "absolute_fix escape_from_false_elevator source=" + debugSource
                            + " floor=" + floorPrior
            );
            return false;
        }
        int validationFloor = floorPrior != null ? floorPrior : getAbsoluteCurrentFloor();
        boolean postLiftBypass = isPostLiftDestinationFixWindowActive(timestampMs)
                && postLiftExpectedAbsoluteFloor == validationFloor;
        if (isPostLiftDestinationFixWindowActive(timestampMs) && !postLiftBypass) {
            lastPostLiftState = "rejected_old_floor_state";
        }
        if (!postLiftBypass && latestFusedPose != null) {
            double distanceMeters = Math.hypot(
                    latestFusedPose.getX() - localFix[0],
                    latestFusedPose.getY() - localFix[1]
            );
            if (distanceMeters > ELEVATOR_ABSOLUTE_FIX_REJECT_DISTANCE_M) {
                logMotionDiagnostic(
                        "absolute_fix suppressed_during_elevator source=" + debugSource
                                + " distanceM=" + formatDebugDouble(distanceMeters)
                                + " thresholdM=" + ELEVATOR_ABSOLUTE_FIX_REJECT_DISTANCE_M
                );
                return true;
            }
        }
        if (hasRenderableMapConstraints()) {
            LatLng absoluteFixLatLng = new LatLng(latitudeDeg, longitudeDeg);
            if (!MapConstraintRepository.isPointLegal(absoluteFixLatLng, validationFloor)) {
                logMotionDiagnostic(
                        "absolute_fix rejected_illegal_during_elevator source=" + debugSource
                                + " floor=" + validationFloor
                                + " latLng=" + formatLatLng(absoluteFixLatLng)
                );
                return true;
            }
        }
        if (postLiftBypass) {
            lastPostLiftState = "accepted_destination_fix";
        }
        return false;
    }

    private boolean shouldAllowBootstrapAbsoluteFix(
            double latitudeDeg,
            double longitudeDeg,
            @Nullable Integer floorPrior,
            long timestampMs,
            long observationAgeMs,
            @NonNull String debugSource
    ) {
        if (pfInitialized || (!"WIFI".equals(debugSource) && !"GNSS".equals(debugSource))) {
            return false;
        }
        if (!isValidLatLng(latitudeDeg, longitudeDeg)) {
            return false;
        }
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        if (observationAgeMs < 0L || observationAgeMs > BOOTSTRAP_ABSOLUTE_FIX_MAX_AGE_MS) {
            return false;
        }
        if (!isElevatorAbsoluteFixSuppressionActive(safeTimestampMs)) {
            return false;
        }
        if (floorPrior == null && resolveStep1InitializationFloor(null) == null) {
            return false;
        }
        LatLng absoluteFixLatLng = new LatLng(latitudeDeg, longitudeDeg);
        if (MapConstraintRepository.hasVenueOutline()
                && !MapConstraintRepository.isPointInsideVenueOutline(absoluteFixLatLng)) {
            return false;
        }
        return true;
    }

    private void resetElevatorState() {
        elevator = false;
        elevatorPositiveDetectionCount = 0;
        elevatorNegativeDetectionCount = 0;
        elevatorPositiveDetectionWindowStartMs = Long.MIN_VALUE;
        lastElevatorPositiveTimestampMs = Long.MIN_VALUE;
        elevatorAbsoluteFixRejectUntilMs = Long.MIN_VALUE;
        lastElevatorBarometerElevationMeters = Float.NaN;
        lastElevatorBarometerTimestampMs = Long.MIN_VALUE;
        smoothedBarometerVerticalSpeedMps = 0f;
        recentBarometerVerticalTravelMeters = 0f;
        recentBarometerWindowStartElevationMeters = Float.NaN;
        recentBarometerNetVerticalDisplacementMeters = 0f;
        recentBarometerVerticalWindowStartMs = Long.MIN_VALUE;
        clearWeakElevatorSuppressionState();
        resetConsistentSuppressedWifiFloorStreak();
        clearPendingElevatorSuppressionEscapeFloor();
        lastElevatorGate = "reset";
        lastLiftTransferState = "inactive";
        clearPostLiftDestinationFixWindow("inactive");
        resetElevatorFloorSession();
    }

    private void noteElevatorStepAndMaybePanicExit(long timestampMs) {
        if (!elevator) {
            return;
        }
        elevatorSessionStepCount++;
        if (!shouldForceExitElevatorFromSteps(elevatorSessionStepCount)) {
            return;
        }
        panicExitElevator(
                timestampMs,
                "step_detector stepCount=" + elevatorSessionStepCount
        );
    }

    private void updateElevatorLowMotionWindow(long timestampMs) {
        if (!elevator) {
            elevatorLowMotionStartTimestampMs = Long.MIN_VALUE;
            return;
        }
        if (smoothedBarometerVerticalSpeedMps < ELEVATOR_PANIC_EXIT_STATIONARY_SPEED_THRESHOLD_MPS) {
            if (elevatorLowMotionStartTimestampMs == Long.MIN_VALUE) {
                elevatorLowMotionStartTimestampMs = timestampMs;
            }
            return;
        }
        elevatorLowMotionStartTimestampMs = Long.MIN_VALUE;
    }

    private boolean maybePanicExitElevatorFromBarometerStillness(long timestampMs) {
        if (!elevator || elevatorLowMotionStartTimestampMs == Long.MIN_VALUE) {
            return false;
        }
        long lowMotionDurationMs = Math.max(0L, timestampMs - elevatorLowMotionStartTimestampMs);
        if (!shouldForceExitElevatorFromBarometerStillness(
                smoothedBarometerVerticalSpeedMps,
                lowMotionDurationMs
        )) {
            return false;
        }
        panicExitElevator(
                timestampMs,
                "barometer_stillness lowMotionMs=" + lowMotionDurationMs
                        + " verticalSpeedMps=" + smoothedBarometerVerticalSpeedMps
        );
        return true;
    }

    private boolean maybePanicExitElevatorFromTimeout(long timestampMs) {
        if (!elevator || elevatorFloorSessionStartTimestampMs == Long.MIN_VALUE) {
            return false;
        }
        long sessionDurationMs = Math.max(0L, timestampMs - elevatorFloorSessionStartTimestampMs);
        if (!shouldForceExitElevatorFromTimeout(sessionDurationMs)) {
            return false;
        }
        panicExitElevator(
                timestampMs,
                "session_timeout durationMs=" + sessionDurationMs
        );
        logInfoSafely(
                "SensorFusion",
                "ELEVATOR:exit_suppression_timeout durationMs=" + sessionDurationMs
        );
        return true;
    }

    private void panicExitElevator(long timestampMs, String reason) {
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        if (!elevator && !elevatorFloorSessionActive) {
            return;
        }
        Log.w(
                "SensorFusion",
                "Elevator panic exit reason=" + reason
                        + " startFloor=" + elevatorFloorSessionStartAbsoluteFloor
                        + " stepCount=" + elevatorSessionStepCount
        );
        elevator = false;
        elevatorPositiveDetectionCount = 0;
        elevatorNegativeDetectionCount = 0;
        elevatorPositiveDetectionWindowStartMs = Long.MIN_VALUE;
        lastElevatorPositiveTimestampMs = Long.MIN_VALUE;
        recentBarometerVerticalTravelMeters = 0f;
        recentBarometerVerticalWindowStartMs = safeTimestampMs;
        recentBarometerWindowStartElevationMeters = Float.isFinite(elevation) ? elevation : Float.NaN;
        recentBarometerNetVerticalDisplacementMeters = 0f;
        smoothedBarometerVerticalSpeedMps = 0f;
        lastElevatorBarometerElevationMeters = Float.isFinite(elevation) ? elevation : Float.NaN;
        lastElevatorBarometerTimestampMs = safeTimestampMs;
        markWeakElevatorSuppressionState(safeTimestampMs, "panic_exit");
        extendElevatorAbsoluteFixSuppression(safeTimestampMs);
        finishElevatorFloorSession(safeTimestampMs);
        elevatorLowMotionStartTimestampMs = Long.MIN_VALUE;
        logInfoSafely("SensorFusion", "ELEVATOR:exit_suppression_recovered reason=" + reason);
    }

    private void resetStationaryMotionGate() {
        recentLinearAccelerationMagnitudes.clear();
        recentLinearAccelerationMagnitudeSum = 0.0;
        recentLinearAccelerationSpikeCount = 0;
        isStationary = true;
        lastStationaryStepLogMs = 0L;
        lastStationaryAbsoluteFixLogMs = 0L;
        lastStationaryWindowSampleCount = 0;
        lastStationaryWindowMeanMagnitude = Double.NaN;
        lastStationaryWindowSpikeCount = 0;
        lastLinearAccelerationMagnitude = Double.NaN;
        motionResumeWindowUntilMs = Long.MIN_VALUE;
        lastStationaryTransitionTimestampMs = Long.MIN_VALUE;
        lastStationaryTransitionReason = "reset";
        lastStepDetectorEventTimestampMs = Long.MIN_VALUE;
        lastAcceptedStepTimestampMs = Long.MIN_VALUE;
        recentAcceptedStepTimestampsMs.clear();
        lastStepDecision = "reset";
        lastAbsoluteFixReceivedTimestampMs = Long.MIN_VALUE;
        lastAcceptedAbsoluteFixTimestampMs = Long.MIN_VALUE;
        lastAbsoluteFixDecision = "reset";
        lastRejectReason = "none";
        lastBlockReason = "none";
        lastPoseAdvanceSource = "none";
        lastPoseAdvanceTimestampMs = Long.MIN_VALUE;
        resetStableAbsoluteFloorConsensus();
        lastFloorConsensus = "reset";
        lastFloorSource = "relative_only";
        lastFloorAnchorState = "unresolved";
        lastPostLiftState = "inactive";
    }

    private void activateMotionResumeWindow(
            long timestampMs,
            @NonNull String reason,
            double triggeringMagnitude
    ) {
        long safeTimestampMs = timestampMs > 0L ? timestampMs : System.currentTimeMillis();
        long previousWindowUntilMs = motionResumeWindowUntilMs;
        motionResumeWindowUntilMs = Math.max(
                motionResumeWindowUntilMs,
                safeTimestampMs + MOTION_RESUME_WINDOW_MS
        );
        boolean previousStationary = isStationary;
        isStationary = false;
        if (previousStationary) {
            recordStationaryTransition(
                    true,
                    false,
                    reason,
                    triggeringMagnitude
            );
        }
        logMotionDiagnostic(
                "motion_resume activated"
                        + " reason=" + reason
                        + " ts=" + safeTimestampMs
                        + " untilMs=" + motionResumeWindowUntilMs
                        + " previousUntilMs=" + previousWindowUntilMs
                        + " triggerMag=" + formatDebugDouble(triggeringMagnitude)
        );
    }

    private void recordStationaryTransition(
            boolean previousStationary,
            boolean nextStationary,
            @NonNull String reason,
            double triggeringMagnitude
    ) {
        lastStationaryTransitionTimestampMs = System.currentTimeMillis();
        lastStationaryTransitionReason = reason;
        logMotionDiagnostic(
                "stationary_transition prev=" + previousStationary
                        + " next=" + nextStationary
                        + " reason=" + reason
                        + " samples=" + lastStationaryWindowSampleCount
                        + " meanAbs=" + formatDebugDouble(lastStationaryWindowMeanMagnitude)
                        + " spikes=" + lastStationaryWindowSpikeCount
                        + " lastMag=" + formatDebugDouble(triggeringMagnitude)
        );
    }

    private void recordStepDecision(
            long timestampMs,
            boolean accepted,
            @NonNull String reason,
            long eventAgeMs,
            long sinceLastStepMs,
            @Nullable PdrDelta pdrDelta
    ) {
        if (accepted) {
            lastAcceptedStepTimestampMs = timestampMs;
            noteAcceptedStepForElevatorSuppression(timestampMs);
        }
        lastStepDecision = reason;
        if (!accepted) {
            lastRejectReason = reason;
            lastBlockReason = reason;
        }
        logMotionDiagnostic(
                "step_event decision=" + reason
                        + " accepted=" + accepted
                        + " ts=" + timestampMs
                        + " ageMs=" + eventAgeMs
                        + " sinceLastStepMs=" + sinceLastStepMs
                        + " stationary=" + isStationary
                        + " samples=" + lastStationaryWindowSampleCount
                        + " meanAbs=" + formatDebugDouble(lastStationaryWindowMeanMagnitude)
                        + " spikes=" + lastStationaryWindowSpikeCount
                        + " stepLenM=" + formatDebugDouble(pdrDelta == null ? Double.NaN : pdrDelta.getStepLengthMeters())
                        + " deltaHeadingRad=" + formatDebugDouble(pdrDelta == null ? Double.NaN : pdrDelta.getDeltaHeadingRad())
                        + " heightDeltaM=" + formatDebugDouble(pdrDelta == null ? Double.NaN : pdrDelta.getHeightDeltaMeters())
                        + " headingSource=" + lastPredictHeadingSource
        );
    }

    private void recordAbsoluteFixReceived(
            @NonNull String source,
            long timestampMs,
            double latitudeDeg,
            double longitudeDeg,
            @Nullable Integer floorPrior,
            float accuracyMeters,
            long observationAgeMs
    ) {
        lastAbsoluteFixReceivedTimestampMs = timestampMs;
        boolean motionResumeActive = isMotionResumeWindowActive(motionResumeWindowUntilMs, timestampMs);
        logMotionDiagnostic(
                "absolute_fix received source=" + source
                        + " ts=" + timestampMs
                        + " lat=" + formatDebugDouble(latitudeDeg)
                        + " lon=" + formatDebugDouble(longitudeDeg)
                        + " floorPrior=" + (floorPrior == null ? "n/a" : floorPrior)
                        + " accuracyM=" + formatDebugDouble(accuracyMeters)
                        + " obsAgeMs=" + observationAgeMs
                        + " stationary=" + isStationary
                        + " motionResumeActive=" + motionResumeActive
                        + " pfInitialized=" + pfInitialized
        );
    }

    private void recordAbsoluteFixDecision(
            @NonNull String source,
            long timestampMs,
            boolean accepted,
            @NonNull String reason
    ) {
        if (accepted) {
            lastAcceptedAbsoluteFixTimestampMs = timestampMs;
            lastBlockReason = "none";
        } else {
            lastRejectReason = source + ":" + reason;
            lastBlockReason = source + ":" + reason;
        }
        lastAbsoluteFixDecision = source + ":" + reason;
        boolean motionResumeActive = isMotionResumeWindowActive(motionResumeWindowUntilMs, timestampMs);
        logMotionDiagnostic(
                "absolute_fix decision source=" + source
                        + " accepted=" + accepted
                        + " reason=" + reason
                        + " ts=" + timestampMs
                        + " stationary=" + isStationary
                        + " motionResumeActive=" + motionResumeActive
                        + " pfInitialized=" + pfInitialized
        );
    }

    private void recordPoseAdvance(@NonNull String source, long timestampMs) {
        lastPoseAdvanceSource = source;
        lastPoseAdvanceTimestampMs = timestampMs;
        lastBlockReason = "none";
        logMotionDiagnostic(
                "pose_advanced INTERNAL_POSE_UPDATED=true"
                        + " source=" + source
                        + " ts=" + timestampMs
                        + " fused=" + formatFusedPose(latestFusedPose)
        );
    }

    private void recordPoseBlocked(@NonNull String reason) {
        lastRejectReason = reason;
        lastBlockReason = reason;
        logMotionDiagnostic("pose_blocked INTERNAL_POSE_UPDATED=false reason=" + reason);
    }

    private void logMotionDiagnostic(@NonNull String message) {
        if (!DEBUG_FUSION_TRACE) {
            return;
        }
        try {
            Log.d(MOTION_DIAG_TAG, message);
        } catch (RuntimeException ignored) {
            // Unit tests may run without an Android Log implementation.
        }
    }

    private void logInfoSafely(@NonNull String tag, @NonNull String message) {
        try {
            Log.i(tag, message);
        } catch (RuntimeException ignored) {
            // Unit tests may run without an Android Log implementation.
        }
    }

    private void logWarnSafely(@NonNull String tag, @NonNull String message) {
        try {
            Log.w(tag, message);
        } catch (RuntimeException ignored) {
            // Unit tests may run without an Android Log implementation.
        }
    }

    private void logStationaryStepSuppression(long currentTimeMs) {
        if (currentTimeMs - lastStationaryStepLogMs < STATIONARY_LOG_INTERVAL_MS) {
            return;
        }
        Log.i("SensorFusion", "Ignoring step event while stationary to suppress fake PDR drift.");
        lastStationaryStepLogMs = currentTimeMs;
    }

    private boolean shouldFreezeAbsoluteFix(@NonNull String debugSource, @Nullable Integer floorPrior) {
        if (!pfInitialized) {
            return false;
        }
        if (!isStationary) {
            return false;
        }
        if (isMotionResumeWindowActive(motionResumeWindowUntilMs, System.currentTimeMillis())) {
            return false;
        }
        if ("WIFI".equals(debugSource)
                && shouldAcceptStationaryWifiFloorBootstrap(
                floorPrior,
                getAbsoluteCurrentFloor(),
                isFloorOffsetInitialized
        )) {
            return false;
        }
        return "GNSS".equals(debugSource) || "WIFI".equals(debugSource);
    }

    static boolean shouldAcceptStationaryWifiFloorBootstrap(
            @Nullable Integer reportedFloor,
            int currentAbsoluteFloor,
            boolean isFloorOffsetInitialized
    ) {
        if (reportedFloor == null) {
            return false;
        }
        // Modified: while stationary, still allow WiFi to bootstrap or correct the absolute floor.
        return !isFloorOffsetInitialized || reportedFloor != currentAbsoluteFloor;
    }

    static boolean shouldForceWifiFloorBootstrap(
            @NonNull String debugSource,
            @Nullable Integer floorPrior,
            boolean pfInitialized,
            boolean isFloorOffsetInitialized,
            int matchingWifiFloorCount,
            float accuracyMeters
    ) {
        return pfInitialized
                && !isFloorOffsetInitialized
                && floorPrior != null
                && "WIFI".equals(debugSource)
                && matchingWifiFloorCount >= WIFI_BOOTSTRAP_REQUIRED_CONFIRMATIONS;
    }

    private void logStationaryAbsoluteFixSuppression(@NonNull String debugSource) {
        long now = SystemClock.elapsedRealtime();
        if (now - lastStationaryAbsoluteFixLogMs < STATIONARY_LOG_INTERVAL_MS) {
            return;
        }
        Log.i(
                "SensorFusion",
                "Suppressing " + debugSource + " absolute fix while stationary to freeze drift."
                        + " samples=" + lastStationaryWindowSampleCount
                        + " meanAbs=" + formatDebugDouble(lastStationaryWindowMeanMagnitude)
                        + " spikes=" + lastStationaryWindowSpikeCount
        );
        lastStationaryAbsoluteFixLogMs = now;
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
        try {
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
        } catch (RuntimeException ignored) {
            // Local JVM unit tests may not provide a functional GeomagneticField implementation.
        }
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

    public synchronized int getUserVisibleFloor() {
        if (committedDisplayFloorAbsolute != Integer.MIN_VALUE) {
            return committedDisplayFloorAbsolute;
        }
        if (latestFusedPose != null) {
            return clampAbsoluteFloorToVenue(latestFusedPose.getFloor());
        }
        return getAbsoluteCurrentFloor();
    }

    /**
     * True once an accepted absolute floor has calibrated the floor offset. Until then the
     * internal floor value is only a default/unanchored baseline and should not be shown as real.
     */
    public boolean isFloorCalibrated() {
        return isFloorOffsetInitialized;
    }

    private void logFloorDiagnostic(
            @NonNull String event,
            @Nullable Integer floor,
            @NonNull String reason,
            @NonNull String source
    ) {
        safeDebugLog(
                SENSOR_FLOOR_DIAG_TAG,
                "event=" + event
                        + " venue=" + collectionVenue
                        + " floor=" + (floor == null ? "null" : floor)
                        + " trusted=" + isFloorOffsetInitialized
                        + " floorOffsetInitialized=" + isFloorOffsetInitialized
                        + " pdrFloorOffset=" + pdrFloorOffset
                        + " reason=" + reason
                        + " source=" + source
        );
        if (floor != null && floor == 0 && isFloorOffsetInitialized) {
            safeDebugLog(
                    SENSOR_FLOOR_DIAG_TAG,
                    "event=trusted_absolute_floor_zero venue=" + collectionVenue
                            + " floor=0"
                            + " trusted=true"
                            + " floorOffsetInitialized=" + isFloorOffsetInitialized
                            + " pdrFloorOffset=" + pdrFloorOffset
                            + " reason=" + reason
                            + " source=" + source
            );
        }
    }

    private void logFloorSwitchTrace(
            @NonNull String event,
            int targetFloor,
            @NonNull String anchorSource,
            @Nullable LatLng landingLatLng,
            boolean landingLegal,
            boolean tokenIssued,
            boolean committed,
            @NonNull String blockReason
    ) {
        safeDebugLog(
                SENSOR_FLOOR_DIAG_TAG,
                event
                        + " ELEVATOR_TARGET_FLOOR=" + targetFloor
                        + " DESTINATION_LIFT_ANCHOR_FOUND=" + (landingLatLng != null)
                        + " DESTINATION_LIFT_ANCHOR_SOURCE=" + anchorSource
                        + " LANDING_POSE_X="
                        + formatDebugDouble(extractLocalCoordinate(landingLatLng, 0))
                        + " LANDING_POSE_Y="
                        + formatDebugDouble(extractLocalCoordinate(landingLatLng, 1))
                        + " LANDING_POSE_LEGAL=" + landingLegal
                        + " FLOOR_SWITCH_PENDING=" + floorSwitchPending
                        + " FLOOR_SWITCH_COMMITTED=" + committed
                        + " FLOOR_SWITCH_BLOCK_REASON=" + blockReason
                        + " DISPLAY_RESET_TOKEN_ISSUED=" + tokenIssued
                        + " DISPLAY_RESET_TOKEN_CONSUMED=" + committed
                        + " ABSOLUTE_FLOOR=" + getAbsoluteCurrentFloor()
                        + " FUSED_POSE_FLOOR="
                        + (latestFusedPose == null ? "null" : latestFusedPose.getFloor())
                        + " TRUSTED_FLOOR=" + getUserVisibleFloor()
                        + " DISPLAY_FLOOR=" + committedDisplayFloorAbsolute
        );
    }

    private double extractLocalCoordinate(@Nullable LatLng latLng, int index) {
        double[] localMeters = getLocalMetersForLatLng(latLng);
        if (localMeters == null || index < 0 || index >= localMeters.length) {
            return Double.NaN;
        }
        return localMeters[index];
    }

    private void safeDebugLog(@NonNull String tag, @NonNull String message) {
        try {
            Log.d(tag, message);
        } catch (RuntimeException ignored) {
        }
    }

    private synchronized void recalibrateAbsoluteFloorBaselineInternal(
            int absoluteFloor,
            long timestampMs
    ) {
        recalibrateAbsoluteFloorBaselineInternal(
                absoluteFloor,
                timestampMs,
                null,
                "locked_absolute_floor"
        );
    }

    private synchronized void recalibrateAbsoluteFloorBaselineInternal(
            int absoluteFloor,
            long timestampMs,
            @Nullable LiftAnchor liftAnchor,
            @NonNull String floorAnchorState
    ) {
        int boundedAbsoluteFloor = clampAbsoluteFloorToVenue(absoluteFloor);
        int previousAbsoluteFloor = getAbsoluteCurrentFloor();
        cumulativeFloorTracker.reanchorToCurrentElevation(this.elevation);
        this.pdrFloorOffset = boundedAbsoluteFloor;
        this.isFloorOffsetInitialized = true;
        this.blockHistoricalPoseFloorSeedUntilTrustedFix = false;
        resetWifiFloorBootstrapConsensus();
        absoluteFloorTransitionResolver.reset();

        if (liftAnchor != null) {
            reseedPoseToLiftAnchor(liftAnchor, timestampMs);
        } else {
            applyAbsoluteFloorKeepingCurrentXy(boundedAbsoluteFloor, timestampMs);
        }
        if (previousAbsoluteFloor != boundedAbsoluteFloor) {
            activateFloorOnlyResyncWindow(
                    boundedAbsoluteFloor,
                    timestampMs,
                    liftAnchor == null ? null : liftAnchor.latLng,
                    liftAnchor == null ? "none" : "destination_lift_anchor"
            );
        }
        noteFloorConsensusDecision("current_floor_locked", timestampMs);
        lastFloorAnchorState = floorAnchorState;
        if (latestFusedPose != null) {
            recordLatestFusedPoseIfNeeded();
        }
        logFloorDiagnostic(
                "floor_calibration_initialized",
                boundedAbsoluteFloor,
                floorAnchorState,
                liftAnchor != null ? "lift_transfer" : "accepted_floor_prior"
        );
        logInfoSafely(
                "SensorFusion",
                "Recalibrated absolute floor baseline absFloor="
                        + boundedAbsoluteFloor
                        + " elevation="
                        + elevation
                        + " liftAnchor="
                        + (liftAnchor == null ? "none" : formatLatLng(liftAnchor.latLng))
                        + " floorAnchorState="
                        + floorAnchorState
        );
    }

    @Deprecated
    public synchronized void recalibrateAbsoluteFloorBaseline(int absoluteFloor, long timestampMs) {
        recalibrateAbsoluteFloorBaselineIfTrusted(
                absoluteFloor,
                isFloorOffsetInitialized,
                timestampMs
        );
    }

    public synchronized boolean recalibrateAbsoluteFloorBaselineIfTrusted(
            @Nullable Integer absoluteFloor,
            boolean trustedFloor,
            long timestampMs
    ) {
        if (!trustedFloor || absoluteFloor == null) {
            logInfoSafely(
                    "SensorFusion",
                    "Ignoring baseline recalibration from untrusted floor source floor="
                            + (absoluteFloor == null ? "unknown" : absoluteFloor)
                            + " trusted=" + trustedFloor
            );
            return false;
        }
        recalibrateAbsoluteFloorBaselineInternal(absoluteFloor, timestampMs);
        return true;
    }

    public void setVenueFloorHeightMeters(float floorHeightMeters) {
        if (this.pdrProcessing != null && floorHeightMeters > 0f) {
            this.pdrProcessing.setFloorHeightMeters(floorHeightMeters);
        }
        clampCumulativeFloorTrackerToVenueBounds();
        clampLatestFusedPoseFloorToVenueIfNeeded(System.currentTimeMillis());
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

    public boolean isStationary() {
        return isStationary;
    }

    public boolean isFloorOnlyResyncActive() {
        return getActiveFloorOnlyResyncFloor(System.currentTimeMillis()) != null;
    }

    @NonNull
    public MotionDebugSnapshot getMotionDebugSnapshot() {
        long nowMs = System.currentTimeMillis();
        expirePostLiftDestinationFixWindowIfNeeded(nowMs);
        return new MotionDebugSnapshot(
                isStationary,
                lastStationaryWindowSampleCount,
                lastStationaryWindowMeanMagnitude,
                lastStationaryWindowSpikeCount,
                lastStationaryTransitionTimestampMs,
                lastStationaryTransitionReason,
                isMotionResumeWindowActive(motionResumeWindowUntilMs, System.currentTimeMillis()),
                motionResumeWindowUntilMs,
                lastStepDetectorEventTimestampMs,
                lastAcceptedStepTimestampMs,
                lastStepDecision,
                lastAbsoluteFixReceivedTimestampMs,
                lastAcceptedAbsoluteFixTimestampMs,
                lastAbsoluteFixDecision,
                lastRejectReason,
                lastBlockReason,
                lastPoseAdvanceSource,
                lastPoseAdvanceTimestampMs,
                lastDisplayedFusedMarkerTimestampMs,
                latestFusedPose == null ? Long.MIN_VALUE : latestFusedPose.getTimestampMs(),
                getAbsoluteCurrentFloor(),
                getRelativeCurrentFloor(),
                lastFloorConsensus,
                resolveFloorSourceDebug(nowMs),
                lastElevatorGate,
                lastLiftTransferState,
                lastFloorAnchorState,
                lastPostLiftState
        );
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
            stepDetectionSensor.sensorManager.registerListener(
                    this,
                    stepDetectionSensor.sensor,
                    SensorManager.SENSOR_DELAY_FASTEST
            );
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
        int previousFloor = getAbsoluteCurrentFloor();
        boolean wasFloorInitialized = isFloorOffsetInitialized;
        this.pdrFloorOffset = 0;
        this.isFloorOffsetInitialized = false;
        this.blockHistoricalPoseFloorSeedUntilTrustedFix = false;
        this.floorHeightOverrideForTesting = Float.NaN;
        this.cumulativeFloorTracker.reset();
        resetFloorOnlyResyncWindow();
        resetStableAbsoluteFloorConsensus();
        resetWifiFloorBootstrapConsensus();
        clearProvisionalFloorBootstrapState();
        this.lastFloorConsensus = "reset";
        this.lastFloorSource = "relative_only";
        this.lastFloorAnchorState = "unresolved";
        this.lastLiftTransferState = "inactive";
        this.lastPostLiftState = "inactive";
        safeDebugLog(
                SENSOR_FLOOR_DIAG_TAG,
                "event=floor_calibration_invalidated venue=" + collectionVenue
                        + " floor=" + previousFloor
                        + " trustedBefore=" + wasFloorInitialized
                        + " floorOffsetInitialized=" + isFloorOffsetInitialized
                        + " pdrFloorOffset=" + pdrFloorOffset
                        + " reason=start_recording_reset source=null"
        );
        this.elevation = 0f;
        resetElevatorState();
        this.particleFilterEngine = createParticleFilterEngine();
        this.pfInitialized = false;
        this.latestFusedPose = null;
        this.particleCloudTrustedUnderConstraints = false;
        this.absoluteFloorTransitionResolver.reset();
        this.coordinateConverter = hasManualStartLocation && startLocation != null && startLocation.length >= 2
                ? new CoordinateConverter(startLocation[0], startLocation[1])
                : null;
        this.pendingConstrainedAbsoluteFix = null;
        this.allowProvisionalFloorReconcileForCurrentFix = false;
        this.provisionalFloorReconcileTargetFloor = Integer.MIN_VALUE;
        this.hasGeomagneticDeclination = false;
        this.geomagneticDeclinationDeg = 0f;
        this.geomagneticDeclinationTimestampMs = 0L;
        this.geomagneticDeclinationLatitudeDeg = Double.NaN;
        this.geomagneticDeclinationLongitudeDeg = Double.NaN;
        this.geomagneticDeclinationAltitudeMeters = 0f;
        this.lastPredictHeadingRad = getCurrentHeadingRad();
        this.lastPredictHeadingSource = displayOrientation != null
                && displayOrientation.length > 0
                && !Float.isNaN(displayOrientation[0])
                ? "device_display"
                : "device_raw";
        this.lastPredictElevation = this.elevation;
        this.lastRecordedFusedPoseTimestampMs = -1L;
        this.lastWifiScanWallClockMs = -1L;
        this.lastDisplayedFusedMarkerLatLng = null;
        this.lastDisplayedFusedMarkerRawLatLng = null;
        this.lastDisplayedFusedMarkerTimestampMs = -1L;
        this.committedDisplayFloorAbsolute = Integer.MIN_VALUE;
        this.pendingCommittedDisplayFloorAbsolute = Integer.MIN_VALUE;
        this.floorSwitchPending = false;
        this.lastFloorSwitchBlockReason = "none";
        this.pendingDisplayFloorResetLandingLatLng = null;
        this.pendingDisplayFloorResetAnchorSource = "none";
        this.lastDisplayOrientationTimestampMs = Long.MIN_VALUE;
        resetStationaryMotionGate();
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
        resetWifiFloorBootstrapConsensus();
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
        try {
            Log.d(
                    "SensorFusion",
                    "FUSION_DBG ts=" + timestampMs
                            + " source=" + source
                            + " rawLatLon=" + formatLatLon(latitudeDeg, longitudeDeg)
                            + " localEN=" + formatLocal(localFix)
                            + " floorPrior=" + (floorPrior == null ? "n/a" : floorPrior)
                            + " currentFloor=" + getCurrentFloor()
                            + " displayFloor=" + getUserVisibleFloor()
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
        } catch (RuntimeException ignored) {
            // Unit tests may run without an Android Log implementation.
        }
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
        try {
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
                            + " displayFloor=" + getUserVisibleFloor()
                            + " obsAgeMs=" + observationAgeMs
                            + " fused=" + formatFusedPose(latestFusedPose)
                            + " displayedMarker=" + formatLatLng(lastDisplayedFusedMarkerLatLng)
                            + " displayedMarkerEN=" + formatLocal(displayedLocal)
                            + " displayedAgeMs=" + resolveDisplayedMarkerAgeMs(timestampMs)
                            + " pf=" + buildParticleDebugSummary()
            );
        } catch (RuntimeException ignored) {
            // Unit tests may run without an Android Log implementation.
        }
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
                + " reanchor=" + particleFilterEngine.wasLastAbsoluteFixReanchored()
                + " absReject=" + particleFilterEngine.wasLastAbsoluteFixRejectedByConstraints();
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
