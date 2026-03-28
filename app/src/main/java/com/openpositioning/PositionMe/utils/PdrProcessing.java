package com.openpositioning.PositionMe.utils;

import android.content.Context;
import android.content.SharedPreferences;
import android.hardware.SensorManager;

import androidx.preference.PreferenceManager;

import com.openpositioning.PositionMe.sensors.PdrDelta;
import com.openpositioning.PositionMe.sensors.SensorFusion;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Objects;
import java.util.stream.Collectors;

/**
 * Processes data recorded in the {@link SensorFusion} class and calculates live PDR estimates.
 * It calculates the position from the steps and directions detected, using either estimated values
 * (eg. stride length from the Weiberg algorithm) or provided constants, calculates the elevation
 * and attempts to estimate the current floor as well as elevators.
 *
 * @author Mate Stodulka
 * @author Michal Dvorak
 */
public class PdrProcessing {

    //region Static variables
    // Weiberg algorithm coefficient for stride calculations
    private static final float K = 0.364f;
    // Number of samples (seconds) to keep as memory for elevation calculation
    private static final int elevationSeconds = 4;
    // Number of samples (0.01 seconds)
    private static final int accelSamples = 100;
    // Threshold used to detect significant movement
    private static final float movementThreshold = 0.3f; // m/s^2
    // Threshold under which movement is considered non-existent
    private static final float epsilon = 0.18f;
    private static final int MIN_REQUIRED_SAMPLES = 8;
    private static final float MIN_STEP_LENGTH_M = 0.20f;
    private static final float MAX_STEP_LENGTH_M = 0.90f;
    private static final float STEP_LENGTH_SMOOTHING_ALPHA = 0.28f;
    private static final float STEP_LENGTH_GAIN = 0.93f;
    private static final float CADENCE_REFERENCE_HZ = 1.8f;
    private static final float CADENCE_FACTOR_MIN = 0.82f;
    private static final float CADENCE_FACTOR_MAX = 1.24f;
    private static final long DEFAULT_STEP_INTERVAL_MS = 520L;
    private static final long MIN_STEP_INTERVAL_MS = 280L;
    private static final float MIN_DYNAMIC_RANGE_FOR_STEP = 0.08f;
    private static final float ELEVATOR_STEP_SUPPRESSION = 0.75f;
    private static final float MIN_STEP_SCALE = 0.75f;
    private static final float MAX_STEP_SCALE = 1.40f;
    private static final float BASE_STEP_SCALE_CALIBRATION_ALPHA = 0.10f;
    private static final float MIN_CALIBRATION_FIX_QUALITY_M = 15.0f;
    private static final float MIN_CALIBRATION_DISPLACEMENT_M = 2.2f;
    //endregion

    //region Instance variables
    // Settings for accessing shared variables
    private SharedPreferences settings;

    // Step length
    private float stepLength;
    // Using manually input constants instead of estimated values
    private boolean useManualStep;

    // Current 2D position coordinates
    private float positionX;
    private float positionY;

    // Vertical movement calculation
    private Float[] startElevationBuffer;
    private float startElevation;
    private int setupIndex = 0;
    private float elevation;
    private float floorHeight;
    private int currentFloor;

    // Buffer of most recent elevations calculated
    private CircularFloatBuffer elevationList;

    // Buffer for most recent directional acceleration magnitudes
    private CircularFloatBuffer verticalAccel;
    private CircularFloatBuffer horizontalAccel;

    // Step sum and length aggregation variables
    private float sumStepLength = 0;
    private int stepCount = 0;
    private float adaptiveStepScale = 1.0f;
    private final CircularFloatBuffer recentStepLengthBuffer = new CircularFloatBuffer(6);
    private boolean calibrationAnchorReady = false;
    private float anchorPdrX = 0f;
    private float anchorPdrY = 0f;
    private float anchorFusedX = 0f;
    private float anchorFusedY = 0f;
    //endregion

    /**
     * Public constructor for the PDR class.
     * Takes context for variable access. Sets initial values based on settings.
     *
     * @param context   Application context for variable access.
     */
    public PdrProcessing(Context context) {
        // Initialise settings
        this.settings = PreferenceManager.getDefaultSharedPreferences(context);
        // Check if estimate or manual values should be used
        this.useManualStep = this.settings.getBoolean("manual_step_values", false);
        if(useManualStep) {
            try {
                // Retrieve manual step  length
                this.stepLength = this.settings.getInt("user_step_length", 75) / 100f;
            } catch (Exception e) {
                // Invalid values - reset to defaults
                this.stepLength = 0.75f;
                this.settings.edit().putInt("user_step_length", 75).apply();
            }
        }
        else {
            // Using estimated step length - set to zero
            this.stepLength = 0;
        }

        // Initial position and elevation - starts from zero
        this.positionX = 0f;
        this.positionY = 0f;
        this.elevation = 0f;


        if(this.settings.getBoolean("overwrite_constants", false)) {
            // Capacity - pressure is read with 1Hz - store values of past 10 seconds
            this.elevationList = new CircularFloatBuffer(Integer.parseInt(settings.getString("elevation_seconds", "4")));

            // Buffer for most recent acceleration values
            this.verticalAccel = new CircularFloatBuffer(Integer.parseInt(settings.getString("accel_samples", "4")));
            this.horizontalAccel = new CircularFloatBuffer(Integer.parseInt(settings.getString("accel_samples", "4")));
        }
        else {
            // Capacity - pressure is read with 1Hz - store values of past 10 seconds
            this.elevationList = new CircularFloatBuffer(elevationSeconds);

            // Buffer for most recent acceleration values
            this.verticalAccel = new CircularFloatBuffer(accelSamples);
            this.horizontalAccel = new CircularFloatBuffer(accelSamples);
        }

        // Distance between floors is building dependent, use manual value
        this.floorHeight = settings.getInt("floor_height", 4);
        // Array for holding initial values
        this.startElevationBuffer = new Float[3];
        // Start floor - assumed to be zero
        this.currentFloor = 0;
    }

    /**
     * Function to calculate PDR coordinates from sensor values.
     * Should be called from the step detector sensor's event with the sensor values since the last
     * step.
     *
     * @param currentStepEnd            relative time in milliseconds since the start of the recording.
     * @param accelMagnitudeOvertime    recorded acceleration magnitudes since the last step.
     * @param headingRad                heading relative to magnetic north in radians.
     */
    public float[] updatePdr(long currentStepEnd, List<Double> accelMagnitudeOvertime, float headingRad) {
        PdrDelta delta = buildStepDelta(accelMagnitudeOvertime, 0f, 0f);
        return applyStepDelta(delta, headingRad);
    }

    public PdrDelta buildStepDelta(
            List<Double> accelMagnitudeOvertime,
            float deltaHeadingRad,
            float heightDeltaMeters
    ) {
        return buildStepDelta(
                accelMagnitudeOvertime,
                deltaHeadingRad,
                heightDeltaMeters,
                false,
                DEFAULT_STEP_INTERVAL_MS
        );
    }

    public PdrDelta buildStepDelta(
            List<Double> accelMagnitudeOvertime,
            float deltaHeadingRad,
            float heightDeltaMeters,
            boolean elevatorLikely
    ) {
        return buildStepDelta(
                accelMagnitudeOvertime,
                deltaHeadingRad,
                heightDeltaMeters,
                elevatorLikely,
                DEFAULT_STEP_INTERVAL_MS
        );
    }

    public PdrDelta buildStepDelta(
            List<Double> accelMagnitudeOvertime,
            float deltaHeadingRad,
            float heightDeltaMeters,
            boolean elevatorLikely,
            long stepIntervalMs
    ) {
        float computedStepLength = computeStepLength(accelMagnitudeOvertime, elevatorLikely, stepIntervalMs);
        if (computedStepLength > 0f) {
            sumStepLength += computedStepLength;
            stepCount++;
        }
        return new PdrDelta(computedStepLength, deltaHeadingRad, heightDeltaMeters, elevatorLikely);
    }

    /**
     * Projects a step into the local coursework frame used across the app.
     *
     * <p>The heading follows the Android azimuth convention consumed by {@link SensorFusion}:
     * {@code 0} radians points north and positive rotation turns east. The returned vector is
     * {@code [eastMeters, northMeters]}.</p>
     *
     * @param stepLengthMeters step length in meters
     * @param headingRad heading in radians
     * @return local step vector as {@code [eastMeters, northMeters]}
     */
    public static double[] projectStepToLocalFrame(double stepLengthMeters, double headingRad) {
        return new double[]{
                stepLengthMeters * Math.sin(headingRad),
                stepLengthMeters * Math.cos(headingRad)
        };
    }

    public float[] applyStepDelta(PdrDelta delta, float headingRad) {
        if (delta == null || delta.getStepLengthMeters() <= 0f) {
            return new float[]{this.positionX, this.positionY};
        }

        double[] localStep = projectStepToLocalFrame(delta.getStepLengthMeters(), headingRad);
        float x = (float) localStep[0];
        float y = (float) localStep[1];

        this.positionX += x;
        this.positionY += y;
        return new float[]{this.positionX, this.positionY};
    }

    /**
     * Force-set current PDR local position in meters.
     */
    public void setPdrPosition(float xMeters, float yMeters) {
        this.positionX = xMeters;
        this.positionY = yMeters;
    }

    /**
     * Calculates the relative elevation compared to the start position.
     * The start elevation is the median of the first three seconds of data to give the sensor time
     * to settle. The sea level is irrelevant as only values relative to the initial position are
     * reported.
     *
     * @param absoluteElevation absolute elevation in meters compared to sea level.
     * @return                  current elevation in meters relative to the start position.
     */
    public float updateElevation(float absoluteElevation) {
        // Set start to median of first three values
        if(setupIndex < 3) {
            // Add values to buffer until it's full
            this.startElevationBuffer[setupIndex] = absoluteElevation;
            // When buffer is full, find median, assign as startElevation
            if(setupIndex == 2) {
                Arrays.sort(startElevationBuffer);
                startElevation = startElevationBuffer[1];
            }
            this.setupIndex++;
        }
        else {
            // Get relative elevation in meters
            this.elevation = absoluteElevation - startElevation;
            // Add to buffer
            this.elevationList.putNewest(absoluteElevation);
            // Elevation-dominant floor estimation: each floor occupies a height band centered on
            // n * floorHeight, i.e. [(n-0.5)h, (n+0.5)h).
            this.currentFloor = resolveFloorFromElevation(this.elevation);
            // Return current elevation
            return elevation;
        }
        // Keep elevation at zero if there is no calculated value
        return 0;
    }

    /**
     * Uses the Weiberg Stride Length formula to calculate step length from accelerometer values.
     *
     * @param accelMagnitude    magnitude of acceleration values between the last and current step.
     * @return                  float stride length in meters.
     */
    private float weibergMinMax(List<Double> accelMagnitude) {
        // if the list itself is null or empty, return 0 (or return other default values as needed)
        if (accelMagnitude == null || accelMagnitude.isEmpty()) {
            return 0f;
        }

        // filter out null values from the list
        List<Double> validAccel = accelMagnitude.stream()
                .filter(Objects::nonNull)
                .collect(Collectors.toList());
        if (validAccel.isEmpty()) {
            return 0f;
        }

        // calculate max and min values
        double maxAccel = Collections.max(validAccel);
        double minAccel = Collections.min(validAccel);

        // calculate bounce
        float bounce = (float) Math.pow((maxAccel - minAccel), 0.25);

        // determine which constant to use based on settings
        if (this.settings.getBoolean("overwrite_constants", false)) {
            return bounce * Float.parseFloat(settings.getString("weiberg_k", "0.934")) * 2;
        }

        return bounce * K * 2;
    }

    private float computeStepLength(
            List<Double> accelMagnitudeOvertime,
            boolean elevatorLikely,
            long stepIntervalMs
    ) {
        if (useManualStep) {
            return this.stepLength;
        }
        if (accelMagnitudeOvertime == null || accelMagnitudeOvertime.size() < MIN_REQUIRED_SAMPLES) {
            return 0f;
        }
        if (accelMagnitudeOvertime.isEmpty()) {
            return 0f;
        }
        float rawStep = robustWeibergStep(accelMagnitudeOvertime);
        float cadenceFactor = computeCadenceFactor(stepIntervalMs);
        float scaledStep;
        if (rawStep > 0f) {
            scaledStep = rawStep * STEP_LENGTH_GAIN * cadenceFactor * adaptiveStepScale;
        } else {
            // Keep moving when step detector fires but waveform quality is poor.
            // This avoids "stuck in place" behaviour under noisy linear acceleration streams.
            float baseStride = this.stepLength > 0f ? this.stepLength : 0.68f;
            scaledStep = baseStride * cadenceFactor * adaptiveStepScale;
        }
        if (elevatorLikely) {
            scaledStep *= ELEVATOR_STEP_SUPPRESSION;
        }
        float smoothedStep = scaledStep;
        if (this.stepLength > 0f) {
            smoothedStep = STEP_LENGTH_SMOOTHING_ALPHA * scaledStep
                    + (1f - STEP_LENGTH_SMOOTHING_ALPHA) * this.stepLength;
        }
        this.stepLength = clamp(applyRecentStepMedianGuard(smoothedStep), MIN_STEP_LENGTH_M, MAX_STEP_LENGTH_M);
        recentStepLengthBuffer.putNewest(this.stepLength);
        return this.stepLength;
    }

    private float robustWeibergStep(List<Double> accelMagnitudeOvertime) {
        List<Double> validAccel = accelMagnitudeOvertime.stream()
                .filter(Objects::nonNull)
                .collect(Collectors.toList());
        if (validAccel.size() < MIN_REQUIRED_SAMPLES) {
            return 0f;
        }
        List<Double> sorted = new ArrayList<>(validAccel);
        Collections.sort(sorted);

        double p10 = percentile(sorted, 0.10);
        double p90 = percentile(sorted, 0.90);
        if (Double.isNaN(p10) || Double.isNaN(p90) || p90 <= p10) {
            return 0f;
        }
        if ((p90 - p10) < MIN_DYNAMIC_RANGE_FOR_STEP) {
            return 0f;
        }

        float bounce = (float) Math.pow((p90 - p10), 0.25);
        float k = K;
        if (this.settings.getBoolean("overwrite_constants", false)) {
            k = Float.parseFloat(settings.getString("weiberg_k", "0.934"));
        }
        return bounce * k * 2f;
    }

    private double percentile(List<Double> sortedValues, double percentile) {
        if (sortedValues == null || sortedValues.isEmpty()) {
            return Double.NaN;
        }
        double bounded = Math.max(0.0, Math.min(1.0, percentile));
        double index = bounded * (sortedValues.size() - 1);
        int lower = (int) Math.floor(index);
        int upper = (int) Math.ceil(index);
        if (lower == upper) {
            return sortedValues.get(lower);
        }
        double weight = index - lower;
        return sortedValues.get(lower) * (1.0 - weight) + sortedValues.get(upper) * weight;
    }

    private float clamp(float value, float minValue, float maxValue) {
        return Math.max(minValue, Math.min(maxValue, value));
    }

    private float computeCadenceFactor(long stepIntervalMs) {
        long safeInterval = Math.max(MIN_STEP_INTERVAL_MS, stepIntervalMs);
        float cadenceHz = 1000f / safeInterval;
        float cadenceFactor = (float) Math.pow(cadenceHz / CADENCE_REFERENCE_HZ, 0.22);
        return clamp(cadenceFactor, CADENCE_FACTOR_MIN, CADENCE_FACTOR_MAX);
    }

    private float applyRecentStepMedianGuard(float candidateStep) {
        List<Float> recent = recentStepLengthBuffer.getListCopy();
        if (recent == null || recent.isEmpty()) {
            return candidateStep;
        }
        List<Float> sorted = new ArrayList<>(recent);
        Collections.sort(sorted);
        float median = sorted.get(sorted.size() / 2);
        if (median <= 0f) {
            return candidateStep;
        }
        float bounded = clamp(candidateStep, median * 0.55f, median * 1.60f);
        return bounded;
    }

    /**
     * Get the current X and Y coordinates from the PDR processing class.
     * The coordinates are in meters, the start of the recording is the (0,0)
     *
     * @return  float array of size 2, with the X and Y coordinates respectively.
     */
    public float[] getPDRMovement() {
        float [] pdrPosition= new float[] {positionX,positionY};
        return pdrPosition;

    }

    /**
     * Get the current elevation as calculated by the PDR class.
     *
     * @return  current elevation in meters, relative to the start position.
     */
    public float getCurrentElevation() {
        return this.elevation;
    }

    /**
     * Get the current floor number as estimated by the PDR class.
     *
     * @return current floor number, assuming start position is on level zero.
     */
    public int getCurrentFloor() {
        return this.currentFloor;
    }

    private int resolveFloorFromElevation(float relativeElevationMeters) {
        if (floorHeight <= 0f || Float.isNaN(relativeElevationMeters) || Float.isInfinite(relativeElevationMeters)) {
            return currentFloor;
        }
        return Math.round(relativeElevationMeters / floorHeight);
    }

    /**
     * Override floor height at runtime (for example from venue metadata).
     */
    public void setFloorHeight(float floorHeightMeters) {
        if (Float.isNaN(floorHeightMeters) || Float.isInfinite(floorHeightMeters) || floorHeightMeters <= 0f) {
            return;
        }
        this.floorHeight = floorHeightMeters;
    }

    /**
     * Re-anchor barometric floor estimation to a known floor label.
     * This is useful when entering a building on a non-zero floor (e.g. G/2/LG).
     */
    public void recalibrateFloorAnchor(float absoluteElevationMeters, int anchorFloor) {
        if (Float.isNaN(absoluteElevationMeters) || Float.isInfinite(absoluteElevationMeters) || floorHeight <= 0f) {
            return;
        }
        this.startElevation = absoluteElevationMeters - (anchorFloor * floorHeight);
        this.elevation = absoluteElevationMeters - this.startElevation;
        this.currentFloor = resolveFloorFromElevation(this.elevation);
        this.setupIndex = 3; // mark baseline as initialized
        this.startElevationBuffer = new Float[]{startElevation, startElevation, startElevation};

        int capacity = elevationList != null ? elevationList.getCapacity() : elevationSeconds;
        this.elevationList = new CircularFloatBuffer(capacity);
        for (int i = 0; i < capacity; i++) {
            this.elevationList.putNewest(absoluteElevationMeters);
        }
    }

    /**
     * Estimates if the user is currently taking an elevator.
     * From the gravity and gravity-removed acceleration values the magnitude of horizontal and
     * vertical acceleration is calculated and stored over time. Averaging these values and
     * comparing with the thresholds set for this class, it estimates if the current movement
     * matches what is expected from an elevator ride.
     *
     * @param gravity   array of size three, strength of gravity along the phone's x-y-z axis.
     * @param acc       array of size three, acceleration other than gravity detected by the phone.
     * @return          boolean true if currently in an elevator, false otherwise.
     */
    public boolean estimateElevator(float[] gravity, float[] acc) {
        // Standard gravity
        float g = SensorManager.STANDARD_GRAVITY;
        // get horizontal and vertical acceleration magnitude
        float verticalAcc = (float) Math.sqrt(
                Math.pow((acc[0] * gravity[0]/g),2) +
                Math.pow((acc[1] * gravity[1]/g), 2) +
                Math.pow((acc[2] * gravity[2]/g), 2));
        float horizontalAcc = (float) Math.sqrt(
                Math.pow((acc[0] * (1 - gravity[0]/g)), 2) +
                Math.pow((acc[1] * (1 - gravity[1]/g)), 2) +
                Math.pow((acc[2] * (1 - gravity[2]/g)), 2));
        // Save into buffer to compare with past values
        this.verticalAccel.putNewest(verticalAcc);
        this.horizontalAccel.putNewest(horizontalAcc);
        // Once buffer is full, evaluate data
        if(this.verticalAccel.isFull() && this.horizontalAccel.isFull()) {

            // calculate average vertical accel
            List<Float> verticalMemory = this.verticalAccel.getListCopy();
            OptionalDouble optVerticalAvg = verticalMemory.stream().mapToDouble(Math::abs).average();
            float verticalAvg = optVerticalAvg.isPresent() ? (float) optVerticalAvg.getAsDouble() : 0;


            // calculate average horizontal accel
            List<Float> horizontalMemory = this.horizontalAccel.getListCopy();
            OptionalDouble optHorizontalAvg = horizontalMemory.stream().mapToDouble(Math::abs).average();
            float horizontalAvg = optHorizontalAvg.isPresent() ? (float) optHorizontalAvg.getAsDouble() : 0;

            //System.err.println("LIFT: Vertical: " + verticalAvg);
            //System.err.println("LIFT: Horizontal: " + horizontalAvg);

            if(this.settings.getBoolean("overwrite_constants", false)) {
                float eps = Float.parseFloat(settings.getString("epsilon", "0.18"));
                return horizontalAvg < eps && verticalAvg > movementThreshold;
            }
            // Check if there is minimal horizontal and significant vertical movement
            return horizontalAvg < epsilon && verticalAvg > movementThreshold;
        }
        return false;

    }

    /**
     * Resets all values stored in the PDR function and re-initialises all buffers.
     * Used to reset to zero position and remove existing history.
     */
    public void resetPDR() {
        // Check if estimate or manual values should be used
        this.useManualStep = this.settings.getBoolean("manual_step_values", false);
        if(useManualStep) {
            try {
                // Retrieve manual step  length
                this.stepLength = this.settings.getInt("user_step_length", 75) / 100f;
            } catch (Exception e) {
                // Invalid values - reset to defaults
                this.stepLength = 0.75f;
                this.settings.edit().putInt("user_step_length", 75).apply();
            }
        }
        else {
            // Using estimated step length - set to zero
            this.stepLength = 0;
        }

        // Initial position and elevation - starts from zero
        this.positionX = 0f;
        this.positionY = 0f;
        this.elevation = 0f;

        if(this.settings.getBoolean("overwrite_constants", false)) {
            // Capacity - pressure is read with 1Hz - store values of past 10 seconds
            this.elevationList = new CircularFloatBuffer(Integer.parseInt(settings.getString("elevation_seconds", "4")));

            // Buffer for most recent acceleration values
            this.verticalAccel = new CircularFloatBuffer(Integer.parseInt(settings.getString("accel_samples", "4")));
            this.horizontalAccel = new CircularFloatBuffer(Integer.parseInt(settings.getString("accel_samples", "4")));
        }
        else {
            // Capacity - pressure is read with 1Hz - store values of past 10 seconds
            this.elevationList = new CircularFloatBuffer(elevationSeconds);

            // Buffer for most recent acceleration values
            this.verticalAccel = new CircularFloatBuffer(accelSamples);
            this.horizontalAccel = new CircularFloatBuffer(accelSamples);
        }

        // Distance between floors is building dependent, use manual value
        this.floorHeight = settings.getInt("floor_height", 4);
        // Array for holding initial values
        this.startElevationBuffer = new Float[3];
        // Start floor - assumed to be zero
        this.currentFloor = 0;
        this.adaptiveStepScale = 1.0f;
        this.calibrationAnchorReady = false;
        this.anchorPdrX = 0f;
        this.anchorPdrY = 0f;
        this.anchorFusedX = 0f;
        this.anchorFusedY = 0f;
    }

    /**
     * Online scale calibration using fused absolute position displacement.
     * Keeps PDR step model aligned with long-term travelled distance.
     */
    public void updateStepScaleFromAbsoluteFix(float fusedX, float fusedY, float fixAccuracyMeters) {
        if (Float.isNaN(fixAccuracyMeters) || fixAccuracyMeters > MIN_CALIBRATION_FIX_QUALITY_M) {
            return;
        }
        if (!calibrationAnchorReady) {
            calibrationAnchorReady = true;
            anchorPdrX = positionX;
            anchorPdrY = positionY;
            anchorFusedX = fusedX;
            anchorFusedY = fusedY;
            return;
        }

        float pdrDx = positionX - anchorPdrX;
        float pdrDy = positionY - anchorPdrY;
        float fusedDx = fusedX - anchorFusedX;
        float fusedDy = fusedY - anchorFusedY;

        float pdrDistance = (float) Math.hypot(pdrDx, pdrDy);
        float fusedDistance = (float) Math.hypot(fusedDx, fusedDy);
        if (pdrDistance < MIN_CALIBRATION_DISPLACEMENT_M || fusedDistance < MIN_CALIBRATION_DISPLACEMENT_M) {
            return;
        }

        float observedScale = fusedDistance / Math.max(pdrDistance, 1e-3f);
        observedScale = clamp(observedScale, MIN_STEP_SCALE, MAX_STEP_SCALE);
        float confidence = clamp((MIN_CALIBRATION_FIX_QUALITY_M - fixAccuracyMeters) / MIN_CALIBRATION_FIX_QUALITY_M, 0.15f, 1.0f);
        float alpha = BASE_STEP_SCALE_CALIBRATION_ALPHA * confidence;
        adaptiveStepScale = clamp(
                (1f - alpha) * adaptiveStepScale + alpha * observedScale,
                MIN_STEP_SCALE,
                MAX_STEP_SCALE
        );

        anchorPdrX = positionX;
        anchorPdrY = positionY;
        anchorFusedX = fusedX;
        anchorFusedY = fusedY;
    }

    /**
     * Getter for the average step length calculated from the aggregated distance and step count.
     *
     * @return  average step length in meters.
     */
    public float getAverageStepLength(){
        if (stepCount == 0) {
            return 0f;
        }
        //Calculate average step length
        float averageStepLength = sumStepLength/(float) stepCount;

        //Reset sum and number of steps
        stepCount = 0;
        sumStepLength = 0;

        //Return average step length
        return averageStepLength;
    }

}
