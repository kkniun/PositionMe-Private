package com.openpositioning.PositionMe.utils;

import android.location.Location;

import androidx.annotation.Nullable;

import com.google.android.gms.maps.model.LatLng;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

/**
 * Local EN particle-filter tracker used to combine GNSS, WiFi and PDR updates.
 *
 * <p>The filter deliberately adapts its noise parameters to the incoming measurements instead of
 * using the design note values verbatim. GNSS accuracy, WiFi floor availability and the actual
 * PDR step magnitude all influence the active model parameters.</p>
 */
public class ParticleFilterEngine {

    public enum ObservationSource {
        GNSS,
        WIFI
    }

    private static final int PARTICLE_COUNT = 240;
    private static final int MAX_TAIL_SIZE = 5;
    private static final int MAX_HISTORY_SIZE = 600;
    private static final long MIN_HISTORY_INTERVAL_MS = 1_000L;
    private static final long HISTORY_ACTIVE_MOTION_WINDOW_MS = 2_500L;
    private static final double MIN_HISTORY_DISTANCE_METERS = 0.85;
    private static final double MAX_HISTORY_SEGMENT_METERS = 4.0;
    private static final double STATIONARY_REBASE_DISTANCE_METERS = 2.5;
    private static final double MAX_STEP_METERS = 2.5;
    private static final double MIN_INIT_CONFIDENCE = 0.40;
    private static final double RESAMPLE_THRESHOLD_RATIO = 0.48;
    private static final double WIFI_FLOOR_SIGMA = 0.9;
    private static final double FLOOR_MODE_SWITCH_DOMINANCE = 0.46;
    private static final double FLOOR_MODE_SWITCH_MARGIN = 0.10;
    private static final double FLOOR_JITTER_PROBABILITY = 0.08;
    private static final double GNSS_OBSERVATION_PULL_MIN = 0.28;
    private static final double GNSS_OBSERVATION_PULL_MAX = 0.64;
    private static final double WIFI_OBSERVATION_PULL_MIN = 0.22;
    private static final double WIFI_OBSERVATION_PULL_MAX = 0.48;
    private static final double EARTH_RADIUS_METERS = 6_378_137.0;

    private final Random random = new Random();
    private final ArrayList<Particle> particles = new ArrayList<>(PARTICLE_COUNT);
    private final ArrayDeque<LatLng> fusedHistory = new ArrayDeque<>();
    private final ArrayDeque<LatLng> gnssTail = new ArrayDeque<>();
    private final ArrayDeque<LatLng> wifiTail = new ArrayDeque<>();
    private final ArrayDeque<LatLng> pdrTail = new ArrayDeque<>();
    private final IndoorSpatialConstraintModel spatialConstraintModel;
    private final AdaptivePlanarKalmanFilter displayKalmanFilter = new AdaptivePlanarKalmanFilter();

    private EnuReference reference;
    private boolean initialized;
    private boolean hasDisplayPose;
    private boolean hasLastPdrCoordinate;

    private float lastPdrEast;
    private float lastPdrNorth;
    private double rawEasting;
    private double rawNorthing;
    private double rawHeadingRad;
    private double displayEasting;
    private double displayNorthing;
    private double displayHeadingRad;
    private double pdrTrackEasting;
    private double pdrTrackNorthing;
    private int latestWifiFloor;
    private int dominantParticleFloor;
    private LatLng currentLatLng;
    private long lastHistoryTimestamp;
    private long lastPdrMotionTimestamp;
    private long currentPositionVersion;
    private long fusedHistoryVersion;
    private long gnssTailVersion;
    private long wifiTailVersion;
    private long pdrTailVersion;

    public ParticleFilterEngine() {
        this(new IndoorSpatialConstraintModel());
    }

    public ParticleFilterEngine(IndoorSpatialConstraintModel spatialConstraintModel) {
        this.spatialConstraintModel = spatialConstraintModel;
    }

    /**
     * Clears the full filter state and all UI traces.
     */
    public synchronized void reset() {
        particles.clear();
        fusedHistory.clear();
        gnssTail.clear();
        wifiTail.clear();
        pdrTail.clear();
        displayKalmanFilter.reset();
        reference = null;
        initialized = false;
        hasDisplayPose = false;
        hasLastPdrCoordinate = false;
        lastPdrEast = 0f;
        lastPdrNorth = 0f;
        rawEasting = 0d;
        rawNorthing = 0d;
        rawHeadingRad = 0d;
        displayEasting = 0d;
        displayNorthing = 0d;
        displayHeadingRad = 0d;
        pdrTrackEasting = 0d;
        pdrTrackNorthing = 0d;
        latestWifiFloor = 0;
        dominantParticleFloor = 0;
        currentLatLng = null;
        lastHistoryTimestamp = 0L;
        lastPdrMotionTimestamp = 0L;
        currentPositionVersion = 0L;
        fusedHistoryVersion = 0L;
        gnssTailVersion = 0L;
        wifiTailVersion = 0L;
        pdrTailVersion = 0L;
        spatialConstraintModel.reset();
    }

    /**
     * PDR coordinates are reset to zero when a new recording starts.
     * The fusion state stays where it is; only the relative PDR origin is re-anchored.
     */
    public synchronized void onPdrStreamReset() {
        hasLastPdrCoordinate = false;
        if (hasDisplayPose) {
            pdrTrackEasting = rawEasting;
            pdrTrackNorthing = rawNorthing;
        }
    }

    public synchronized boolean isInitialized() {
        return initialized && currentLatLng != null;
    }

    @Nullable
    public synchronized LatLng getCurrentLatLng() {
        return currentLatLng;
    }

    public synchronized int getLatestWifiFloor() {
        return latestWifiFloor;
    }

    public synchronized int normalizeObservedFloor(int observedFloor) {
        return spatialConstraintModel.normalizeExternalFloorObservation(observedFloor);
    }

    public synchronized int getCurrentLogicalFloor() {
        return spatialConstraintModel.getCurrentLogicalFloor();
    }

    public synchronized int getDominantParticleFloor() {
        return dominantParticleFloor;
    }

    public synchronized void setCurrentLogicalFloor(int logicalFloor) {
        spatialConstraintModel.setCurrentLogicalFloor(logicalFloor);
        dominantParticleFloor = spatialConstraintModel.getCurrentLogicalFloor();
        synchronizeParticlesToResolvedFloor(spatialConstraintModel.getCurrentLogicalFloor());
    }

    public synchronized float getCurrentFloorHeight() {
        return spatialConstraintModel.getCurrentFloorHeight();
    }

    public synchronized String getCurrentFloorDisplayName() {
        return spatialConstraintModel.getCurrentFloorDisplayName();
    }

    @Nullable
    public synchronized String getCurrentBuildingId() {
        return spatialConstraintModel.getCurrentBuildingId();
    }

    public synchronized boolean isIndoorContextActive() {
        return spatialConstraintModel.hasIndoorContext();
    }

    public synchronized boolean isNearIndoorFeature(@Nullable LatLng point,
                                                    String indoorType,
                                                    double radiusMeters) {
        return spatialConstraintModel.isNearFeature(point, indoorType, radiusMeters);
    }

    public synchronized List<LatLng> getFusedHistory() {
        return new ArrayList<>(fusedHistory);
    }

    public synchronized long getCurrentPositionVersion() {
        return currentPositionVersion;
    }

    public synchronized long getFusedHistoryVersion() {
        return fusedHistoryVersion;
    }

    public synchronized long getObservationTrailsVersion() {
        return gnssTailVersion + wifiTailVersion + pdrTailVersion;
    }

    public synchronized List<LatLng> getRecentGnssTail() {
        return new ArrayList<>(gnssTail);
    }

    public synchronized List<LatLng> getRecentWifiTail() {
        return new ArrayList<>(wifiTail);
    }

    public synchronized List<LatLng> getRecentPdrTail() {
        return new ArrayList<>(pdrTail);
    }

    /**
     * Applies a post-filter map correction, for example after anti-wall clipping on the map layer.
     */
    public synchronized void overrideCurrentPosition(@Nullable LatLng corrected, long timestampMillis) {
        if (!initialized || corrected == null || reference == null) {
            return;
        }

        double[] local = reference.toLocal(corrected);
        double deltaE = local[0] - displayEasting;
        double deltaN = local[1] - displayNorthing;

        for (Particle particle : particles) {
            particle.easting += deltaE;
            particle.northing += deltaN;
        }

        rawEasting += deltaE;
        rawNorthing += deltaN;
        displayEasting = local[0];
        displayNorthing = local[1];
        displayKalmanFilter.reset(local[0], local[1], timestampMillis);
        currentLatLng = corrected;
        currentPositionVersion++;
        appendHistory(corrected, timestampMillis, true);
        spatialConstraintModel.updatePosition(corrected);
    }

    /**
     * Injects a GNSS observation into the filter.
     */
    @Nullable
    public synchronized LatLng onGnssObservation(Location location, double headingRad) {
        if (location == null) {
            return currentLatLng;
        }

        double accuracyMeters = location.hasAccuracy()
                ? clamp(location.getAccuracy(), 3.0, 45.0)
                : 22.0;
        double confidence;
        if (accuracyMeters <= 10.0) {
            confidence = 0.92;
        } else if (accuracyMeters <= 20.0) {
            confidence = 0.76;
        } else if (accuracyMeters <= 35.0) {
            confidence = 0.60;
        } else {
            confidence = 0.42;
        }

        return onAbsoluteObservation(
                new LatLng(location.getLatitude(), location.getLongitude()),
                ObservationSource.GNSS,
                accuracyMeters,
                confidence,
                headingRad,
                System.currentTimeMillis(),
                null
        );
    }

    /**
     * Injects a WiFi positioning observation into the filter.
     */
    @Nullable
    public synchronized LatLng onWifiObservation(@Nullable LatLng location,
                                                 int floor,
                                                 double headingRad,
                                                 long timestampMillis) {
        if (location == null) {
            return currentLatLng;
        }

        double accuracyMeters = floor != 0 ? 7.0 : 8.5;
        double confidence = floor != 0 ? 0.86 : 0.78;

        return onAbsoluteObservation(
                location,
                ObservationSource.WIFI,
                accuracyMeters,
                confidence,
                headingRad,
                timestampMillis,
                floor
        );
    }

    /**
     * Injects a PDR absolute coordinate. The filter internally converts it to a single-step delta.
     */
    @Nullable
    public synchronized LatLng onPdrAbsolute(float absoluteEast,
                                             float absoluteNorth,
                                             long timestampMillis,
                                             double headingRad) {
        if (!hasLastPdrCoordinate) {
            lastPdrEast = absoluteEast;
            lastPdrNorth = absoluteNorth;
            hasLastPdrCoordinate = true;
            if (initialized) {
                pdrTrackEasting = rawEasting;
                pdrTrackNorthing = rawNorthing;
            }
            return currentLatLng;
        }

        double deltaE = absoluteEast - lastPdrEast;
        double deltaN = absoluteNorth - lastPdrNorth;
        lastPdrEast = absoluteEast;
        lastPdrNorth = absoluteNorth;

        double stepDistance = Math.hypot(deltaE, deltaN);
        if (stepDistance < 0.05) {
            return currentLatLng;
        }

        if (stepDistance > MAX_STEP_METERS) {
            onPdrStreamReset();
            return currentLatLng;
        }

        lastPdrMotionTimestamp = timestampMillis;

        if (!initialized) {
            return null;
        }

        double motionHeading = stepDistance > 1e-6
                ? normalizeRad(Math.atan2(deltaE, deltaN))
                : normalizeHeading(headingRad);

        predict(stepDistance, motionHeading);

        pdrTrackEasting += deltaE;
        pdrTrackNorthing += deltaN;
        if (appendTail(pdrTail, reference.toLatLng(pdrTrackEasting, pdrTrackNorthing))) {
            pdrTailVersion++;
        }

        PoseEstimate estimate = estimatePose();
        commitPose(
                estimate.easting,
                estimate.northing,
                estimate.headingRad,
                estimate.logicalFloor,
                timestampMillis,
                0.22,
                clamp(0.55 + stepDistance * 0.35, 0.55, 1.35)
        );
        return currentLatLng;
    }

    @Nullable
    private LatLng onAbsoluteObservation(LatLng latLng,
                                         ObservationSource source,
                                         double accuracyMeters,
                                         double confidence,
                                         double headingRad,
                                         long timestampMillis,
                                         @Nullable Integer floor) {
        spatialConstraintModel.updatePosition(latLng);
        Integer normalizedFloor = floor;
        if (source == ObservationSource.WIFI && floor != null && spatialConstraintModel.hasIndoorContext()) {
            normalizedFloor = spatialConstraintModel.normalizeExternalFloorObservation(floor);
            latestWifiFloor = normalizedFloor;
        }

        if (source == ObservationSource.GNSS) {
            if (appendTail(gnssTail, latLng)) {
                gnssTailVersion++;
            }
        } else {
            if (appendTail(wifiTail, latLng)) {
                wifiTailVersion++;
            }
            if (normalizedFloor != null) {
                latestWifiFloor = normalizedFloor;
            }
        }

        if (!initialized) {
            if (confidence < MIN_INIT_CONFIDENCE) {
                return null;
            }

            reference = new EnuReference(latLng);
            if (normalizedFloor != null && spatialConstraintModel.hasIndoorContext()) {
                spatialConstraintModel.setCurrentLogicalFloor(normalizedFloor);
            }
            initialiseParticles(
                    accuracyMeters,
                    headingRad,
                    spatialConstraintModel.getCurrentLogicalFloor(),
                    normalizedFloor,
                    confidence
            );
            initialized = true;
            pdrTrackEasting = 0d;
            pdrTrackNorthing = 0d;
            commitPose(
                    0d,
                    0d,
                    normalizeHeading(headingRad),
                    spatialConstraintModel.getCurrentLogicalFloor(),
                    timestampMillis,
                    1.0,
                    clamp(accuracyMeters * 0.40, 0.9, source == ObservationSource.WIFI ? 3.0 : 5.5)
            );
            return currentLatLng;
        }

        if (source == ObservationSource.WIFI && normalizedFloor != null) {
            injectFloorHypotheses(normalizedFloor, confidence);
        }
        double[] local = reference.toLocal(latLng);
        updateWeights(local[0], local[1], accuracyMeters, confidence, source, normalizedFloor);

        if (effectiveParticleCount() < RESAMPLE_THRESHOLD_RATIO * particles.size()) {
            resample();
        }

        pullParticlesTowardObservation(
                local[0],
                local[1],
                accuracyMeters,
                confidence,
                source
        );

        PoseEstimate estimate = estimatePose();
        double smoothing = source == ObservationSource.WIFI ? 0.58 : 0.68;
        commitPose(
                estimate.easting,
                estimate.northing,
                estimate.headingRad,
                estimate.logicalFloor,
                timestampMillis,
                smoothing,
                source == ObservationSource.WIFI
                        ? clamp(accuracyMeters * 0.36, 0.9, 3.0)
                        : clamp(accuracyMeters * 0.46, 1.1, 6.0)
        );
        return currentLatLng;
    }

    private void pullParticlesTowardObservation(double observedEasting,
                                                double observedNorthing,
                                                double accuracyMeters,
                                                double confidence,
                                                ObservationSource source) {
        double basePull = source == ObservationSource.WIFI
                ? clamp(0.18 + confidence * 0.28, WIFI_OBSERVATION_PULL_MIN, WIFI_OBSERVATION_PULL_MAX)
                : clamp(0.24 + confidence * 0.34, GNSS_OBSERVATION_PULL_MIN, GNSS_OBSERVATION_PULL_MAX);
        double distanceSensitiveBoost = source == ObservationSource.GNSS
                ? clamp(0.08 - accuracyMeters * 0.0025, 0.0, 0.08)
                : clamp(0.06 - accuracyMeters * 0.0020, 0.0, 0.06);
        double pull = clamp(basePull + distanceSensitiveBoost, 0.20, 0.68);
        double jitterStd = clamp(accuracyMeters * 0.12, 0.35, source == ObservationSource.WIFI ? 1.2 : 1.8);

        for (Particle particle : particles) {
            particle.easting = lerp(
                    particle.easting,
                    observedEasting + gaussian(0d, jitterStd),
                    pull
            );
            particle.northing = lerp(
                    particle.northing,
                    observedNorthing + gaussian(0d, jitterStd),
                    pull
            );
        }
    }

    private void initialiseParticles(double accuracyMeters,
                                     double headingRad,
                                     int initialLogicalFloor,
                                     @Nullable Integer floorHint,
                                     double confidence) {
        particles.clear();

        double positionStd = clamp(accuracyMeters * 0.6, 1.5, 6.0);
        double baseHeading = normalizeHeading(headingRad);
        double headingStd = Math.toRadians(Double.isNaN(headingRad) ? 45.0 : 24.0);
        double uniformWeight = 1.0 / PARTICLE_COUNT;
        String buildingId = spatialConstraintModel.getCurrentBuildingId();

        for (int i = 0; i < PARTICLE_COUNT; i++) {
            particles.add(new Particle(
                    gaussian(0d, positionStd),
                    gaussian(0d, positionStd),
                    normalizeRad(baseHeading + gaussian(0d, headingStd)),
                    uniformWeight,
                    sampleInitialFloor(buildingId, initialLogicalFloor, floorHint, confidence)
            ));
        }
    }

    private void predict(double stepDistance, double headingRad) {
        double stepNoiseStd = clamp(0.08 + stepDistance * 0.12, 0.08, 0.22);
        double headingNoiseStd = Math.toRadians(stepDistance < 0.4 ? 14.0 : 9.0);
        String buildingId = spatialConstraintModel.getCurrentBuildingId();

        for (Particle particle : particles) {
            double previousEasting = particle.easting;
            double previousNorthing = particle.northing;
            double noisyStep = Math.max(0d, stepDistance + gaussian(0d, stepNoiseStd));
            double noisyHeading = normalizeRad(headingRad + gaussian(0d, headingNoiseStd));
            double candidateEasting = previousEasting + noisyStep * Math.sin(noisyHeading);
            double candidateNorthing = previousNorthing + noisyStep * Math.cos(noisyHeading);
            double motionPenalty = 1.0;
            particle.logicalFloor = resolveParticleFloor(buildingId, particle.logicalFloor);

            if (reference != null
                    && spatialConstraintModel.hasIndoorContext(buildingId, particle.logicalFloor)) {
                LatLng previousLatLng = reference.toLatLng(previousEasting, previousNorthing);
                LatLng candidateLatLng = reference.toLatLng(candidateEasting, candidateNorthing);
                IndoorSpatialConstraintModel.ConstraintResult constraintResult =
                        spatialConstraintModel.constrainMotion(
                                previousLatLng,
                                candidateLatLng,
                                buildingId,
                                particle.logicalFloor
                        );
                LatLng corrected = constraintResult.getCorrectedPosition();
                if (corrected != null) {
                    double[] correctedLocal = reference.toLocal(corrected);
                    candidateEasting = correctedLocal[0];
                    candidateNorthing = correctedLocal[1];
                } else {
                    candidateEasting = previousEasting;
                    candidateNorthing = previousNorthing;
                }
                motionPenalty = constraintResult.getWeightScale();
            }

            particle.easting = candidateEasting;
            particle.northing = candidateNorthing;
            particle.headingRad = noisyHeading;
            particle.weight *= motionPenalty;
        }

        normalizeWeights();
    }

    private void updateWeights(double observedEasting,
                               double observedNorthing,
                               double accuracyMeters,
                               double confidence,
                               ObservationSource source,
                               @Nullable Integer observedFloor) {
        double sigma = clamp(accuracyMeters / Math.max(confidence, 0.45), 2.5, 18.0);
        double sigmaSquared = sigma * sigma;
        String buildingId = spatialConstraintModel.getCurrentBuildingId();
        Integer clampedObservedFloor = observedFloor == null
                ? null
                : resolveParticleFloor(buildingId, observedFloor);

        for (Particle particle : particles) {
            double deltaE = particle.easting - observedEasting;
            double deltaN = particle.northing - observedNorthing;
            double distanceSquared = deltaE * deltaE + deltaN * deltaN;
            double likelihood = Math.exp(-0.5 * distanceSquared / sigmaSquared);
            double spatialWeight = 1.0;
            double floorWeight = 1.0;
            particle.logicalFloor = resolveParticleFloor(buildingId, particle.logicalFloor);
            if (reference != null
                    && spatialConstraintModel.hasIndoorContext(buildingId, particle.logicalFloor)) {
                spatialWeight = spatialConstraintModel.scorePoint(
                        reference.toLatLng(particle.easting, particle.northing),
                        buildingId,
                        particle.logicalFloor
                );
            }
            if (source == ObservationSource.WIFI && clampedObservedFloor != null) {
                floorWeight = floorObservationLikelihood(clampedObservedFloor, particle.logicalFloor);
            }
            particle.weight *= likelihood * spatialWeight * floorWeight;
        }

        normalizeWeights();
    }

    private double effectiveParticleCount() {
        double sum = 0d;
        for (Particle particle : particles) {
            sum += particle.weight * particle.weight;
        }
        return sum <= 0d ? 0d : 1d / sum;
    }

    private void resample() {
        int count = particles.size();
        double[] cumulative = new double[count];
        cumulative[0] = particles.get(0).weight;
        for (int i = 1; i < count; i++) {
            cumulative[i] = cumulative[i - 1] + particles.get(i).weight;
        }

        ArrayList<Particle> resampled = new ArrayList<>(count);
        double step = 1.0 / count;
        double threshold = random.nextDouble() * step;
        int index = 0;
        String buildingId = spatialConstraintModel.getCurrentBuildingId();

        for (int i = 0; i < count; i++) {
            double sample = threshold + i * step;
            while (index < count - 1 && sample > cumulative[index]) {
                index++;
            }

            Particle selected = particles.get(index);
            int logicalFloor = selected.logicalFloor;
            if (buildingId != null && random.nextDouble() < FLOOR_JITTER_PROBABILITY) {
                logicalFloor += random.nextBoolean() ? 1 : -1;
            }
            logicalFloor = resolveParticleFloor(buildingId, logicalFloor);
            resampled.add(new Particle(
                    selected.easting,
                    selected.northing,
                    selected.headingRad,
                    step,
                    logicalFloor
            ));
        }

        particles.clear();
        particles.addAll(resampled);
    }

    private PoseEstimate estimatePose() {
        double easting = 0d;
        double northing = 0d;
        double sinSum = 0d;
        double cosSum = 0d;

        for (Particle particle : particles) {
            easting += particle.easting * particle.weight;
            northing += particle.northing * particle.weight;
            sinSum += Math.sin(particle.headingRad) * particle.weight;
            cosSum += Math.cos(particle.headingRad) * particle.weight;
        }

        return new PoseEstimate(
                easting,
                northing,
                Math.atan2(sinSum, cosSum),
                estimateLogicalFloor()
        );
    }

    private void commitPose(double estimatedEasting,
                            double estimatedNorthing,
                            double estimatedHeadingRad,
                            int logicalFloor,
                            long timestampMillis,
                            double headingSmoothingAlpha,
                            double positionMeasurementSigmaMeters) {
        rawEasting = estimatedEasting;
        rawNorthing = estimatedNorthing;
        rawHeadingRad = normalizeRad(estimatedHeadingRad);

        if (!hasDisplayPose) {
            displayKalmanFilter.reset(rawEasting, rawNorthing, timestampMillis);
            displayEasting = rawEasting;
            displayNorthing = rawNorthing;
            displayHeadingRad = rawHeadingRad;
            hasDisplayPose = true;
        } else {
            displayKalmanFilter.update(
                    rawEasting,
                    rawNorthing,
                    positionMeasurementSigmaMeters,
                    isRecentPdrMotion(timestampMillis),
                    timestampMillis
            );
            displayEasting = displayKalmanFilter.getEasting();
            displayNorthing = displayKalmanFilter.getNorthing();
            displayHeadingRad = normalizeRad(circleBlend(
                    displayHeadingRad,
                    rawHeadingRad,
                    headingSmoothingAlpha
            ));
        }

        currentLatLng = reference.toLatLng(displayEasting, displayNorthing);
        currentPositionVersion++;
        spatialConstraintModel.updatePosition(currentLatLng);
        dominantParticleFloor = logicalFloor;
        clampParticlesToCurrentContext();
        appendHistory(currentLatLng, timestampMillis, false);
    }

    private void appendHistory(LatLng point, long timestampMillis, boolean forceReplaceLastPoint) {
        if (point == null) {
            return;
        }

        if (forceReplaceLastPoint && !fusedHistory.isEmpty()) {
            fusedHistory.removeLast();
            addHistoryPoint(point, timestampMillis);
            return;
        }

        if (fusedHistory.isEmpty()) {
            addHistoryPoint(point, timestampMillis);
            return;
        }

        LatLng lastPoint = fusedHistory.peekLast();
        double distanceMeters = UtilFunctions.distanceBetweenPoints(lastPoint, point);
        long elapsedMillis = Math.max(0L, timestampMillis - lastHistoryTimestamp);
        boolean recentlyMoving = isRecentPdrMotion(timestampMillis);

        if (!recentlyMoving) {
            if (fusedHistory.size() == 1
                    && elapsedMillis >= MIN_HISTORY_INTERVAL_MS
                    && distanceMeters >= STATIONARY_REBASE_DISTANCE_METERS) {
                fusedHistory.removeLast();
                addHistoryPoint(point, timestampMillis);
            }
            return;
        }

        if (elapsedMillis < MIN_HISTORY_INTERVAL_MS
                || distanceMeters < MIN_HISTORY_DISTANCE_METERS
                || distanceMeters > MAX_HISTORY_SEGMENT_METERS) {
            return;
        }

        addHistoryPoint(point, timestampMillis);
    }

    private void addHistoryPoint(LatLng point, long timestampMillis) {
        fusedHistory.addLast(point);
        while (fusedHistory.size() > MAX_HISTORY_SIZE) {
            fusedHistory.removeFirst();
        }
        lastHistoryTimestamp = timestampMillis;
        fusedHistoryVersion++;
    }

    private boolean appendTail(ArrayDeque<LatLng> tail, LatLng point) {
        boolean changed = false;
        LatLng last = tail.peekLast();
        if (last != null && UtilFunctions.distanceBetweenPoints(last, point) < 0.75) {
            tail.removeLast();
            changed = true;
        }
        tail.addLast(point);
        changed = true;
        while (tail.size() > MAX_TAIL_SIZE) {
            tail.removeFirst();
        }
        return changed;
    }

    private boolean isRecentPdrMotion(long timestampMillis) {
        return lastPdrMotionTimestamp > 0L
                && timestampMillis - lastPdrMotionTimestamp <= HISTORY_ACTIVE_MOTION_WINDOW_MS;
    }

    private double normalizeHeading(double headingRad) {
        return Double.isNaN(headingRad) ? 0d : normalizeRad(headingRad);
    }

    private double circleBlend(double from, double to, double alpha) {
        double delta = normalizeRad(to - from);
        return from + alpha * delta;
    }

    private double gaussian(double mean, double std) {
        return mean + random.nextGaussian() * std;
    }

    private double lerp(double from, double to, double alpha) {
        return from + alpha * (to - from);
    }

    private int sampleInitialFloor(@Nullable String buildingId,
                                   int initialLogicalFloor,
                                   @Nullable Integer floorHint,
                                   double confidence) {
        int seedFloor = resolveParticleFloor(buildingId, initialLogicalFloor);
        if (buildingId == null || floorHint == null) {
            return seedFloor;
        }

        int hintedFloor = resolveParticleFloor(buildingId, floorHint);
        double primaryProbability = clamp(0.58 + confidence * 0.24, 0.65, 0.82);
        double draw = random.nextDouble();
        if (draw < primaryProbability) {
            return hintedFloor;
        }
        int adjacentOffset = draw < primaryProbability + (1d - primaryProbability) / 2d ? -1 : 1;
        return resolveParticleFloor(buildingId, hintedFloor + adjacentOffset);
    }

    private void injectFloorHypotheses(int observedFloor, double confidence) {
        if (particles.isEmpty()) {
            return;
        }

        String buildingId = spatialConstraintModel.getCurrentBuildingId();
        if (buildingId == null) {
            return;
        }

        int mutationCount = Math.max(
                12,
                (int) Math.round(particles.size() * clamp(0.10 + confidence * 0.10, 0.10, 0.22))
        );
        for (int i = 0; i < mutationCount; i++) {
            Particle particle = particles.get(random.nextInt(particles.size()));
            particle.logicalFloor = sampleInitialFloor(
                    buildingId,
                    particle.logicalFloor,
                    observedFloor,
                    confidence
            );
        }
    }

    private int estimateLogicalFloor() {
        if (particles.isEmpty()) {
            return spatialConstraintModel.getCurrentLogicalFloor();
        }

        String buildingId = spatialConstraintModel.getCurrentBuildingId();
        Map<Integer, Double> floorWeights = new HashMap<>();
        for (Particle particle : particles) {
            int floor = resolveParticleFloor(buildingId, particle.logicalFloor);
            floorWeights.put(floor, floorWeights.getOrDefault(floor, 0d) + particle.weight);
        }

        int fallbackFloor = spatialConstraintModel.getCurrentLogicalFloor();
        double fallbackWeight = floorWeights.getOrDefault(fallbackFloor, 0d);
        int bestFloor = fallbackFloor;
        double bestWeight = fallbackWeight;

        for (Map.Entry<Integer, Double> entry : floorWeights.entrySet()) {
            if (entry.getValue() > bestWeight) {
                bestFloor = entry.getKey();
                bestWeight = entry.getValue();
            }
        }

        if (bestFloor != fallbackFloor
                && (bestWeight < FLOOR_MODE_SWITCH_DOMINANCE
                || bestWeight - fallbackWeight < FLOOR_MODE_SWITCH_MARGIN)) {
            return fallbackFloor;
        }
        return bestFloor;
    }

    private void synchronizeParticlesToResolvedFloor(int logicalFloor) {
        String buildingId = spatialConstraintModel.getCurrentBuildingId();
        int resolvedFloor = resolveParticleFloor(buildingId, logicalFloor);
        for (Particle particle : particles) {
            particle.logicalFloor = resolvedFloor;
        }
    }

    private void clampParticlesToCurrentContext() {
        String buildingId = spatialConstraintModel.getCurrentBuildingId();
        if (buildingId == null) {
            return;
        }
        for (Particle particle : particles) {
            particle.logicalFloor = resolveParticleFloor(buildingId, particle.logicalFloor);
        }
    }

    private int resolveParticleFloor(@Nullable String buildingId, int logicalFloor) {
        return buildingId == null
                ? logicalFloor
                : spatialConstraintModel.clampLogicalFloor(buildingId, logicalFloor);
    }

    private double floorObservationLikelihood(int observedFloor, int particleFloor) {
        double floorDifference = observedFloor - particleFloor;
        double sigmaSquared = WIFI_FLOOR_SIGMA * WIFI_FLOOR_SIGMA;
        return Math.max(0.02, Math.exp(-0.5 * floorDifference * floorDifference / sigmaSquared));
    }

    private void normalizeWeights() {
        if (particles.isEmpty()) {
            return;
        }

        double weightSum = 0d;
        for (Particle particle : particles) {
            weightSum += particle.weight;
        }

        if (weightSum <= 0d || Double.isNaN(weightSum) || Double.isInfinite(weightSum)) {
            double uniformWeight = 1.0 / particles.size();
            for (Particle particle : particles) {
                particle.weight = uniformWeight;
            }
            return;
        }

        for (Particle particle : particles) {
            particle.weight /= weightSum;
        }
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double normalizeRad(double radians) {
        double value = radians;
        while (value > Math.PI) {
            value -= 2d * Math.PI;
        }
        while (value < -Math.PI) {
            value += 2d * Math.PI;
        }
        return value;
    }

    private static final class Particle {
        private double easting;
        private double northing;
        private double headingRad;
        private double weight;
        private int logicalFloor;

        private Particle(double easting,
                         double northing,
                         double headingRad,
                         double weight,
                         int logicalFloor) {
            this.easting = easting;
            this.northing = northing;
            this.headingRad = headingRad;
            this.weight = weight;
            this.logicalFloor = logicalFloor;
        }
    }

    private static final class PoseEstimate {
        private final double easting;
        private final double northing;
        private final double headingRad;
        private final int logicalFloor;

        private PoseEstimate(double easting,
                             double northing,
                             double headingRad,
                             int logicalFloor) {
            this.easting = easting;
            this.northing = northing;
            this.headingRad = headingRad;
            this.logicalFloor = logicalFloor;
        }
    }

    /**
     * Lightweight local tangent-plane converter centred at the initial reliable fix.
     */
    private static final class EnuReference {
        private final LatLng origin;
        private final double originLatRad;
        private final double originLonRad;
        private final double cosOriginLat;

        private EnuReference(LatLng origin) {
            this.origin = origin;
            this.originLatRad = Math.toRadians(origin.latitude);
            this.originLonRad = Math.toRadians(origin.longitude);
            this.cosOriginLat = Math.cos(originLatRad);
        }

        private double[] toLocal(LatLng latLng) {
            double latRad = Math.toRadians(latLng.latitude);
            double lonRad = Math.toRadians(latLng.longitude);
            double east = (lonRad - originLonRad) * cosOriginLat * EARTH_RADIUS_METERS;
            double north = (latRad - originLatRad) * EARTH_RADIUS_METERS;
            return new double[]{east, north};
        }

        private LatLng toLatLng(double easting, double northing) {
            double lat = Math.toDegrees(originLatRad + northing / EARTH_RADIUS_METERS);
            double lon = Math.toDegrees(originLonRad + easting / (EARTH_RADIUS_METERS * cosOriginLat));
            return new LatLng(lat, lon);
        }
    }
}
