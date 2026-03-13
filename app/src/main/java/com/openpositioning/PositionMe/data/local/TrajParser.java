package com.openpositioning.PositionMe.data.local;

import android.content.Context;
import android.hardware.SensorManager;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.google.android.gms.maps.model.LatLng;
import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonParser;
import com.google.gson.JsonObject;
import com.openpositioning.PositionMe.presentation.fragment.ReplayFragment;
import com.openpositioning.PositionMe.sensors.SensorFusion;
import com.openpositioning.PositionMe.utils.CoordinateConverter;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Handles parsing of trajectory data stored in JSON files, combining IMU, PDR, and GNSS data
 * to reconstruct motion paths.
 *
 * <p>
 * The **TrajParser** is primarily responsible for processing recorded trajectory data and
 * reconstructing motion information, including estimated positions, GNSS coordinates, speed, and orientation.
 * It does this by reading a JSON file containing:
 * </p>
 * <ul>
 *     <li>IMU (Inertial Measurement Unit) data</li>
 *     <li>PDR (Pedestrian Dead Reckoning) position data</li>
 *     <li>GNSS (Global Navigation Satellite System) location data</li>
 * </ul>
 *
 * <p>
 * **Usage in Module 'PositionMe.app.main':**
 * </p>
 * <ul>
 *     <li>**ReplayFragment** - Calls `parseTrajectoryData()` to read recorded trajectory files and process movement.</li>
 *     <li>Stores parsed trajectory data as `ReplayPoint` objects.</li>
 *     <li>Provides data for updating map visualizations in `ReplayFragment`.</li>
 * </ul>
 *
 * @see ReplayFragment which uses parsed trajectory data for visualization.
 * @see SensorFusion for motion processing and sensor integration.
 * @see com.openpositioning.PositionMe.presentation.fragment.ReplayFragment for implementation details.
 *
 * @author Shu Gu
 * @author Lin Cheng
 */
public class TrajParser {

    private static final String TAG = "TrajParser";

    /**
     * Represents a single replay point containing estimated PDR position, GNSS location,
     * orientation, speed, and timestamp.
     */
    public static class ReplayPoint {
        public LatLng trackLocation;  // Fused track estimate (or legacy PDR fallback)
        public LatLng gnssLocation; // GNSS location (may be null if unavailable)
        public float orientation;   // Orientation in degrees
        public float speed;         // Speed in meters per second
        public long timestamp;      // Relative timestamp
        public int floor;          // Estimated floor relative to the starting floor
        public boolean elevator;   // Elevator detection flag
        public float elevation;    // Relative elevation in metres

        /**
         * Constructs a ReplayPoint.
         *
         * @param trackLocation The fused or fallback track location.
         * @param gnssLocation The GNSS location, or null if unavailable.
         * @param orientation  The orientation angle in degrees.
         * @param speed        The speed in meters per second.
         * @param timestamp    The timestamp associated with this point.
         * @param floor        The estimated floor.
         * @param elevator     The elevator detection state.
         * @param elevation    The relative elevation in metres.
         */
        public ReplayPoint(
                LatLng trackLocation,
                LatLng gnssLocation,
                float orientation,
                float speed,
                long timestamp,
                int floor,
                boolean elevator,
                float elevation
        ) {
            this.trackLocation = trackLocation;
            this.gnssLocation = gnssLocation;
            this.orientation = orientation;
            this.speed = speed;
            this.timestamp = timestamp;
            this.floor = floor;
            this.elevator = elevator;
            this.elevation = elevation;
        }
    }

    public static class ReplayTestPoint {
        public final LatLng position;
        public final int index;
        public final int floor;

        public ReplayTestPoint(LatLng position, int index, int floor) {
            this.position = position;
            this.index = index;
            this.floor = floor;
        }
    }

    public enum ReplayInitializationSource {
        FUSED_POSE,
        ABSOLUTE_FIX,
        LEGACY_FALLBACK,
        UNAVAILABLE
    }

    public static final class ReplayInitialization {
        @Nullable
        public final LatLng origin;
        public final boolean useFusedPose;
        @NonNull
        public final ReplayInitializationSource source;

        public ReplayInitialization(
                @Nullable LatLng origin,
                boolean useFusedPose,
                @NonNull ReplayInitializationSource source
        ) {
            this.origin = origin;
            this.useFusedPose = useFusedPose;
            this.source = source;
        }

        public boolean hasOrigin() {
            return origin != null;
        }

        public static ReplayInitialization unavailable(boolean useFusedPose) {
            return new ReplayInitialization(null, useFusedPose, ReplayInitializationSource.UNAVAILABLE);
        }
    }

    private static final class TimestampedLatLng {
        private final long relativeTimestamp;
        private final LatLng latLng;

        private TimestampedLatLng(long relativeTimestamp, @NonNull LatLng latLng) {
            this.relativeTimestamp = relativeTimestamp;
            this.latLng = latLng;
        }
    }

    /** Represents an IMU (Inertial Measurement Unit) data record used for orientation calculations. */
    private static class ImuRecord {
        public long relativeTimestamp;
        public float accX, accY, accZ; // Accelerometer values
        public float gyrX, gyrY, gyrZ; // Gyroscope values
        public float rotationVectorX, rotationVectorY, rotationVectorZ, rotationVectorW; // Rotation quaternion
    }

    /** Represents a Pedestrian Dead Reckoning (PDR) data record storing position shifts over time. */
    private static class PdrRecord {
        public long relativeTimestamp;
        public float x, y; // Position relative to the starting point
        public int floor;
        public boolean elevator;
        public float elevation;
    }

    private static class FusedPoseRecord {
        public long relativeTimestamp;
        public double x, y;
        public int floor;
        public float confidence;
    }

    /** Represents a GNSS (Global Navigation Satellite System) data record with latitude/longitude. */
    private static class GnssRecord {
        public long relativeTimestamp;
        public double latitude, longitude; // GNSS coordinates
        public double altitude;
        public String floor;
    }

    private static class TestPointRecord {
        public int index;
        public GnssRecord position;
    }

    /**
     * Parses trajectory data from a JSON file and reconstructs a list of replay points.
     *
     * <p>
     * This method processes a trajectory log file, extracting IMU, PDR, and GNSS records,
     * and uses them to generate **ReplayPoint** objects. Each point contains:
     * </p>
     * <ul>
     *     <li>Estimated PDR-based position.</li>
     *     <li>GNSS location (if available).</li>
     *     <li>Computed orientation using rotation vectors.</li>
     *     <li>Speed estimation based on movement data.</li>
     * </ul>
     *
     * @param filePath  Path to the JSON file containing trajectory data.
     * @param context   Android application context (used for sensor processing).
     * @param originLat Latitude of the reference origin.
     * @param originLng Longitude of the reference origin.
     * @return A list of parsed {@link ReplayPoint} objects.
     */
    public static List<ReplayPoint> parseTrajectoryData(String filePath, Context context,
                                                        double originLat, double originLng) {
        List<ReplayPoint> result = new ArrayList<>();

        try {
            JsonObject root = readRootObject(filePath);
            if (root == null) {
                return result;
            }

            Log.i(TAG, "Successfully read trajectory file: " + filePath);

            List<ImuRecord> imuList = parseImuData(root.getAsJsonArray("imuData"));
            List<PdrRecord> pdrList = parsePdrData(root.getAsJsonArray("pdrData"));
            List<GnssRecord> gnssList = parseGnssData(root.getAsJsonArray("gnssData"));
            List<FusedPoseRecord> fusedPoseList = parseFusedPoseData(root.getAsJsonArray("fusedPose"));
            CoordinateConverter coordinateConverter = new CoordinateConverter(originLat, originLng);

            Log.i(TAG, "Parsed data - IMU: " + imuList.size() + " records, PDR: "
                    + pdrList.size() + " records, GNSS: " + gnssList.size() + " records, Fused: "
                    + fusedPoseList.size() + " records");

            if (!fusedPoseList.isEmpty()) {
                for (int i = 0; i < fusedPoseList.size(); i++) {
                    FusedPoseRecord fusedPose = fusedPoseList.get(i);

                    ImuRecord closestImu = findClosestImuRecord(imuList, fusedPose.relativeTimestamp);
                    float orientationDeg = closestImu != null ? computeOrientationFromRotationVector(
                            closestImu.rotationVectorX,
                            closestImu.rotationVectorY,
                            closestImu.rotationVectorZ,
                            closestImu.rotationVectorW,
                            context
                    ) : 0f;

                    float speed = 0f;
                    if (i > 0) {
                        FusedPoseRecord previous = fusedPoseList.get(i - 1);
                        double dt = (fusedPose.relativeTimestamp - previous.relativeTimestamp) / 1000.0;
                        double dx = fusedPose.x - previous.x;
                        double dy = fusedPose.y - previous.y;
                        double distance = Math.sqrt(dx * dx + dy * dy);
                        if (dt > 0) {
                            speed = (float) (distance / dt);
                        }
                    }

                    LatLng trackLocation = coordinateConverter.toLatLng(fusedPose.x, fusedPose.y);
                    GnssRecord closestGnss = findClosestGnssRecord(gnssList, fusedPose.relativeTimestamp);
                    LatLng gnssLocation = closestGnss != null
                            ? new LatLng(closestGnss.latitude, closestGnss.longitude)
                            : null;
                    PdrRecord closestPdr = findClosestPdrRecord(pdrList, fusedPose.relativeTimestamp);
                    boolean elevator = closestPdr != null && closestPdr.elevator;
                    float elevation = closestPdr != null ? closestPdr.elevation : 0f;

                    result.add(new ReplayPoint(
                            trackLocation,
                            gnssLocation,
                            orientationDeg,
                            speed,
                            fusedPose.relativeTimestamp,
                            fusedPose.floor,
                            elevator,
                            elevation
                    ));
                }
            } else {
                for (int i = 0; i < pdrList.size(); i++) {
                    PdrRecord pdr = pdrList.get(i);

                    ImuRecord closestImu = findClosestImuRecord(imuList, pdr.relativeTimestamp);
                    float orientationDeg = closestImu != null ? computeOrientationFromRotationVector(
                            closestImu.rotationVectorX,
                            closestImu.rotationVectorY,
                            closestImu.rotationVectorZ,
                            closestImu.rotationVectorW,
                            context
                    ) : 0f;

                    float speed = 0f;
                    if (i > 0) {
                        PdrRecord prev = pdrList.get(i - 1);
                        double dt = (pdr.relativeTimestamp - prev.relativeTimestamp) / 1000.0;
                        double dx = pdr.x - prev.x;
                        double dy = pdr.y - prev.y;
                        double distance = Math.sqrt(dx * dx + dy * dy);
                        if (dt > 0) speed = (float) (distance / dt);
                    }

                    LatLng trackLocation = coordinateConverter.toLatLng(pdr.x, pdr.y);
                    GnssRecord closestGnss = findClosestGnssRecord(gnssList, pdr.relativeTimestamp);
                    LatLng gnssLocation = closestGnss != null ?
                            new LatLng(closestGnss.latitude, closestGnss.longitude) : null;

                    result.add(new ReplayPoint(
                            trackLocation,
                            gnssLocation,
                            orientationDeg,
                            speed,
                            pdr.relativeTimestamp,
                            pdr.floor,
                            pdr.elevator,
                            pdr.elevation
                    ));
                }
            }

            Collections.sort(result, Comparator.comparingLong(rp -> rp.timestamp));

            Log.i(TAG, "Final ReplayPoints count: " + result.size());

        } catch (Exception e) {
            Log.e(TAG, "Error parsing trajectory file!", e);
        }

        return result;
    }

    public static List<ReplayPoint> parseTrajectoryData(
            String filePath,
            Context context,
            @Nullable ReplayInitialization replayInitialization
    ) {
        List<ReplayPoint> result = new ArrayList<>();

        try {
            JsonObject root = readRootObject(filePath);
            if (root == null) {
                return result;
            }

            ReplayInitialization effectiveInitialization = replayInitialization != null
                    ? replayInitialization
                    : resolveReplayInitialization(root);
            if (!effectiveInitialization.hasOrigin()) {
                Log.w(TAG, "No replay origin found in file. Replay data cannot be reconstructed.");
                return result;
            }

            LatLng origin = effectiveInitialization.origin;
            CoordinateConverter coordinateConverter =
                    new CoordinateConverter(origin.latitude, origin.longitude);

            Log.i(TAG, "Successfully read trajectory file: " + filePath);
            Log.i(TAG, "Replay initialization source: " + effectiveInitialization.source
                    + ", useFusedPose=" + effectiveInitialization.useFusedPose
                    + ", origin=" + origin);

            List<ImuRecord> imuList = parseImuData(getArray(root, "imuData", "imu_data"));
            List<PdrRecord> pdrList = parsePdrData(getArray(root, "pdrData", "pdr_data"));
            List<GnssRecord> gnssList = parseGnssData(getArray(root, "gnssData", "gnss_data"));
            List<FusedPoseRecord> fusedPoseList =
                    parseFusedPoseData(getArray(root, "fusedPose", "fused_pose"));

            Log.i(TAG, "Parsed data - IMU: " + imuList.size() + " records, PDR: "
                    + pdrList.size() + " records, GNSS: " + gnssList.size() + " records, Fused: "
                    + fusedPoseList.size() + " records");

            boolean useFusedPose = effectiveInitialization.useFusedPose && !fusedPoseList.isEmpty();
            if (useFusedPose) {
                for (int i = 0; i < fusedPoseList.size(); i++) {
                    FusedPoseRecord fusedPose = fusedPoseList.get(i);

                    ImuRecord closestImu = findClosestImuRecord(imuList, fusedPose.relativeTimestamp);
                    float orientationDeg = closestImu != null ? computeOrientationFromRotationVector(
                            closestImu.rotationVectorX,
                            closestImu.rotationVectorY,
                            closestImu.rotationVectorZ,
                            closestImu.rotationVectorW,
                            context
                    ) : 0f;

                    float speed = 0f;
                    if (i > 0) {
                        FusedPoseRecord previous = fusedPoseList.get(i - 1);
                        double dt = (fusedPose.relativeTimestamp - previous.relativeTimestamp) / 1000.0;
                        double dx = fusedPose.x - previous.x;
                        double dy = fusedPose.y - previous.y;
                        double distance = Math.sqrt(dx * dx + dy * dy);
                        if (dt > 0) {
                            speed = (float) (distance / dt);
                        }
                    }

                    LatLng trackLocation = coordinateConverter.toLatLng(fusedPose.x, fusedPose.y);
                    GnssRecord closestGnss = findClosestGnssRecord(gnssList, fusedPose.relativeTimestamp);
                    LatLng gnssLocation = closestGnss != null
                            ? new LatLng(closestGnss.latitude, closestGnss.longitude)
                            : null;
                    PdrRecord closestPdr = findClosestPdrRecord(pdrList, fusedPose.relativeTimestamp);
                    boolean elevator = closestPdr != null && closestPdr.elevator;
                    float elevation = closestPdr != null ? closestPdr.elevation : 0f;

                    result.add(new ReplayPoint(
                            trackLocation,
                            gnssLocation,
                            orientationDeg,
                            speed,
                            fusedPose.relativeTimestamp,
                            fusedPose.floor,
                            elevator,
                            elevation
                    ));
                }
            } else {
                for (int i = 0; i < pdrList.size(); i++) {
                    PdrRecord pdr = pdrList.get(i);

                    ImuRecord closestImu = findClosestImuRecord(imuList, pdr.relativeTimestamp);
                    float orientationDeg = closestImu != null ? computeOrientationFromRotationVector(
                            closestImu.rotationVectorX,
                            closestImu.rotationVectorY,
                            closestImu.rotationVectorZ,
                            closestImu.rotationVectorW,
                            context
                    ) : 0f;

                    float speed = 0f;
                    if (i > 0) {
                        PdrRecord prev = pdrList.get(i - 1);
                        double dt = (pdr.relativeTimestamp - prev.relativeTimestamp) / 1000.0;
                        double dx = pdr.x - prev.x;
                        double dy = pdr.y - prev.y;
                        double distance = Math.sqrt(dx * dx + dy * dy);
                        if (dt > 0) {
                            speed = (float) (distance / dt);
                        }
                    }

                    LatLng trackLocation = coordinateConverter.toLatLng(pdr.x, pdr.y);
                    GnssRecord closestGnss = findClosestGnssRecord(gnssList, pdr.relativeTimestamp);
                    LatLng gnssLocation = closestGnss != null
                            ? new LatLng(closestGnss.latitude, closestGnss.longitude)
                            : null;

                    result.add(new ReplayPoint(
                            trackLocation,
                            gnssLocation,
                            orientationDeg,
                            speed,
                            pdr.relativeTimestamp,
                            pdr.floor,
                            pdr.elevator,
                            pdr.elevation
                    ));
                }
            }

            Collections.sort(result, Comparator.comparingLong(rp -> rp.timestamp));
            Log.i(TAG, "Final ReplayPoints count: " + result.size());
        } catch (Exception e) {
            Log.e(TAG, "Error parsing trajectory file!", e);
        }

        return result;
    }

    public static ReplayInitialization resolveReplayInitialization(String filePath) {
        try {
            JsonObject root = readRootObject(filePath);
            if (root == null) {
                return ReplayInitialization.unavailable(false);
            }
            return resolveReplayInitialization(root);
        } catch (IOException e) {
            Log.e(TAG, "Failed to resolve replay initialization for file: " + filePath, e);
            return ReplayInitialization.unavailable(false);
        }
    }

    static ReplayInitialization resolveReplayInitialization(@Nullable JsonObject root) {
        if (root == null) {
            return ReplayInitialization.unavailable(false);
        }

        boolean hasFusedPose = hasReplayRecords(getArray(root, "fusedPose", "fused_pose"));
        TimestampedLatLng firstAbsoluteFix = findFirstAbsoluteFix(root);
        LatLng legacyInitialPosition = findLegacyInitialPosition(root);

        if (hasFusedPose) {
            if (firstAbsoluteFix != null) {
                return new ReplayInitialization(
                        firstAbsoluteFix.latLng,
                        true,
                        ReplayInitializationSource.FUSED_POSE
                );
            }
            if (legacyInitialPosition != null) {
                // Legacy manual-start trajectories may contain fused_pose but no recorded
                // absolute fix. We keep this fallback for backwards compatibility only.
                return new ReplayInitialization(
                        legacyInitialPosition,
                        true,
                        ReplayInitializationSource.LEGACY_FALLBACK
                );
            }
            return ReplayInitialization.unavailable(true);
        }

        if (firstAbsoluteFix != null) {
            return new ReplayInitialization(
                    firstAbsoluteFix.latLng,
                    false,
                    ReplayInitializationSource.ABSOLUTE_FIX
            );
        }
        if (legacyInitialPosition != null) {
            // Legacy PDR-only trajectories may still rely on initialPosition/manual start.
            // This fallback is intentionally last and is not part of the formal fusion path.
            return new ReplayInitialization(
                    legacyInitialPosition,
                    false,
                    ReplayInitializationSource.LEGACY_FALLBACK
            );
        }
        return ReplayInitialization.unavailable(false);
    }

    public static List<ReplayTestPoint> parseTestPoints(String filePath) {
        List<ReplayTestPoint> result = new ArrayList<>();
        try {
            JsonObject root = readRootObject(filePath);
            if (root == null) {
                return result;
            }

            JsonArray testPointArray = getArray(root, "testPoints", "test_points");
            if (testPointArray == null) {
                JsonArray legacyArray = getArray(root, "legacyTestPoints", "legacy_test_points");
                if (legacyArray == null) {
                    return result;
                }
                for (int i = 0; i < legacyArray.size(); i++) {
                    JsonObject legacyPoint = legacyArray.get(i).getAsJsonObject();
                    if (legacyPoint == null
                            || !legacyPoint.has("latitude")
                            || !legacyPoint.has("longitude")) {
                        continue;
                    }
                    LatLng latLng = new LatLng(
                            legacyPoint.get("latitude").getAsDouble(),
                            legacyPoint.get("longitude").getAsDouble()
                    );
                    int floor = parseFloor(legacyPoint.has("floor")
                            ? legacyPoint.get("floor").getAsString()
                            : null);
                    result.add(new ReplayTestPoint(latLng, i + 1, floor));
                }
                return result;
            }

            Gson gson = new Gson();
            for (int i = 0; i < testPointArray.size(); i++) {
                TestPointRecord record = gson.fromJson(testPointArray.get(i), TestPointRecord.class);
                if (record == null || record.position == null) {
                    continue;
                }
                LatLng latLng = new LatLng(record.position.latitude, record.position.longitude);
                int floor = parseFloor(record.position.floor);
                result.add(new ReplayTestPoint(latLng, record.index, floor));
            }
        } catch (Exception e) {
            Log.e(TAG, "Error parsing test points!", e);
        }
        return result;
    }

    private static JsonObject readRootObject(String filePath) throws IOException {
        File file = new File(filePath);
        if (!file.exists()) {
            Log.e(TAG, "File does NOT exist: " + filePath);
            return null;
        }
        if (!file.canRead()) {
            Log.e(TAG, "File is NOT readable: " + filePath);
            return null;
        }

        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            return JsonParser.parseReader(br).getAsJsonObject();
        }
    }

    private static int parseFloor(String floorValue) {
        if (floorValue == null || floorValue.trim().isEmpty()) {
            return 0;
        }
        try {
            return Integer.parseInt(floorValue.trim());
        } catch (NumberFormatException ignored) {
            return 0;
        }
    }

    /** Parses IMU data from JSON. */
    private static List<ImuRecord> parseImuData(JsonArray imuArray) {
        List<ImuRecord> imuList = new ArrayList<>();
        if (imuArray == null) {
            return imuList;
        }
        Gson gson = new Gson();
        for (int i = 0; i < imuArray.size(); i++) {
            ImuRecord record = gson.fromJson(imuArray.get(i), ImuRecord.class);
            imuList.add(record);
        }
        return imuList;
    }

    /** Parses PDR data from JSON. */
    private static List<PdrRecord> parsePdrData(JsonArray pdrArray) {
        List<PdrRecord> pdrList = new ArrayList<>();
        if (pdrArray == null) {
            return pdrList;
        }
        Gson gson = new Gson();
        for (int i = 0; i < pdrArray.size(); i++) {
            PdrRecord record = gson.fromJson(pdrArray.get(i), PdrRecord.class);
            pdrList.add(record);
        }
        return pdrList;
    }

    private static List<FusedPoseRecord> parseFusedPoseData(JsonArray fusedPoseArray) {
        List<FusedPoseRecord> fusedPoseList = new ArrayList<>();
        if (fusedPoseArray == null) {
            return fusedPoseList;
        }
        Gson gson = new Gson();
        for (int i = 0; i < fusedPoseArray.size(); i++) {
            FusedPoseRecord record = gson.fromJson(fusedPoseArray.get(i), FusedPoseRecord.class);
            if (record != null) {
                fusedPoseList.add(record);
            }
        }
        return fusedPoseList;
    }

    private static boolean hasReplayRecords(@Nullable JsonArray array) {
        return array != null && array.size() > 0;
    }

    @Nullable
    private static TimestampedLatLng findFirstAbsoluteFix(@NonNull JsonObject root) {
        TimestampedLatLng firstGnssFix = findFirstGnssFix(getArray(root, "gnssData", "gnss_data"));
        TimestampedLatLng firstWifiFix =
                findFirstWifiFix(getArray(root, "wifiFingerprints", "wifi_fingerprints"));
        return earlierOf(firstGnssFix, firstWifiFix);
    }

    @Nullable
    private static TimestampedLatLng findFirstGnssFix(@Nullable JsonArray gnssArray) {
        TimestampedLatLng earliest = null;
        if (gnssArray == null) {
            return null;
        }
        for (int i = 0; i < gnssArray.size(); i++) {
            JsonObject reading = gnssArray.get(i).getAsJsonObject();
            JsonObject position = getObject(reading, "position");
            TimestampedLatLng candidate = parseTimestampedLatLng(
                    position,
                    getLong(reading, -1L, "relativeTimestamp", "relative_timestamp")
            );
            earliest = earlierOf(earliest, candidate);
        }
        return earliest;
    }

    @Nullable
    private static TimestampedLatLng findFirstWifiFix(@Nullable JsonArray wifiFingerprintsArray) {
        TimestampedLatLng earliest = null;
        if (wifiFingerprintsArray == null) {
            return null;
        }
        for (int i = 0; i < wifiFingerprintsArray.size(); i++) {
            JsonObject fingerprint = wifiFingerprintsArray.get(i).getAsJsonObject();
            long fingerprintTimestamp = getLong(
                    fingerprint,
                    -1L,
                    "relativeTimestamp",
                    "relative_timestamp"
            );
            JsonArray rfScans = getArray(fingerprint, "rfScans", "rf_scans");
            if (rfScans == null) {
                continue;
            }
            for (int j = 0; j < rfScans.size(); j++) {
                JsonObject rfScan = rfScans.get(j).getAsJsonObject();
                JsonObject position = getObject(rfScan, "position");
                TimestampedLatLng candidate = parseTimestampedLatLng(
                        position,
                        getLong(rfScan, fingerprintTimestamp, "relativeTimestamp", "relative_timestamp")
                );
                earliest = earlierOf(earliest, candidate);
            }
        }
        return earliest;
    }

    @Nullable
    private static LatLng findLegacyInitialPosition(@NonNull JsonObject root) {
        JsonObject initialPosition = getObject(root, "initialPosition", "initial_position");
        TimestampedLatLng parsed = parseTimestampedLatLng(initialPosition, -1L);
        return parsed == null ? null : parsed.latLng;
    }

    @Nullable
    private static TimestampedLatLng parseTimestampedLatLng(
            @Nullable JsonObject positionObject,
            long fallbackTimestamp
    ) {
        if (positionObject == null) {
            return null;
        }
        Double latitude = getDouble(positionObject, "latitude");
        Double longitude = getDouble(positionObject, "longitude");
        if (latitude == null || longitude == null) {
            return null;
        }
        long relativeTimestamp = getLong(
                positionObject,
                fallbackTimestamp,
                "relativeTimestamp",
                "relative_timestamp"
        );
        return new TimestampedLatLng(relativeTimestamp, new LatLng(latitude, longitude));
    }

    @Nullable
    private static TimestampedLatLng earlierOf(
            @Nullable TimestampedLatLng first,
            @Nullable TimestampedLatLng second
    ) {
        if (first == null) {
            return second;
        }
        if (second == null) {
            return first;
        }
        return second.relativeTimestamp < first.relativeTimestamp ? second : first;
    }

    @Nullable
    private static JsonArray getArray(@Nullable JsonObject object, String... keys) {
        if (object == null) {
            return null;
        }
        for (String key : keys) {
            if (object.has(key) && object.get(key).isJsonArray()) {
                return object.getAsJsonArray(key);
            }
        }
        return null;
    }

    @Nullable
    private static JsonObject getObject(@Nullable JsonObject object, String... keys) {
        if (object == null) {
            return null;
        }
        for (String key : keys) {
            if (object.has(key) && object.get(key).isJsonObject()) {
                return object.getAsJsonObject(key);
            }
        }
        return null;
    }

    private static long getLong(@Nullable JsonObject object, long defaultValue, String... keys) {
        if (object == null) {
            return defaultValue;
        }
        for (String key : keys) {
            if (object.has(key) && object.get(key).isJsonPrimitive()) {
                try {
                    return object.get(key).getAsLong();
                } catch (NumberFormatException ignored) {
                    return defaultValue;
                }
            }
        }
        return defaultValue;
    }

    @Nullable
    private static Double getDouble(@Nullable JsonObject object, String... keys) {
        if (object == null) {
            return null;
        }
        for (String key : keys) {
            if (object.has(key) && object.get(key).isJsonPrimitive()) {
                try {
                    return object.get(key).getAsDouble();
                } catch (NumberFormatException ignored) {
                    return null;
                }
            }
        }
        return null;
    }

    /** Parses GNSS data from JSON. */
    private static List<GnssRecord> parseGnssData(JsonArray gnssArray) {
        List<GnssRecord> gnssList = new ArrayList<>();
        if (gnssArray == null) {
            return gnssList;
        }
        Gson gson = new Gson();
        for (int i = 0; i < gnssArray.size(); i++) {
            GnssRecord record = gson.fromJson(gnssArray.get(i), GnssRecord.class);
            gnssList.add(record);
        }
        return gnssList;
    }

    /** Finds the closest IMU record to the given timestamp. */
    private static ImuRecord findClosestImuRecord(List<ImuRecord> imuList, long targetTimestamp) {
        return imuList.stream()
                .min(Comparator.comparingLong(imu -> Math.abs(imu.relativeTimestamp - targetTimestamp)))
                .orElse(null);
    }

    /** Finds the closest GNSS record to the given timestamp. */
    private static GnssRecord findClosestGnssRecord(List<GnssRecord> gnssList, long targetTimestamp) {
        return gnssList.stream()
                .min(Comparator.comparingLong(gnss -> Math.abs(gnss.relativeTimestamp - targetTimestamp)))
                .orElse(null);
    }

    private static PdrRecord findClosestPdrRecord(List<PdrRecord> pdrList, long targetTimestamp) {
        return pdrList.stream()
                .min(Comparator.comparingLong(pdr -> Math.abs(pdr.relativeTimestamp - targetTimestamp)))
                .orElse(null);
    }

    /** Computes the orientation from a rotation vector. */
    private static float computeOrientationFromRotationVector(
            float rx,
            float ry,
            float rz,
            float rw,
            Context context
    ) {
        float[] rotationVector = new float[]{rx, ry, rz, rw};
        float[] rotationMatrix = new float[9];
        float[] orientationAngles = new float[3];

        SensorManager.getRotationMatrixFromVector(rotationMatrix, rotationVector);
        SensorManager.getOrientation(rotationMatrix, orientationAngles);

        float azimuthDeg = (float) Math.toDegrees(orientationAngles[0]);
        return azimuthDeg < 0 ? azimuthDeg + 360.0f : azimuthDeg;
    }
}
