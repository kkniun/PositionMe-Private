package com.openpositioning.PositionMe.sensors;

import com.google.android.gms.maps.model.LatLng;
import com.openpositioning.PositionMe.utils.CoordinateConverter;
import com.openpositioning.PositionMe.utils.MapConstraintRepository;

import org.junit.After;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.Collections;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

public class SensorFusionFloorOnlyResyncTest {

    private static final double ORIGIN_LAT = 55.9444;
    private static final double ORIGIN_LON = -3.1878;

    private final SensorFusion sensorFusion = SensorFusion.getInstance();

    @After
    public void tearDown() throws Exception {
        MapConstraintRepository.clear();
        sensorFusion.resetAbsoluteAnchorStateForTesting();
        setField("floorOnlyResyncAbsoluteFloor", Integer.MIN_VALUE);
        setField("floorOnlyResyncWindowUntilMs", Long.MIN_VALUE);
        setField("pendingDisplayFloorResetAbsoluteFloor", Integer.MIN_VALUE);
        setField("pendingDisplayFloorResetUntilMs", Long.MIN_VALUE);
        setField("committedDisplayFloorAbsolute", Integer.MIN_VALUE);
        setField("pendingCommittedDisplayFloorAbsolute", Integer.MIN_VALUE);
        setField("liveCurrentFloorAbsolute", Integer.MIN_VALUE);
        setField("floorSwitchPending", false);
        setField("pendingStableAbsoluteFloorCandidate", null);
        setField("pendingStableAbsoluteFloorCount", 0);
        setField("pendingStableAbsoluteFloorFirstTimestampMs", Long.MIN_VALUE);
        setField("pendingStableAbsoluteFloorLastTimestampMs", Long.MIN_VALUE);
        setField("pdrFloorOffset", 0);
        setField("pfInitialized", false);
        setField("isFloorOffsetInitialized", false);
        setField("lastFloorConsensus", "reset");
        setField("lastFloorSource", "relative_only");
        setField("latestFusedPose", null);
        setField("pendingElevatorSuppressionEscapeFloor", null);
        setField("pendingElevatorSuppressionEscapeUntilMs", Long.MIN_VALUE);
    }

    @Test
    public void mismatchedWifiFloorIsRejectedDuringFloorOnlyResyncWindow() throws Exception {
        long now = System.currentTimeMillis();
        invokeActivateFloorOnlyResyncWindow(3, now);

        Object policy = invokeResolveFloorOnlyResyncPolicy("WIFI", Integer.valueOf(2), 4.0f, now);

        assertTrue(getBooleanField(policy, "rejectAbsoluteFix"));
    }

    @Test
    public void floorlessGnssFixIsPinnedToNewFloorWithLooserAccuracyDuringResyncWindow()
            throws Exception {
        long now = System.currentTimeMillis();
        invokeActivateFloorOnlyResyncWindow(4, now);

        Object policy = invokeResolveFloorOnlyResyncPolicy("GNSS", null, 4.0f, now);

        assertFalse(getBooleanField(policy, "rejectAbsoluteFix"));
        assertEquals(Integer.valueOf(4), getObjectField(policy, "effectiveFloorPrior"));
        assertEquals(12.0f, getFloatField(policy, "effectiveAccuracyMeters"), 1e-6f);
    }

    @Test
    public void lockedFloorRejectsShortWifiCrossFloorBurstWithoutBarometer() throws Exception {
        seedKnownFloors();
        setField("pfInitialized", true);
        setField("isFloorOffsetInitialized", true);
        setField("latestFusedPose", new FusedPose(0.0, 0.0, 0, 0.30, 900L));

        long now = 1_000L;
        assertNull(invokeResolveAcceptedAbsoluteFloorPrior(1, now));
        assertNull(invokeResolveAcceptedAbsoluteFloorPrior(1, now + 1_500L));
        assertNull(invokeResolveAcceptedAbsoluteFloorPrior(1, now + 3_000L));
        assertEquals("blocked_by_no_barometer", getObjectField(sensorFusion, "lastFloorConsensus"));
    }

    @Test
    public void lockedFloorPromotesStableWifiConsensusAfterSustainedWindow() throws Exception {
        seedKnownFloors();
        setField("pfInitialized", true);
        setField("isFloorOffsetInitialized", true);
        setField("latestFusedPose", new FusedPose(0.0, 0.0, 0, 0.30, 9_900L));

        long now = 10_000L;
        assertNull(invokeResolveAcceptedAbsoluteFloorPrior(1, now));
        assertNull(invokeResolveAcceptedAbsoluteFloorPrior(1, now + 3_000L));
        assertNull(invokeResolveAcceptedAbsoluteFloorPrior(1, now + 6_000L));
        assertEquals(Integer.valueOf(1), invokeResolveAcceptedAbsoluteFloorPrior(1, now + 9_000L));
        assertEquals(
                "recovered_from_wrong_lock",
                getObjectField(sensorFusion, "lastFloorConsensus")
        );
    }

    private void invokeActivateFloorOnlyResyncWindow(int absoluteFloor, long timestampMs)
            throws Exception {
        Method method = SensorFusion.class.getDeclaredMethod(
                "activateFloorOnlyResyncWindow",
                int.class,
                long.class
        );
        method.setAccessible(true);
        method.invoke(sensorFusion, absoluteFloor, timestampMs);
    }

    private Object invokeResolveFloorOnlyResyncPolicy(
            String debugSource,
            Integer sanitizedFloorPrior,
            float accuracyMeters,
            long timestampMs
    ) throws Exception {
        Method method = SensorFusion.class.getDeclaredMethod(
                "resolveFloorOnlyResyncPolicy",
                String.class,
                Integer.class,
                float.class,
                long.class
        );
        method.setAccessible(true);
        return method.invoke(sensorFusion, debugSource, sanitizedFloorPrior, accuracyMeters, timestampMs);
    }

    private Integer invokeResolveAcceptedAbsoluteFloorPrior(int reportedFloor, long timestampMs)
            throws Exception {
        Method method = SensorFusion.class.getDeclaredMethod(
                "resolveAcceptedAbsoluteFloorPrior",
                double.class,
                double.class,
                Integer.class,
                long.class
        );
        method.setAccessible(true);
        return (Integer) method.invoke(sensorFusion, 55.9444, -3.1878, reportedFloor, timestampMs);
    }

    private boolean getBooleanField(Object target, String fieldName) throws Exception {
        Field field = target.getClass().getDeclaredField(fieldName);
        field.setAccessible(true);
        return field.getBoolean(target);
    }

    private float getFloatField(Object target, String fieldName) throws Exception {
        Field field = target.getClass().getDeclaredField(fieldName);
        field.setAccessible(true);
        return field.getFloat(target);
    }

    private Object getObjectField(Object target, String fieldName) throws Exception {
        Field field = target.getClass().getDeclaredField(fieldName);
        field.setAccessible(true);
        return field.get(target);
    }

    private void setField(String fieldName, Object value) throws Exception {
        Field field = SensorFusion.class.getDeclaredField(fieldName);
        field.setAccessible(true);
        field.set(sensorFusion, value);
    }

    private void seedKnownFloors() {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        MapConstraintRepository.replaceVenueConstraints(
                "venue",
                square(converter, -20.0, -20.0, 60.0)
        );
        MapConstraintRepository.setConstraintsForFloor(0, "0", Collections.emptyList(), null);
        MapConstraintRepository.setConstraintsForFloor(1, "1", Collections.emptyList(), null);
        MapConstraintRepository.setLiftsForFloor(
                0,
                Collections.singletonList(square(converter, -1.0, -1.0, 2.0))
        );
        MapConstraintRepository.setLiftsForFloor(
                1,
                Collections.singletonList(square(converter, -1.0, -1.0, 2.0))
        );
    }

    private static java.util.List<LatLng> square(
            CoordinateConverter converter,
            double westXMeters,
            double southYMeters,
            double sizeMeters
    ) {
        return java.util.List.of(
                converter.toLatLng(westXMeters, southYMeters),
                converter.toLatLng(westXMeters + sizeMeters, southYMeters),
                converter.toLatLng(westXMeters + sizeMeters, southYMeters + sizeMeters),
                converter.toLatLng(westXMeters, southYMeters + sizeMeters)
        );
    }
}
