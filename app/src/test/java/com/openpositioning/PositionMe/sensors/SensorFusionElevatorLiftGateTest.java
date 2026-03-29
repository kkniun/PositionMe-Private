package com.openpositioning.PositionMe.sensors;

import com.openpositioning.PositionMe.utils.CoordinateConverter;
import com.openpositioning.PositionMe.utils.MapConstraintRepository;

import org.junit.After;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.Collections;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class SensorFusionElevatorLiftGateTest {

    private static final double ORIGIN_LAT = 55.9444;
    private static final double ORIGIN_LON = -3.1878;

    private final SensorFusion sensorFusion = SensorFusion.getInstance();

    @After
    public void tearDown() throws Exception {
        MapConstraintRepository.clear();
        invokeResetElevatorState();
        setField("coordinateConverter", null);
        setField("latestFusedPose", null);
        setField("pdrFloorOffset", 0);
        setField("isFloorOffsetInitialized", false);
        setField("lastFloorAnchorState", "unresolved");
        setField("postLiftExpectedAbsoluteFloor", Integer.MIN_VALUE);
        setField("postLiftDestinationFixWindowUntilMs", Long.MIN_VALUE);
    }

    @Test
    public void strongBarometerOutsideLiftZoneDoesNotTurnElevatorOn() throws Exception {
        seedLiftZone();
        setField("isFloorOffsetInitialized", true);
        setField("latestFusedPose", new FusedPose(8.0, 8.0, 0, 0.8, 1_000L));

        configureStrongBarometerEvidence(5_000L);
        invokeUpdateElevatorState(true, 5_000L);
        configureStrongBarometerEvidence(6_700L);
        invokeUpdateElevatorState(true, 6_700L);

        assertFalse(sensorFusion.getElevator());
        assertEquals(
                "blocked_by_not_near_lift",
                sensorFusion.getMotionDebugSnapshot().elevatorGate
        );
        assertEquals("inactive", sensorFusion.getMotionDebugSnapshot().liftTransferState);
    }

    @Test
    public void strongBarometerNearLiftZoneAllowsElevatorEntry() throws Exception {
        seedLiftZone();
        setField("isFloorOffsetInitialized", true);
        setField("latestFusedPose", new FusedPose(1.0, 1.0, 0, 0.8, 1_000L));

        configureStrongBarometerEvidence(5_000L);
        invokeUpdateElevatorState(true, 5_000L);
        configureStrongBarometerEvidence(6_700L);
        invokeUpdateElevatorState(true, 6_700L);

        assertTrue(sensorFusion.getElevator());
        assertEquals(
                "passed_barometer_and_lift_zone",
                sensorFusion.getMotionDebugSnapshot().elevatorGate
        );
        assertEquals("active", sensorFusion.getMotionDebugSnapshot().liftTransferState);
    }

    @Test
    public void staleFusedPoseDoesNotKeepLiftContextLatched() throws Exception {
        CoordinateConverter converter = seedLiftZone();
        setField("isFloorOffsetInitialized", true);
        setField("latestFusedPose", new FusedPose(1.0, 1.0, 0, 0.10, 1_000L));
        setField("latitude", (float) converter.toLatLng(8.0, 8.0).latitude);
        setField("longitude", (float) converter.toLatLng(8.0, 8.0).longitude);

        assertFalse(invokeIsCurrentPositionNearLiftZone(6_500L));
    }

    @Test
    public void weakStaticElevatorSignalDoesNotStartElevatorSession() throws Exception {
        seedLiftZone();
        setField("isFloorOffsetInitialized", true);
        setField("latestFusedPose", new FusedPose(1.0, 1.0, 0, 0.8, 1_000L));
        invokeMarkWeakElevatorSuppressionState(5_000L, "test_false_entry");

        configureStrongBarometerEvidence(5_000L);
        invokeUpdateElevatorState(true, 5_000L);
        configureStrongBarometerEvidence(6_700L);
        invokeUpdateElevatorState(true, 6_700L);

        assertFalse(sensorFusion.getElevator());
        assertFalse("passed_barometer_and_lift_zone".equals(sensorFusion.getMotionDebugSnapshot().elevatorGate));
        assertEquals("inactive", sensorFusion.getMotionDebugSnapshot().liftTransferState);
    }

    @Test
    public void elevatorClearsAfterLeavingLiftContextWithoutMotion() throws Exception {
        seedLiftZone();
        setField("isFloorOffsetInitialized", true);
        setField("latestFusedPose", new FusedPose(1.0, 1.0, 0, 0.8, 1_000L));

        configureStrongBarometerEvidence(5_000L);
        invokeUpdateElevatorState(true, 5_000L);
        configureStrongBarometerEvidence(6_700L);
        invokeUpdateElevatorState(true, 6_700L);
        assertTrue(sensorFusion.getElevator());

        setField("latestFusedPose", new FusedPose(8.0, 8.0, 0, 0.8, 9_500L));
        clearBarometerEvidence();
        invokeUpdateElevatorState(false, 10_000L);
        invokeUpdateElevatorState(false, 10_100L);

        assertFalse(sensorFusion.getElevator());
        assertEquals(
                "cleared_by_no_motion_or_no_lift_context",
                sensorFusion.getMotionDebugSnapshot().elevatorGate
        );
    }

    private CoordinateConverter seedLiftZone() throws Exception {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(0, "0", Collections.emptyList(), null);
        MapConstraintRepository.setLiftsForFloor(
                0,
                Collections.singletonList(square(converter, 0.0, 0.0, 2.0))
        );
        setField("coordinateConverter", converter);
        setField("pdrFloorOffset", 0);
        return converter;
    }

    private void configureStrongBarometerEvidence(long timestampMs) throws Exception {
        setField("lastElevatorBarometerTimestampMs", timestampMs);
        setField("recentBarometerVerticalWindowStartMs", timestampMs - 2_500L);
        setField("smoothedBarometerVerticalSpeedMps", 0.24f);
        setField("recentBarometerNetVerticalDisplacementMeters", 2.15f);
        setField("recentBarometerVerticalTravelMeters", 2.30f);
    }

    private void clearBarometerEvidence() throws Exception {
        setField("lastElevatorBarometerTimestampMs", Long.MIN_VALUE);
        setField("recentBarometerVerticalWindowStartMs", Long.MIN_VALUE);
        setField("smoothedBarometerVerticalSpeedMps", 0f);
        setField("recentBarometerNetVerticalDisplacementMeters", 0f);
        setField("recentBarometerVerticalTravelMeters", 0f);
    }

    private void invokeUpdateElevatorState(boolean rawElevatorDetected, long timestampMs)
            throws Exception {
        Method method = SensorFusion.class.getDeclaredMethod(
                "updateElevatorState",
                boolean.class,
                long.class
        );
        method.setAccessible(true);
        method.invoke(sensorFusion, rawElevatorDetected, timestampMs);
    }

    private boolean invokeIsCurrentPositionNearLiftZone(long timestampMs) throws Exception {
        Method method = SensorFusion.class.getDeclaredMethod(
                "isCurrentPositionNearLiftZone",
                long.class
        );
        method.setAccessible(true);
        return (boolean) method.invoke(sensorFusion, timestampMs);
    }

    private void invokeMarkWeakElevatorSuppressionState(long timestampMs, String reason)
            throws Exception {
        Method method = SensorFusion.class.getDeclaredMethod(
                "markWeakElevatorSuppressionState",
                long.class,
                String.class
        );
        method.setAccessible(true);
        method.invoke(sensorFusion, timestampMs, reason);
    }

    private void invokeResetElevatorState() throws Exception {
        Method method = SensorFusion.class.getDeclaredMethod("resetElevatorState");
        method.setAccessible(true);
        method.invoke(sensorFusion);
    }

    private java.util.List<com.google.android.gms.maps.model.LatLng> square(
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

    private void setField(String fieldName, Object value) throws Exception {
        Field field = SensorFusion.class.getDeclaredField(fieldName);
        field.setAccessible(true);
        field.set(sensorFusion, value);
    }
}
