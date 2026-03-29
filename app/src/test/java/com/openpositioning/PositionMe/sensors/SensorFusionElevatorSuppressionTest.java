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
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class SensorFusionElevatorSuppressionTest {

    private static final double ORIGIN_LAT = 55.9444;
    private static final double ORIGIN_LON = -3.1878;

    private final SensorFusion sensorFusion = SensorFusion.getInstance();

    @After
    public void tearDown() throws Exception {
        MapConstraintRepository.clear();
        invokeResetElevatorState();
        setField("isStationary", false);
        setField("latestFusedPose", null);
        getRecentAcceptedStepTimestamps().clear();
        setField("lastAcceptedStepTimestampMs", Long.MIN_VALUE);
        setField("floorOnlyResyncAbsoluteFloor", Integer.MIN_VALUE);
        setField("floorOnlyResyncWindowUntilMs", Long.MIN_VALUE);
        setField("pendingDisplayFloorResetAbsoluteFloor", Integer.MIN_VALUE);
        setField("pendingDisplayFloorResetUntilMs", Long.MIN_VALUE);
        setField("coordinateConverter", null);
        setField("particleFilterEngine", null);
        setField("pfInitialized", false);
        setField("saveRecording", false);
        setField("pdrFloorOffset", 0);
        setField("isFloorOffsetInitialized", false);
        setField("pendingElevatorSuppressionEscapeFloor", null);
        setField("pendingElevatorSuppressionEscapeUntilMs", Long.MIN_VALUE);
        setField("weakElevatorSuppressionUntilMs", Long.MIN_VALUE);
    }

    @Test
    public void strongBarometerMotionTriggersElevatorEvenWhenNotStationary() throws Exception {
        long now = System.currentTimeMillis();
        setField("isStationary", false);
        setField("lastElevatorBarometerTimestampMs", now);
        setField("recentBarometerVerticalWindowStartMs", now - 2_500L);
        setField("smoothedBarometerVerticalSpeedMps", 0.24f);
        setField("recentBarometerNetVerticalDisplacementMeters", 2.15f);
        setField("recentBarometerVerticalTravelMeters", 2.30f);

        assertTrue(invokeIsBarometerSuggestingElevator(now));
    }

    @Test
    public void recentWalkingStepsSuppressStrongBarometerElevatorDetection() throws Exception {
        long now = System.currentTimeMillis();
        setField("isStationary", false);
        setField("lastAcceptedStepTimestampMs", now - 200L);
        java.util.ArrayDeque<Long> recentSteps = getRecentAcceptedStepTimestamps();
        recentSteps.clear();
        recentSteps.add(now - 1_800L);
        recentSteps.add(now - 1_000L);
        recentSteps.add(now - 200L);
        setField("lastElevatorBarometerTimestampMs", now);
        setField("recentBarometerVerticalWindowStartMs", now - 2_500L);
        setField("smoothedBarometerVerticalSpeedMps", 0.30f);
        setField("recentBarometerNetVerticalDisplacementMeters", 2.20f);
        setField("recentBarometerVerticalTravelMeters", 2.35f);

        assertFalse(invokeIsBarometerSuggestingElevator(now));
    }

    @Test
    public void elevatorSuppressionRejectsFarWifiFixWhileElevatorActive() throws Exception {
        long now = System.currentTimeMillis();
        setField("elevator", true);
        setField("latestFusedPose", new FusedPose(0.0, 0.0, 1, 1.0, now - 100L));

        assertTrue(invokeShouldRejectAbsoluteFixDuringElevatorSuppression(
                new double[]{6.2, 0.0},
                1,
                now,
                "WIFI"
        ));
    }

    @Test
    public void bootstrapAbsoluteFixBypassesElevatorSuppressionBeforePfInitialization() throws Exception {
        long now = 10_000L;
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        ParticleFilterEngine engine = createDeterministicParticleFilter();
        seedKnownFloors();
        setField("saveRecording", true);
        setField("coordinateConverter", converter);
        setField("particleFilterEngine", engine);
        setField("pfInitialized", false);
        setField("elevator", true);
        setField("latestFusedPose", new FusedPose(0.0, 0.0, 1, 0.20, now - 100L));

        LatLng farFix = converter.toLatLng(8.0, 0.0);
        invokeHandleAbsoluteFixInternal(
                farFix.latitude,
                farFix.longitude,
                1,
                1,
                now,
                5.0f,
                "WIFI",
                500L
        );

        assertTrue((Boolean) getField("pfInitialized"));
        assertFalse(sensorFusion.isWaitingForAbsoluteFix());
        FusedPose fusedPose = getLatestFusedPose();
        assertNotNull(fusedPose);
        assertEquals(1, fusedPose.getFloor());
        assertEquals("WIFI:fused", getField("lastAbsoluteFixDecision"));
    }

    @Test
    public void elevatorSuppressionCooldownRejectsFarGnssFixAfterExit() throws Exception {
        long now = System.currentTimeMillis();
        setField("latestFusedPose", new FusedPose(0.0, 0.0, 2, 1.0, now - 100L));
        setField("elevatorAbsoluteFixRejectUntilMs", now + 3_000L);

        assertTrue(invokeShouldRejectAbsoluteFixDuringElevatorSuppression(
                new double[]{0.0, 7.5},
                2,
                now,
                "GNSS"
        ));
    }

    @Test
    public void elevatorSuppressionKeepsNearbyWifiFixAvailable() throws Exception {
        long now = System.currentTimeMillis();
        setField("elevator", true);
        setField("latestFusedPose", new FusedPose(0.0, 0.0, 0, 1.0, now - 100L));

        assertFalse(invokeShouldRejectAbsoluteFixDuringElevatorSuppression(
                new double[]{3.0, 2.0},
                0,
                now,
                "WIFI"
        ));
    }

    @Test
    public void consistentWifiFloorFixBreaksFalseElevatorSuppression() throws Exception {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        seedKnownFloors();
        setField("saveRecording", true);
        setField("coordinateConverter", converter);
        ParticleFilterEngine engine = createDeterministicParticleFilter();
        engine.initialize(0.0, 0.0, 0, 1_000L, 0.0, 8.0);
        setField("particleFilterEngine", engine);
        setField("pfInitialized", true);
        setField("pdrFloorOffset", 0);
        setField("isFloorOffsetInitialized", true);
        setField("latestFusedPose", new FusedPose(0.0, 0.0, 0, 0.30, 1_000L));
        invokeMarkWeakElevatorSuppressionState(10_000L, "false_elevator");
        setField("elevatorAbsoluteFixRejectUntilMs", 20_000L);

        LatLng fix = converter.toLatLng(8.0, 0.0);
        double[] localFix = converter.toLocalMeters(fix.latitude, fix.longitude);
        assertTrue(invokeShouldRejectAbsoluteFixDuringElevatorSuppression(
                localFix,
                1,
                10_500L,
                "WIFI"
        ));
        assertTrue(invokeShouldRejectAbsoluteFixDuringElevatorSuppression(
                localFix,
                1,
                11_500L,
                "WIFI"
        ));
        assertFalse(invokeShouldRejectAbsoluteFixDuringElevatorSuppression(
                localFix,
                1,
                12_500L,
                "WIFI"
        ));

        Integer acceptedFloor = invokeResolveAcceptedAbsoluteFloorPrior(1, 12_500L);
        assertEquals(Integer.valueOf(1), acceptedFloor);
        sensorFusion.recalibrateAbsoluteFloorBaseline(acceptedFloor, 12_500L);
        assertEquals(1, sensorFusion.getCurrentFloor());
        FusedPose fusedPose = getLatestFusedPose();
        assertNotNull(fusedPose);
        assertEquals(1, fusedPose.getFloor());
    }

    @Test
    public void realElevatorTransitionStillPreservesAbsoluteFixSuppression() throws Exception {
        long now = System.currentTimeMillis();
        setField("elevator", true);
        setField("pfInitialized", true);
        setField("latestFusedPose", new FusedPose(0.0, 0.0, 1, 1.0, now - 100L));
        setField("weakElevatorSuppressionUntilMs", Long.MIN_VALUE);

        assertTrue(invokeShouldRejectAbsoluteFixDuringElevatorSuppression(
                new double[]{6.2, 0.0},
                2,
                now,
                "WIFI"
        ));
    }

    @Test
    public void floorDisplayRecoversAfterFalseElevatorLockClears() throws Exception {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        seedKnownFloors();
        setField("saveRecording", true);
        setField("coordinateConverter", converter);
        ParticleFilterEngine engine = createDeterministicParticleFilter();
        engine.initialize(0.0, 0.0, 0, 1_000L, 0.0, 8.0);
        setField("particleFilterEngine", engine);
        setField("pfInitialized", true);
        setField("pdrFloorOffset", 0);
        setField("isFloorOffsetInitialized", true);
        setField("latestFusedPose", new FusedPose(0.0, 0.0, 0, 0.30, 1_000L));
        invokeMarkWeakElevatorSuppressionState(10_000L, "false_elevator");
        setField("elevatorAbsoluteFixRejectUntilMs", 20_000L);

        LatLng fix = converter.toLatLng(8.0, 0.0);
        double[] localFix = converter.toLocalMeters(fix.latitude, fix.longitude);
        assertTrue(invokeShouldRejectAbsoluteFixDuringElevatorSuppression(
                localFix,
                1,
                10_500L,
                "WIFI"
        ));
        assertTrue(invokeShouldRejectAbsoluteFixDuringElevatorSuppression(
                localFix,
                1,
                11_500L,
                "WIFI"
        ));
        assertFalse(invokeShouldRejectAbsoluteFixDuringElevatorSuppression(
                localFix,
                1,
                12_500L,
                "WIFI"
        ));
        Integer acceptedFloor = invokeResolveAcceptedAbsoluteFloorPrior(1, 12_500L);
        assertEquals(Integer.valueOf(1), acceptedFloor);
        sensorFusion.recalibrateAbsoluteFloorBaseline(acceptedFloor, 12_500L);
        assertEquals(1, sensorFusion.getCurrentFloor());
        FusedPose fusedPose = getLatestFusedPose();
        assertNotNull(fusedPose);
        assertEquals(1, fusedPose.getFloor());
    }

    @Test
    public void singleBarometerSpikeDoesNotEnterElevatorSuppression() throws Exception {
        long now = System.currentTimeMillis();
        setField("isStationary", true);
        setField("lastElevatorBarometerTimestampMs", now);
        setField("recentBarometerVerticalWindowStartMs", now - 300L);
        setField("smoothedBarometerVerticalSpeedMps", 0.40f);
        setField("recentBarometerNetVerticalDisplacementMeters", 0.35f);
        setField("recentBarometerVerticalTravelMeters", 0.40f);

        assertFalse(invokeIsBarometerSuggestingElevator(now));
        assertFalse(invokeIsElevatorAbsoluteFixSuppressionActive(now));
    }

    private boolean invokeIsBarometerSuggestingElevator(long timestampMs) throws Exception {
        Method method = SensorFusion.class.getDeclaredMethod(
                "isBarometerSuggestingElevator",
                long.class
        );
        method.setAccessible(true);
        return (boolean) method.invoke(sensorFusion, timestampMs);
    }

    private boolean invokeShouldRejectAbsoluteFixDuringElevatorSuppression(
            double[] localFix,
            Integer floorPrior,
            long timestampMs,
            String source
    ) throws Exception {
        Method method = SensorFusion.class.getDeclaredMethod(
                "shouldRejectAbsoluteFixDuringElevatorSuppression",
                double.class,
                double.class,
                double[].class,
                Integer.class,
                long.class,
                String.class,
                boolean.class
        );
        method.setAccessible(true);
        return (boolean) method.invoke(
                sensorFusion,
                ORIGIN_LAT,
                ORIGIN_LON,
                localFix,
                floorPrior,
                timestampMs,
                source,
                false
        );
    }

    private void invokeHandleAbsoluteFixInternal(
            double latitudeDeg,
            double longitudeDeg,
            Integer initializationFloor,
            Integer floorPrior,
            long timestampMs,
            float accuracyMeters,
            String debugSource,
            long observationAgeMs
    ) throws Exception {
        Method method = SensorFusion.class.getDeclaredMethod(
                "handleAbsoluteFixInternal",
                double.class,
                double.class,
                Integer.class,
                Integer.class,
                long.class,
                float.class,
                String.class,
                long.class
        );
        method.setAccessible(true);
        method.invoke(
                sensorFusion,
                latitudeDeg,
                longitudeDeg,
                initializationFloor,
                floorPrior,
                timestampMs,
                accuracyMeters,
                debugSource,
                observationAgeMs
        );
    }

    private boolean invokeIsElevatorAbsoluteFixSuppressionActive(long timestampMs) throws Exception {
        Method method = SensorFusion.class.getDeclaredMethod(
                "isElevatorAbsoluteFixSuppressionActive",
                long.class
        );
        method.setAccessible(true);
        return (boolean) method.invoke(sensorFusion, timestampMs);
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
        return (Integer) method.invoke(sensorFusion, ORIGIN_LAT, ORIGIN_LON, reportedFloor, timestampMs);
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

    private void setField(String fieldName, Object value) throws Exception {
        Field field = SensorFusion.class.getDeclaredField(fieldName);
        field.setAccessible(true);
        field.set(sensorFusion, value);
    }

    private Object getField(String fieldName) throws Exception {
        Field field = SensorFusion.class.getDeclaredField(fieldName);
        field.setAccessible(true);
        return field.get(sensorFusion);
    }

    @SuppressWarnings("unchecked")
    private java.util.ArrayDeque<Long> getRecentAcceptedStepTimestamps() throws Exception {
        Field field = SensorFusion.class.getDeclaredField("recentAcceptedStepTimestampsMs");
        field.setAccessible(true);
        return (java.util.ArrayDeque<Long>) field.get(sensorFusion);
    }

    private ParticleFilterEngine createDeterministicParticleFilter() {
        return new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                ParticleInitializer.allowAll(),
                new ZeroRandom()
        );
    }

    private void seedKnownFloors() {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        MapConstraintRepository.replaceVenueConstraints(
                "venue-test",
                square(converter, -20.0, -20.0, 60.0)
        );
        java.util.List<java.util.List<LatLng>> floorZeroWalls = Collections.singletonList(
                square(converter, 25.0, 25.0, 1.0)
        );
        java.util.List<java.util.List<LatLng>> floorOneWalls = Collections.singletonList(
                square(converter, 26.0, 26.0, 1.0)
        );
        MapConstraintRepository.setConstraintsForFloor(0, "0", floorZeroWalls, null);
        MapConstraintRepository.setConstraintsForFloor(1, "1", floorOneWalls, null);
    }

    private FusedPose getLatestFusedPose() throws Exception {
        Field field = SensorFusion.class.getDeclaredField("latestFusedPose");
        field.setAccessible(true);
        return (FusedPose) field.get(sensorFusion);
    }

    private static final class ZeroRandom extends java.util.Random {
        @Override
        public double nextGaussian() {
            return 0.0;
        }

        @Override
        public double nextDouble() {
            return 0.5;
        }
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
