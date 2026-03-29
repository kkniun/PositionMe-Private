package com.openpositioning.PositionMe.sensors;

import com.google.android.gms.maps.model.LatLng;
import com.openpositioning.PositionMe.utils.CoordinateConverter;
import com.openpositioning.PositionMe.utils.MapConstraintRepository;

import org.junit.After;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.Collections;
import java.util.Random;
import java.util.Set;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class SensorFusionLiftTransferTest {

    private static final double ORIGIN_LAT = 55.9444;
    private static final double ORIGIN_LON = -3.1878;

    private final SensorFusion sensorFusion = SensorFusion.getInstance();

    @After
    public void tearDown() throws Exception {
        MapConstraintRepository.clear();
        invokeResetElevatorState();
        getTracker().reset();
        setField("coordinateConverter", null);
        setField("particleFilterEngine", null);
        setField("latestFusedPose", null);
        setField("pfInitialized", false);
        setField("pdrFloorOffset", 0);
        setField("isFloorOffsetInitialized", false);
        setField("saveRecording", false);
        setField("isStationary", true);
        setField("committedDisplayFloorAbsolute", Integer.MIN_VALUE);
        setField("pendingCommittedDisplayFloorAbsolute", Integer.MIN_VALUE);
        setField("floorSwitchPending", false);
        setField("lastFloorSwitchBlockReason", "none");
        setField("pendingDisplayFloorResetLandingLatLng", null);
        setField("pendingDisplayFloorResetAnchorSource", "none");
        setField("pendingStableAbsoluteFloorCandidate", null);
        setField("pendingStableAbsoluteFloorCount", 0);
        setField("pendingStableAbsoluteFloorFirstTimestampMs", Long.MIN_VALUE);
        setField("pendingStableAbsoluteFloorLastTimestampMs", Long.MIN_VALUE);
        setField("pendingWifiBootstrapFloor", null);
        setField("pendingWifiBootstrapCount", 0);
        setField("lastWifiBootstrapCandidateTimestampMs", Long.MIN_VALUE);
        setField("lastFloorConsensus", "reset");
        setField("lastFloorSource", "relative_only");
        setField("lastFloorAnchorState", "unresolved");
        setField("lastLiftTransferState", "inactive");
        setField("lastPostLiftState", "inactive");
        setField("postLiftExpectedAbsoluteFloor", Integer.MIN_VALUE);
        setField("postLiftDestinationFixWindowUntilMs", Long.MIN_VALUE);
        sensorFusion.setFloorHeightOverrideForTesting(Float.NaN);
    }

    @Test
    public void confirmedLiftTraversalUsesPendingBootstrapFloorBeforeInitialLock() throws Exception {
        CoordinateConverter converter = seedLiftShafts();
        seedDeterministicParticleFilter();
        seedFloorHeight(4.0f);
        setField("coordinateConverter", converter);
        setField("latestFusedPose", new FusedPose(1.0, 1.0, 0, 0.25, 1_000L));
        setField("pdrFloorOffset", 0);
        setField("isFloorOffsetInitialized", false);
        setField("pendingWifiBootstrapFloor", 1);
        setField("pendingWifiBootstrapCount", 3);
        setField("lastWifiBootstrapCandidateTimestampMs", 6_000L);

        configureStrongBarometerEvidence(6_000L);
        invokeUpdateElevatorState(true, 6_000L);
        configureStrongBarometerEvidence(7_700L);
        invokeUpdateElevatorState(true, 7_700L);

        assertTrue(sensorFusion.getElevator());
        assertEquals("active", getStringField("lastLiftTransferState"));
        assertEquals("corrected_floor_anchor", getStringField("lastFloorAnchorState"));
        assertEquals(1, getIntField("elevatorFloorSessionStartAbsoluteFloor"));
        assertEquals(1, sensorFusion.getCurrentFloor());
    }

    @Test
    public void liftTraversalReseedsDestinationLiftAndResolvesUpperFloor() throws Exception {
        CoordinateConverter converter = seedLiftShafts();
        seedDeterministicParticleFilter();
        seedFloorHeight(4.0f);
        setField("coordinateConverter", converter);
        setField("latestFusedPose", new FusedPose(1.0, 1.0, 1, 0.9, 1_000L));
        setField("pdrFloorOffset", 1);
        setField("isFloorOffsetInitialized", true);
        setField("committedDisplayFloorAbsolute", 1);

        configureStrongBarometerEvidence(5_000L);
        invokeUpdateElevatorState(true, 5_000L);
        configureStrongBarometerEvidence(6_700L);
        invokeUpdateElevatorState(true, 6_700L);
        setField("elevatorFloorSessionStartElevationMeters", 0f);
        setField("elevatorFloorSessionFilteredElevationMeters", 4.25f);

        invokeFinishElevatorFloorSession(10_000L);

        FusedPose fusedPose = getLatestFusedPose();
        assertEquals(2, sensorFusion.getCurrentFloor());
        assertEquals(1, sensorFusion.getUserVisibleFloor());
        assertEquals(11.0, fusedPose.getX(), 1e-6);
        assertEquals(11.0, fusedPose.getY(), 1e-6);
        assertEquals("reseeded_destination_lift", getStringField("lastLiftTransferState"));
        assertEquals("resolved_from_lift_transfer", getStringField("lastFloorAnchorState"));
        assertEquals("awaiting_destination_fix", getStringField("lastPostLiftState"));
        assertTrue(sensorFusion.hasPendingDisplayFloorReset(2, 10_000L));
        LatLng landing = sensorFusion.peekPendingDisplayFloorResetLanding(2, 10_000L);
        assertNotNull(landing);
        assertTrue(sensorFusion.commitPendingDisplayFloorReset(2, 10_000L, landing));
        assertEquals(2, sensorFusion.getUserVisibleFloor());
    }

    @Test
    public void liftTraversalWithoutDestinationAnchorDoesNotCommitFloorSwitch() throws Exception {
        CoordinateConverter converter = seedLiftShafts();
        MapConstraintRepository.setLiftsForFloor(2, Collections.emptyList());
        seedDeterministicParticleFilter();
        seedFloorHeight(4.0f);
        setField("coordinateConverter", converter);
        setField("latestFusedPose", new FusedPose(1.0, 1.0, 1, 0.9, 1_000L));
        setField("pdrFloorOffset", 1);
        setField("isFloorOffsetInitialized", true);
        setField("committedDisplayFloorAbsolute", 1);

        configureStrongBarometerEvidence(5_000L);
        invokeUpdateElevatorState(true, 5_000L);
        configureStrongBarometerEvidence(6_700L);
        invokeUpdateElevatorState(true, 6_700L);
        setField("elevatorFloorSessionStartElevationMeters", 0f);
        setField("elevatorFloorSessionFilteredElevationMeters", 4.25f);

        invokeFinishElevatorFloorSession(10_000L);

        FusedPose fusedPose = getLatestFusedPose();
        assertEquals(1, sensorFusion.getCurrentFloor());
        assertEquals(1, sensorFusion.getUserVisibleFloor());
        assertEquals(1, fusedPose.getFloor());
        assertEquals(1.0, fusedPose.getX(), 1e-6);
        assertEquals(1.0, fusedPose.getY(), 1e-6);
        assertFalse(sensorFusion.hasPendingDisplayFloorReset(2, 10_000L));
        assertEquals("blocked_missing_destination_lift_anchor", getStringField("lastLiftTransferState"));
        assertEquals("landing_blocked", getStringField("lastPostLiftState"));
    }

    @Test
    public void postLiftDestinationFloorFixIsAcceptedDuringCooldown() throws Exception {
        CoordinateConverter converter = seedLiftShafts();
        seedDeterministicParticleFilter();
        seedFloorHeight(4.0f);
        setField("coordinateConverter", converter);
        setField("latestFusedPose", new FusedPose(1.0, 1.0, 1, 0.9, 1_000L));
        setField("pdrFloorOffset", 1);
        setField("isFloorOffsetInitialized", true);

        configureStrongBarometerEvidence(5_000L);
        invokeUpdateElevatorState(true, 5_000L);
        configureStrongBarometerEvidence(6_700L);
        invokeUpdateElevatorState(true, 6_700L);
        setField("elevatorFloorSessionStartElevationMeters", 0f);
        setField("elevatorFloorSessionFilteredElevationMeters", 4.25f);
        invokeFinishElevatorFloorSession(10_000L);

        LatLng farDestinationFix = converter.toLatLng(20.0, 20.0);
        setField("saveRecording", true);
        setField("isStationary", false);
        invokeHandleAbsoluteFixInternal(
                farDestinationFix.latitude,
                farDestinationFix.longitude,
                2,
                Integer.valueOf(2),
                12_000L,
                4.0f,
                "WIFI",
                0L
        );

        FusedPose fusedPose = getLatestFusedPose();
        assertEquals(2, fusedPose.getFloor());
        assertTrue(Set.of("reseeded_destination_lift", "cleared_completed")
                .contains(getStringField("lastLiftTransferState")));
        assertTrue(Set.of("accepted_destination_fix", "cleared_after_accept")
                .contains(getStringField("lastPostLiftState")));
    }

    @Test
    public void descendingFromSecondFloorReturnsToFirstFloorNotGroundFloor() throws Exception {
        CoordinateConverter converter = seedLiftShafts();
        seedDeterministicParticleFilter();
        seedFloorHeight(4.0f);
        setField("coordinateConverter", converter);
        setField("latestFusedPose", new FusedPose(11.0, 11.0, 2, 0.9, 1_000L));
        setField("pdrFloorOffset", 2);
        setField("isFloorOffsetInitialized", true);

        configureStrongBarometerEvidence(15_000L);
        invokeUpdateElevatorState(true, 15_000L);
        configureStrongBarometerEvidence(16_700L);
        invokeUpdateElevatorState(true, 16_700L);
        setField("elevatorFloorSessionStartElevationMeters", 0f);
        setField("elevatorFloorSessionFilteredElevationMeters", -4.25f);

        invokeFinishElevatorFloorSession(20_000L);

        FusedPose fusedPose = getLatestFusedPose();
        assertEquals(1, sensorFusion.getCurrentFloor());
        assertEquals(1, fusedPose.getFloor());
    }

    private CoordinateConverter seedLiftShafts() throws Exception {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        MapConstraintRepository.replaceVenueConstraints(
                "venue",
                square(converter, -20.0, -20.0, 60.0)
        );
        MapConstraintRepository.setConstraintsForFloor(1, "1", Collections.emptyList(), null);
        MapConstraintRepository.setConstraintsForFloor(2, "2", Collections.emptyList(), null);
        MapConstraintRepository.setLiftsForFloor(
                1,
                Collections.singletonList(square(converter, 0.0, 0.0, 2.0))
        );
        MapConstraintRepository.setLiftsForFloor(
                2,
                Collections.singletonList(square(converter, 10.0, 10.0, 2.0))
        );
        return converter;
    }

    private void seedDeterministicParticleFilter() throws Exception {
        ParticleFilterEngine engine = new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                ParticleInitializer.allowAll(),
                new ZeroRandom()
        );
        setField("particleFilterEngine", engine);
    }

    private void seedFloorHeight(float floorHeightMeters) throws Exception {
        sensorFusion.setFloorHeightOverrideForTesting(floorHeightMeters);
    }

    private void configureStrongBarometerEvidence(long timestampMs) throws Exception {
        setField("lastElevatorBarometerTimestampMs", timestampMs);
        setField("recentBarometerVerticalWindowStartMs", timestampMs - 2_500L);
        setField("smoothedBarometerVerticalSpeedMps", 0.24f);
        setField("recentBarometerNetVerticalDisplacementMeters", 2.15f);
        setField("recentBarometerVerticalTravelMeters", 2.30f);
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

    private void invokeFinishElevatorFloorSession(long timestampMs) throws Exception {
        Method method = SensorFusion.class.getDeclaredMethod(
                "finishElevatorFloorSession",
                long.class
        );
        method.setAccessible(true);
        method.invoke(sensorFusion, timestampMs);
    }

    private void invokeHandleAbsoluteFixInternal(
            double latitudeDeg,
            double longitudeDeg,
            int initializationFloor,
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

    private void invokeResetElevatorState() throws Exception {
        Method method = SensorFusion.class.getDeclaredMethod("resetElevatorState");
        method.setAccessible(true);
        method.invoke(sensorFusion);
    }

    private CumulativeFloorTracker getTracker() throws Exception {
        Field field = SensorFusion.class.getDeclaredField("cumulativeFloorTracker");
        field.setAccessible(true);
        return (CumulativeFloorTracker) field.get(sensorFusion);
    }

    private FusedPose getLatestFusedPose() throws Exception {
        Field field = SensorFusion.class.getDeclaredField("latestFusedPose");
        field.setAccessible(true);
        return (FusedPose) field.get(sensorFusion);
    }

    private int getIntField(String fieldName) throws Exception {
        Field field = SensorFusion.class.getDeclaredField(fieldName);
        field.setAccessible(true);
        return field.getInt(sensorFusion);
    }

    private String getStringField(String fieldName) throws Exception {
        Field field = SensorFusion.class.getDeclaredField(fieldName);
        field.setAccessible(true);
        return (String) field.get(sensorFusion);
    }

    private void setField(String fieldName, Object value) throws Exception {
        Field field = SensorFusion.class.getDeclaredField(fieldName);
        field.setAccessible(true);
        field.set(sensorFusion, value);
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

    private static final class ZeroRandom extends Random {
        @Override
        public double nextGaussian() {
            return 0.0;
        }

        @Override
        public double nextDouble() {
            return 0.5;
        }
    }
}
