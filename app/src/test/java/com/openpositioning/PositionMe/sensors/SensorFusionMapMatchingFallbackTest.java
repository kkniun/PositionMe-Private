package com.openpositioning.PositionMe.sensors;

import com.google.android.gms.maps.model.LatLng;
import com.openpositioning.PositionMe.utils.CoordinateConverter;
import com.openpositioning.PositionMe.utils.MapConstraintRepository;

import org.junit.After;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.Map;
import java.util.Collections;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public class SensorFusionMapMatchingFallbackTest {

    private static final double ORIGIN_LAT = 55.9444;
    private static final double ORIGIN_LON = -3.1878;

    private final SensorFusion sensorFusion = SensorFusion.getInstance();

    @After
    public void tearDown() throws Exception {
        MapConstraintRepository.clear();
        sensorFusion.resetAbsoluteAnchorStateForTesting();
        getTracker().reset();
        setCoordinateConverter(null);
        setParticleFilterEngine(null);
        setLatestFusedPose(null);
        setPfInitialized(false);
        setField("liveCurrentFloorAbsolute", Integer.MIN_VALUE);
        setField("committedDisplayFloorAbsolute", Integer.MIN_VALUE);
        setField("pendingCommittedDisplayFloorAbsolute", Integer.MIN_VALUE);
        setField("floorSwitchPending", false);
        setField("pdrFloorOffset", 0);
        setField("isFloorOffsetInitialized", false);
        setField("floorOnlyResyncAbsoluteFloor", Integer.MIN_VALUE);
        setField("floorOnlyResyncWindowUntilMs", Long.MIN_VALUE);
        setField("pendingDisplayFloorResetAbsoluteFloor", Integer.MIN_VALUE);
        setField("pendingDisplayFloorResetUntilMs", Long.MIN_VALUE);
        setField("pendingConstrainedAbsoluteFix", null);
        setField("particleCloudTrustedUnderConstraints", false);
        setField("provisionalFloorBootstrapActive", false);
        setField("provisionalFloorBootstrapFloor", Integer.MIN_VALUE);
        setField("provisionalFloorBootstrapSource", "none");
        setField("lastAbsoluteFixDecision", "reset");
        setField("saveRecording", false);
    }

    @Test
    public void sameFloorCredibleAbsoluteFixRecoversDriftedCloudThroughSensorFusion()
            throws Exception {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        setCoordinateConverter(converter);
        setField("saveRecording", true);
        setField("isStationary", false);
        setField("pdrFloorOffset", 0);
        setField("isFloorOffsetInitialized", true);

        MapConstraintRepository.replaceVenueConstraints("venue", square(converter, -30.0, -10.0, 80.0));
        MapConstraintRepository.setConstraintsForFloor(
                0,
                "0",
                Collections.singletonList(localRectangle(converter, 60.0, 60.0, 1.0, 1.0)),
                null
        );

        ParticleInitializer.SpawnValidator rejectingMotionValidator = new ParticleInitializer.SpawnValidator() {
            @Override
            public boolean isValid(double x, double y, int floor) {
                return true;
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
                return false;
            }
        };
        ParticleFilterEngine engine = new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                rejectingMotionValidator,
                new ZeroRandom()
        );
        setParticleFilterEngine(engine);
        engine.setParticlesForTesting(java.util.List.of(
                new Particle(0.0, 0.0, 0, 0.5, 0.0),
                new Particle(0.0, 0.0, 0, 0.5, 0.0)
        ), 900L);
        setLatestFusedPose(new FusedPose(0.0, 0.0, 0, 1.0, 900L));
        setPfInitialized(true);

        LatLng fix = converter.toLatLng(30.0, 0.0);
        double preRecoveryDistanceMeters = Math.hypot(
                getLatestFusedPose().getX() - 30.0,
                getLatestFusedPose().getY()
        );

        sensorFusion.handleAbsoluteFix(fix.latitude, fix.longitude, 0, 1_000L, 4.0f);

        assertTrue(engine.wasLastAbsoluteFixReanchored()
                || engine.wasLastAbsoluteFixCloudRecovered());
        FusedPose recoveredPose = getLatestFusedPose();
        assertNotNull(recoveredPose);
        assertTrue(getPfInitialized());
        assertEquals(0, recoveredPose.getFloor());
        assertEquals(30.0, recoveredPose.getX(), 1e-6);
        assertEquals(0.0, recoveredPose.getY(), 1e-6);
        assertTrue(Math.hypot(recoveredPose.getX() - 30.0, recoveredPose.getY()) < preRecoveryDistanceMeters);

        FusedPose particlePose = engine.estimatePose();
        assertNotNull(
                "particles=" + engine.snapshotParticlesForTesting().size()
                        + " pfInitialized=" + getPfInitialized()
                        + " trusted=" + getParticleCloudTrustedUnderConstraints(),
                particlePose
        );
        assertEquals(0, particlePose.getFloor());
        assertEquals(30.0, particlePose.getX(), 1e-6);
        assertEquals(0.0, particlePose.getY(), 1e-6);
        assertFalse(hasPendingConstrainedAbsoluteFix());
    }

    @Test
    public void rejectedConstrainedReanchorRollsBackHiddenPfCloud() throws Exception {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        setCoordinateConverter(converter);
        setField("saveRecording", true);
        setField("isStationary", false);
        setField("pdrFloorOffset", 0);
        setField("isFloorOffsetInitialized", true);
        setField("particleCloudTrustedUnderConstraints", false);

        MapConstraintRepository.replaceVenueConstraints("venue", square(converter, -30.0, -10.0, 80.0));
        MapConstraintRepository.setConstraintsForFloor(
                0,
                "0",
                Collections.singletonList(localRectangle(converter, 9.5, -5.0, 1.0, 10.0)),
                null
        );

        ParticleFilterEngine engine = createDeterministicSensorFusionEngine();
        setParticleFilterEngine(engine);
        engine.setParticlesForTesting(java.util.List.of(
                new Particle(0.0, 0.0, 0, 0.5, 0.0),
                new Particle(0.0, 0.0, 0, 0.5, 0.0)
        ), 900L);
        setLatestFusedPose(new FusedPose(0.0, 0.0, 0, 1.0, 900L));
        setPfInitialized(true);

        LatLng fix = converter.toLatLng(30.0, 0.0);
        sensorFusion.handleAbsoluteFix(fix.latitude, fix.longitude, 0, 1_000L, 4.0f);

        FusedPose publishedPose = getLatestFusedPose();
        assertNotNull(publishedPose);
        assertEquals(0.0, publishedPose.getX(), 1e-6);
        assertEquals(0.0, publishedPose.getY(), 1e-6);
        assertEquals(0, publishedPose.getFloor());

        FusedPose particlePose = engine.estimatePose();
        assertNotNull(particlePose);
        assertEquals(0.0, particlePose.getX(), 1e-6);
        assertEquals(0.0, particlePose.getY(), 1e-6);
        assertEquals(0, particlePose.getFloor());

        assertFalse(getParticleCloudTrustedUnderConstraints());
        assertTrue(getPfInitialized());
        assertFalse(engine.wasLastAbsoluteFixReanchored());
    }

    @Test
    public void rejectedWifiBootstrapInitializationRollsBackHiddenPfCloud() throws Exception {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        setCoordinateConverter(converter);
        setField("saveRecording", true);
        setField("pdrFloorOffset", 0);
        setField("isFloorOffsetInitialized", false);
        setField("particleCloudTrustedUnderConstraints", false);
        setField("pendingWifiBootstrapFloor", 2);
        setField("pendingWifiBootstrapCount", 2);
        setField("pendingWifiBootstrapFirstTimestampMs", 1_000L);
        setField("lastWifiBootstrapCandidateTimestampMs", 3_000L);
        setField("pendingStableAbsoluteFloorCandidate", 2);
        setField("pendingStableAbsoluteFloorCount", 2);
        setField("pendingStableAbsoluteFloorFirstTimestampMs", 1_000L);
        setField("pendingStableAbsoluteFloorLastTimestampMs", 2_000L);

        MapConstraintRepository.replaceVenueConstraints("venue", square(converter, -30.0, -10.0, 80.0));
        MapConstraintRepository.setConstraintsForFloor(
                2,
                "2",
                Collections.singletonList(localRectangle(converter, 9.5, -5.0, 1.0, 10.0)),
                null
        );

        ParticleFilterEngine engine = createDeterministicSensorFusionEngine();
        setParticleFilterEngine(engine);
        engine.setParticlesForTesting(java.util.List.of(
                new Particle(0.0, 0.0, 2, 0.5, 0.0),
                new Particle(0.0, 0.0, 2, 0.5, 0.0)
        ), 900L);
        setLatestFusedPose(new FusedPose(0.0, 0.0, 2, 1.0, 900L));
        setPfInitialized(true);

        LatLng fix = converter.toLatLng(30.0, 0.0);
        invokeHandleAbsoluteFixInternal(
                fix.latitude,
                fix.longitude,
                2,
                2,
                4_000L,
                8.0f,
                "WIFI",
                0L
        );

        FusedPose publishedPose = getLatestFusedPose();
        assertNotNull(publishedPose);
        assertEquals(0.0, publishedPose.getX(), 1e-6);
        assertEquals(0.0, publishedPose.getY(), 1e-6);
        assertEquals(2, publishedPose.getFloor());
        assertEquals(900L, publishedPose.getTimestampMs());

        FusedPose particlePose = engine.estimatePose();
        if (particlePose == null) {
            fail(
                    "particles=" + engine.snapshotParticlesForTesting().size()
                            + " pfInitialized=" + getPfInitialized()
                            + " trusted=" + getParticleCloudTrustedUnderConstraints()
            );
        }
        assertEquals(0.0, particlePose.getX(), 1e-6);
        assertEquals(0.0, particlePose.getY(), 1e-6);
        assertEquals(2, particlePose.getFloor());
        assertEquals(900L, particlePose.getTimestampMs());

        assertFalse(getParticleCloudTrustedUnderConstraints());
        assertTrue(getPfInitialized());
        assertFalse(engine.wasLastAbsoluteFixReanchored());
        assertFalse(engine.wasLastAbsoluteFixRejectedByConstraints());
    }

    @Test
    public void lowConfidenceConflictingWifiFixIsRejectedBeforeItCanReanchorWeakAnchor()
            throws Exception {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        setCoordinateConverter(converter);
        setField("saveRecording", true);
        setField("isStationary", false);
        setField("pdrFloorOffset", 0);
        setField("isFloorOffsetInitialized", true);
        setField("particleCloudTrustedUnderConstraints", true);

        MapConstraintRepository.replaceVenueConstraints("venue", square(converter, -30.0, -30.0, 80.0));
        MapConstraintRepository.setConstraintsForFloor(
                2,
                "2",
                Collections.singletonList(localRectangle(converter, -20.0, -20.0, 40.0, 40.0)),
                null
        );

        ParticleFilterEngine engine = createDeterministicSensorFusionEngine();
        setParticleFilterEngine(engine);
        engine.setParticlesForTesting(java.util.List.of(
                new Particle(0.0, 0.0, 2, 0.5, 0.0),
                new Particle(0.0, 0.0, 2, 0.5, 0.0)
        ), 900L);
        setLatestFusedPose(new FusedPose(0.0, 0.0, 2, 0.28, 900L));
        setPfInitialized(true);
        setField("lastTrustedAbsoluteAnchorPose", new FusedPose(0.0, 0.0, 2, 0.9, 900L));
        setField("lastTrustedAbsoluteAnchorSource", "GNSS");
        setAbsoluteObservationField("lastGnssAbsoluteObservation", 0.0, 8.0, 2, 3.5f, 980L);

        LatLng wifiFix = converter.toLatLng(0.0, -8.5);
        invokeHandleAbsoluteFixInternal(
                wifiFix.latitude,
                wifiFix.longitude,
                2,
                2,
                1_000L,
                5.0f,
                "WIFI",
                0L
        );

        FusedPose publishedPose = getLatestFusedPose();
        assertNotNull(publishedPose);
        assertEquals(0.0, publishedPose.getX(), 1e-6);
        assertEquals(0.0, publishedPose.getY(), 1e-6);
        assertEquals(2, publishedPose.getFloor());
        assertEquals(900L, publishedPose.getTimestampMs());
        assertEquals("WIFI:rejected_low_conflict_anchor_shift", getFieldValue("lastAbsoluteFixDecision"));
        assertTrue((Long) getFieldValue("weakAnchorModeUntilMs") >= 1_000L);

        assertFalse(engine.wasLastAbsoluteFixReanchored());
        assertFalse(engine.wasLastAbsoluteFixRejectedByConstraints());
    }

    @Test
    public void completeTransitionConstraintsDisableFailOpenEvenWithStrongBarometerEvidence()
            throws Exception {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        seedStrongAscendingFloorTransitionEvidence();
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(0, "0", Collections.emptyList(), null);
        MapConstraintRepository.setConstraintsForFloor(1, "1", Collections.emptyList(), null);
        MapConstraintRepository.setStairsForFloor(0, Collections.singletonList(square(converter, -1.0, -1.0, 2.0)));
        MapConstraintRepository.setStairsForFloor(1, Collections.singletonList(square(converter, -1.0, -1.0, 2.0)));

        assertFalse(invokeShouldFailOpenBarometerFloorTransition(0, 1));
    }

    @Test
    public void crossFloorWallMotionIsStillRejectedWhenFallbackEvidenceExists() throws Exception {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        setCoordinateConverter(converter);
        seedStrongAscendingFloorTransitionEvidence();
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(0, "0", Collections.emptyList(), null);
        MapConstraintRepository.setConstraintsForFloor(
                1,
                "1",
                Collections.singletonList(localRectangle(converter, -0.5, -1.0, 1.0, 2.0)),
                null
        );

        assertTrue(invokeShouldFailOpenBarometerFloorTransition(0, 1));
        assertFalse(invokeIsValidParticleMotion(-2.0, 0.0, 2.0, 0.0, 0, 1));
    }

    @Test
    public void floorTransitionIsNotApprovedByMapWhenConstraintsUnavailable() throws Exception {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        setCoordinateConverter(converter);
        seedStrongAscendingFloorTransitionEvidence();
        MapConstraintRepository.clear();

        assertTrue(invokeShouldFailOpenBarometerFloorTransition(0, 1));
        assertTrue(invokeAllowsMapBasedFloorTransition(0.0, 0.0, 0.0, 0.0, 0, 1));
    }

    @Test
    public void constrainedInitializationIsDeferredUntilMapReady() throws Exception {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        setCoordinateConverter(converter);
        setField("saveRecording", true);
        setParticleFilterEngine(createDeterministicSensorFusionEngine());

        LatLng fix = converter.toLatLng(0.0, 0.0);
        sensorFusion.handleAbsoluteFix(fix.latitude, fix.longitude, 0, 1_000L, 1.0f);

        assertNull(getLatestFusedPose());
        assertFalse(getPfInitialized());
        assertTrue(hasPendingConstrainedAbsoluteFix());
        assertTrue(getParticleFilterEngine().snapshotParticlesForTesting().isEmpty());

        MapConstraintRepository.replaceVenueConstraints("venue", square(converter, -5.0, -5.0, 20.0));
        MapConstraintRepository.setConstraintsForFloor(
                0,
                "0",
                Collections.singletonList(localRectangle(converter, 8.0, 8.0, 1.0, 1.0)),
                null
        );

        sensorFusion.onMapMatchingConstraintsUpdated();

        FusedPose replayedPose = getLatestFusedPose();
        assertNotNull(replayedPose);
        assertTrue(getPfInitialized());
        assertFalse(hasPendingConstrainedAbsoluteFix());
        assertEquals(0.0, replayedPose.getX(), 1e-6);
        assertEquals(0.0, replayedPose.getY(), 1e-6);
        assertEquals(0, replayedPose.getFloor());
    }

    @Test
    public void degradedBootstrapInitializesPfBeforeMapReadyAndReplaysWhenConstraintsArrive()
            throws Exception {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        setCoordinateConverter(converter);
        setField("saveRecording", true);
        setParticleFilterEngine(createDeterministicSensorFusionEngine());

        MapConstraintRepository.replaceVenueConstraints("venue", square(converter, -5.0, -5.0, 20.0));
        MapConstraintRepository.setConstraintsForFloor(1, "1", Collections.emptyList(), null);

        LatLng fix = converter.toLatLng(2.0, 1.0);
        invokeHandleAbsoluteFixInternal(
                fix.latitude,
                fix.longitude,
                1,
                1,
                1_000L,
                4.0f,
                "WIFI",
                0L
        );

        FusedPose bootstrappedPose = getLatestFusedPose();
        assertNotNull(bootstrappedPose);
        assertTrue(getPfInitialized());
        assertTrue(hasPendingConstrainedAbsoluteFix());
        assertFalse(getParticleCloudTrustedUnderConstraints());
        assertEquals("WIFI:init_degraded_bootstrap_map_not_ready", getFieldValue("lastAbsoluteFixDecision"));
        assertEquals(1, bootstrappedPose.getFloor());
        assertEquals(2.0, bootstrappedPose.getX(), 1e-6);
        assertEquals(1.0, bootstrappedPose.getY(), 1e-6);

        MapConstraintRepository.setConstraintsForFloor(
                1,
                "1",
                Collections.singletonList(localRectangle(converter, 8.0, 8.0, 1.0, 1.0)),
                null
        );

        sensorFusion.onMapMatchingConstraintsUpdated();

        FusedPose replayedPose = getLatestFusedPose();
        assertNotNull(replayedPose);
        assertTrue(getPfInitialized());
        assertFalse(hasPendingConstrainedAbsoluteFix());
        assertTrue(getParticleCloudTrustedUnderConstraints());
        assertEquals("WIFI:init_full_constraints_ready", getFieldValue("lastAbsoluteFixDecision"));
        assertEquals(1, replayedPose.getFloor());
        assertEquals(2.0, replayedPose.getX(), 1e-6);
        assertEquals(1.0, replayedPose.getY(), 1e-6);
    }

    @Test
    public void gnssUnknownFloorBootstrapUsesUntrustedSeedAndKeepsFloorUncalibrated()
            throws Exception {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        setCoordinateConverter(converter);
        setField("saveRecording", true);
        setParticleFilterEngine(createDeterministicSensorFusionEngine());
        MapConstraintRepository.replaceVenueConstraints("venue", square(converter, -5.0, -5.0, 20.0));
        MapConstraintRepository.setConstraintsForFloor(2, "2", Collections.emptyList(), null);
        MapConstraintRepository.setActiveFloor(2);

        Integer initializationFloor = invokeResolveStep1InitializationFloor(null);
        assertEquals(Integer.valueOf(2), initializationFloor);

        LatLng fix = converter.toLatLng(3.0, 4.0);
        invokeHandleAbsoluteFixInternal(
                fix.latitude,
                fix.longitude,
                initializationFloor,
                null,
                2_000L,
                5.0f,
                "GNSS",
                0L
        );

        FusedPose bootstrappedPose = getLatestFusedPose();
        assertNotNull(bootstrappedPose);
        assertTrue(getPfInitialized());
        assertTrue(hasPendingConstrainedAbsoluteFix());
        assertEquals(2, bootstrappedPose.getFloor());
        assertEquals("GNSS:init_degraded_bootstrap_map_not_ready", getFieldValue("lastAbsoluteFixDecision"));
        assertFalse((Boolean) getFieldValue("isFloorOffsetInitialized"));
        assertTrue((Boolean) getFieldValue("provisionalFloorBootstrapActive"));
    }

    @Test
    public void mapReadyHoldsFusedPoseUntilTrustedConstrainedStateExists() throws Exception {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        setCoordinateConverter(converter);
        ParticleFilterEngine engine = createDeterministicSensorFusionEngine();
        setParticleFilterEngine(engine);
        engine.setParticlesForTesting(java.util.List.of(
                new Particle(5.0, 0.0, 0, 0.5, 0.0),
                new Particle(-20.0, 0.0, 0, 0.5, 0.0)
        ), 900L);
        setLatestFusedPose(new FusedPose(0.0, 0.0, 0, 1.0, 900L));
        setPfInitialized(true);
        setField("particleCloudTrustedUnderConstraints", false);

        MapConstraintRepository.replaceVenueConstraints("venue", square(converter, -30.0, -10.0, 60.0));
        MapConstraintRepository.setConstraintsForFloor(
                0,
                "0",
                Collections.singletonList(localRectangle(converter, -0.5, -1.0, 1.0, 2.0)),
                null
        );

        sensorFusion.onMapMatchingConstraintsUpdated();

        FusedPose heldPose = getLatestFusedPose();
        assertNotNull(heldPose);
        assertEquals(0.0, heldPose.getX(), 1e-6);
        assertEquals(0.0, heldPose.getY(), 1e-6);
        assertEquals(0, heldPose.getFloor());
        assertFalse(getPfInitialized());
        assertTrue(engine.snapshotParticlesForTesting().isEmpty());
    }

    @Test
    public void mapReadyReplaysPendingFixBeforePublishingNewFusedPose() throws Exception {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        setCoordinateConverter(converter);
        setField("saveRecording", true);
        ParticleFilterEngine engine = createDeterministicSensorFusionEngine();
        setParticleFilterEngine(engine);
        engine.setParticlesForTesting(java.util.List.of(
                new Particle(0.0, 0.0, 0, 0.5, 0.0),
                new Particle(-20.0, 0.0, 0, 0.5, 0.0)
        ), 900L);
        setLatestFusedPose(new FusedPose(0.0, 0.0, 0, 1.0, 900L));
        setPfInitialized(true);
        setField("particleCloudTrustedUnderConstraints", false);

        LatLng deferredFix = converter.toLatLng(5.0, 0.0);
        sensorFusion.handleAbsoluteFix(deferredFix.latitude, deferredFix.longitude, 0, 1_000L, 1.0f);

        assertTrue(hasPendingConstrainedAbsoluteFix());
        assertEquals(2, engine.snapshotParticlesForTesting().size());

        MapConstraintRepository.replaceVenueConstraints("venue", square(converter, -30.0, -10.0, 60.0));
        MapConstraintRepository.setConstraintsForFloor(
                0,
                "0",
                Collections.singletonList(localRectangle(converter, -0.5, -1.0, 1.0, 2.0)),
                null
        );

        sensorFusion.onMapMatchingConstraintsUpdated();

        FusedPose replayedPose = getLatestFusedPose();
        assertNotNull(replayedPose);
        assertFalse(hasPendingConstrainedAbsoluteFix());
        assertTrue(getPfInitialized());
        assertEquals(5.0, replayedPose.getX(), 1e-6);
        assertEquals(0.0, replayedPose.getY(), 1e-6);
        for (Particle particle : engine.snapshotParticlesForTesting()) {
            assertEquals(5.0, particle.getX(), 1e-6);
            assertEquals(0.0, particle.getY(), 1e-6);
            assertFalse(Math.abs(particle.getX()) < 1e-9);
        }
    }

    @Test
    public void strongBarometerEvidenceAllowsNearbyLiftToleranceFallback() throws Exception {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        setCoordinateConverter(converter);
        seedStrongAscendingFloorTransitionEvidence();
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(0, "0", Collections.emptyList(), null);
        MapConstraintRepository.setConstraintsForFloor(1, "1", Collections.emptyList(), null);
        MapConstraintRepository.setLiftsForFloor(0, Collections.singletonList(square(converter, 0.0, 0.0, 2.0)));
        MapConstraintRepository.setLiftsForFloor(1, Collections.singletonList(square(converter, 0.0, 0.0, 2.0)));

        assertTrue(invokeAllowsMapBasedFloorTransition(5.0, 1.0, 5.0, 1.0, 0, 1));
    }

    @Test
    public void limitedFailOpenAllowsAdjacentLiftTransitionWhenCombinedGeometryIsMissing()
            throws Exception {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        setCoordinateConverter(converter);
        seedStrongAscendingFloorTransitionEvidence();
        setLatestFusedPose(new FusedPose(0.5, 0.5, 0, 1.0, System.currentTimeMillis()));
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(0, "0", Collections.emptyList(), null);
        MapConstraintRepository.setConstraintsForFloor(1, "1", Collections.emptyList(), null);
        MapConstraintRepository.setLiftsForFloor(0, Collections.singletonList(square(converter, 0.0, 0.0, 2.0)));
        MapConstraintRepository.setLiftsForFloor(1, Collections.singletonList(square(converter, 0.0, 0.0, 2.0)));
        clearCombinedTransitionGeometryForFloor(0);
        clearCombinedTransitionGeometryForFloor(1);

        assertTrue(invokeShouldFailOpenBarometerFloorTransition(0, 1));
        assertTrue(invokeAllowsMapBasedFloorTransition(0.5, 0.5, 0.5, 0.5, 0, 1));
    }

    @Test
    public void limitedFailOpenStillRejectsInsufficientEvidenceAndIllegalJumps() throws Exception {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        setCoordinateConverter(converter);
        getTracker().reset();
        setLatestFusedPose(new FusedPose(8.0, 8.0, 0, 1.0, System.currentTimeMillis()));
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(0, "0", Collections.emptyList(), null);
        MapConstraintRepository.setConstraintsForFloor(1, "1", Collections.emptyList(), null);
        MapConstraintRepository.setConstraintsForFloor(2, "2", Collections.emptyList(), null);
        MapConstraintRepository.setLiftsForFloor(0, Collections.singletonList(square(converter, 0.0, 0.0, 2.0)));
        MapConstraintRepository.setLiftsForFloor(1, Collections.singletonList(square(converter, 0.0, 0.0, 2.0)));
        clearCombinedTransitionGeometryForFloor(0);
        clearCombinedTransitionGeometryForFloor(1);

        assertFalse(invokeShouldFailOpenBarometerFloorTransition(0, 1));
        assertFalse(invokeAllowsMapBasedFloorTransition(8.0, 8.0, 8.0, 8.0, 0, 1));

        seedStrongAscendingFloorTransitionEvidence();
        assertFalse(invokeShouldFailOpenBarometerFloorTransition(0, 2));
    }

    @Test
    public void strongBarometerEvidenceForceSyncsFloorWhenCloudIsTrapped() throws Exception {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        setCoordinateConverter(converter);
        seedStrongAscendingFloorTransitionEvidence();
        setLatestFusedPose(new FusedPose(8.0, 8.0, 0, 1.0, System.currentTimeMillis()));
        setPfInitialized(true);

        ParticleInitializer.SpawnValidator rejectingMotionValidator = new ParticleInitializer.SpawnValidator() {
            @Override
            public boolean isValid(double x, double y, int floor) {
                return true;
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
                return false;
            }
        };
        ParticleFilterEngine engine = new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                rejectingMotionValidator,
                new ZeroRandom()
        );
        engine.setParticlesForTesting(java.util.List.of(
                new Particle(8.0, 8.0, 0, 0.5, 0.0),
                new Particle(8.0, 8.0, 0, 0.5, 0.0)
        ), 1_000L);
        for (int i = 0; i < 5; i++) {
            engine.predict(new PdrDelta(1.0f, 0.0f, 0.0f), 0, 1_100L + i);
        }
        assertTrue(engine.isCloudTrapped());
        setParticleFilterEngine(engine);

        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(0, "0", Collections.emptyList(), null);
        MapConstraintRepository.setConstraintsForFloor(1, "1", Collections.emptyList(), null);
        MapConstraintRepository.setStairsForFloor(0, Collections.singletonList(square(converter, 0.0, 0.0, 2.0)));
        MapConstraintRepository.setStairsForFloor(1, Collections.singletonList(square(converter, 0.0, 0.0, 2.0)));

        invokeSyncFusedFloorFromBarometer(0, 1, 2_000L);

        FusedPose syncedPose = getLatestFusedPose();
        assertNotNull(syncedPose);
        assertEquals(1, syncedPose.getFloor());
        assertEquals("forced_by_barometer_rescue", getFieldValue("lastFloorConsensus"));
    }

    @Test
    public void weakBarometerEvidenceDoesNotUseToleranceFallback() throws Exception {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        setCoordinateConverter(converter);
        getTracker().reset();
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(0, "0", Collections.emptyList(), null);
        MapConstraintRepository.setConstraintsForFloor(1, "1", Collections.emptyList(), null);
        MapConstraintRepository.setLiftsForFloor(0, Collections.singletonList(square(converter, 0.0, 0.0, 2.0)));
        MapConstraintRepository.setLiftsForFloor(1, Collections.singletonList(square(converter, 0.0, 0.0, 2.0)));

        assertFalse(invokeAllowsMapBasedFloorTransition(5.0, 1.0, 5.0, 1.0, 0, 1));
    }

    @Test
    public void floorOnlySyncKeepsXyAndArmsDisplayReset() throws Exception {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        setCoordinateConverter(converter);
        ParticleFilterEngine engine = new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                ParticleInitializer.allowAll(),
                new ZeroRandom()
        );
        engine.initialize(5.0, 1.0, 0, 1_000L, 0.0, 32, 0.0);
        setParticleFilterEngine(engine);
        setLatestFusedPose(new FusedPose(5.0, 1.0, 0, 1.0, 1_000L));
        setPfInitialized(true);

        MapConstraintRepository.replaceVenueConstraints("venue", square(converter, -30.0, -10.0, 60.0));
        MapConstraintRepository.setConstraintsForFloor(
                0,
                "0",
                Collections.singletonList(localRectangle(converter, 20.0, 20.0, 1.0, 1.0)),
                null
        );
        MapConstraintRepository.setConstraintsForFloor(
                1,
                "1",
                Collections.singletonList(localRectangle(converter, 20.0, 22.0, 1.0, 1.0)),
                null
        );

        assertTrue(invokeApplyAbsoluteFloorKeepingCurrentXy(1, 2_000L));
        invokeActivateFloorOnlyResyncWindow(1, 2_000L);

        FusedPose syncedPose = getLatestFusedPose();
        assertEquals(1, syncedPose.getFloor());
        assertEquals(5.0, syncedPose.getX(), 1e-6);
        assertEquals(1.0, syncedPose.getY(), 1e-6);

        FusedPose syncedParticlePose = engine.estimatePose();
        assertEquals(1, syncedParticlePose.getFloor());
        assertEquals(5.0, syncedParticlePose.getX(), 1e-6);
        assertEquals(1.0, syncedParticlePose.getY(), 1e-6);
        assertTrue(sensorFusion.consumePendingDisplayFloorReset(1, 2_000L));
    }

    @Test
    public void floorOnlySyncRejectsIllegalTargetFloorAndKeepsCurrentPose() throws Exception {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        setCoordinateConverter(converter);
        ParticleFilterEngine engine = createDeterministicSensorFusionEngine();
        setParticleFilterEngine(engine);
        engine.setParticlesForTesting(java.util.List.of(
                new Particle(5.0, 1.0, 0, 1.0, 0.0)
        ), 1_000L);
        setLatestFusedPose(new FusedPose(5.0, 1.0, 0, 1.0, 1_000L));
        setPfInitialized(true);

        MapConstraintRepository.replaceVenueConstraints("venue", square(converter, -30.0, -10.0, 60.0));
        MapConstraintRepository.setConstraintsForFloor(
                0,
                "0",
                Collections.singletonList(localRectangle(converter, 20.0, 20.0, 1.0, 1.0)),
                null
        );
        MapConstraintRepository.setConstraintsForFloor(
                1,
                "1",
                Collections.singletonList(localRectangle(converter, 4.5, 0.5, 1.0, 1.0)),
                null
        );

        assertFalse(invokeApplyAbsoluteFloorKeepingCurrentXy(1, 2_000L));

        FusedPose pose = getLatestFusedPose();
        assertNotNull(pose);
        assertEquals(0, pose.getFloor());
        assertEquals(5.0, pose.getX(), 1e-6);
        assertEquals(1.0, pose.getY(), 1e-6);

        FusedPose particlePose = engine.estimatePose();
        assertNotNull(particlePose);
        assertEquals(0, particlePose.getFloor());
        assertEquals(5.0, particlePose.getX(), 1e-6);
        assertEquals(1.0, particlePose.getY(), 1e-6);
    }

    private void seedStrongAscendingFloorTransitionEvidence() throws Exception {
        CumulativeFloorTracker tracker = getTracker();
        tracker.reset();
        long now = System.currentTimeMillis();
        long baseTs = now - 4_000L;
        tracker.update(0f, 4.0f, baseTs);
        tracker.update(3.60f, 4.0f, baseTs + 1_000L);
        tracker.update(3.80f, 4.0f, baseTs + 2_000L);
        tracker.update(3.80f, 4.0f, baseTs + 3_000L);
    }

    @SuppressWarnings("unchecked")
    private void clearCombinedTransitionGeometryForFloor(int floor) throws Exception {
        Field field = MapConstraintRepository.class.getDeclaredField("transitionsByFloor");
        field.setAccessible(true);
        ((Map<Integer, ?>) field.get(null)).remove(floor);
    }

    private boolean invokeShouldFailOpenBarometerFloorTransition(int previousFloor, int newFloor)
            throws Exception {
        Method method = SensorFusion.class.getDeclaredMethod(
                "shouldFailOpenBarometerFloorTransition",
                int.class,
                int.class
        );
        method.setAccessible(true);
        return (boolean) method.invoke(sensorFusion, previousFloor, newFloor);
    }

    private void invokeSyncFusedFloorFromBarometer(
            int previousFloor,
            int newFloor,
            long timestampMs
    ) throws Exception {
        Method method = SensorFusion.class.getDeclaredMethod(
                "syncFusedFloorFromBarometer",
                int.class,
                int.class,
                long.class
        );
        method.setAccessible(true);
        method.invoke(sensorFusion, previousFloor, newFloor, timestampMs);
    }

    private boolean invokeIsValidParticleMotion(
            double previousX,
            double previousY,
            double predictedX,
            double predictedY,
            int previousFloor,
            int predictedFloor
    ) throws Exception {
        Method method = SensorFusion.class.getDeclaredMethod(
                "isValidParticleMotion",
                double.class,
                double.class,
                double.class,
                double.class,
                int.class,
                int.class
        );
        method.setAccessible(true);
        return (boolean) method.invoke(
                sensorFusion,
                previousX,
                previousY,
                predictedX,
                predictedY,
                previousFloor,
                predictedFloor
        );
    }

    private boolean invokeIsValidParticlePrediction(double x, double y, int floor) throws Exception {
        Method method = SensorFusion.class.getDeclaredMethod(
                "isValidParticlePrediction",
                double.class,
                double.class,
                int.class
        );
        method.setAccessible(true);
        return (boolean) method.invoke(sensorFusion, x, y, floor);
    }

    private boolean invokeAllowsMapBasedFloorTransition(
            double previousX,
            double previousY,
            double predictedX,
            double predictedY,
            int previousFloor,
            int newFloor
    ) throws Exception {
        Method method = SensorFusion.class.getDeclaredMethod(
                "allowsMapBasedFloorTransition",
                double.class,
                double.class,
                double.class,
                double.class,
                int.class,
                int.class
        );
        method.setAccessible(true);
        return (boolean) method.invoke(
                sensorFusion,
                previousX,
                previousY,
                predictedX,
                predictedY,
                previousFloor,
                newFloor
        );
    }

    private ParticleFilterEngine createDeterministicSensorFusionEngine() {
        ParticleInitializer.SpawnValidator validator = new ParticleInitializer.SpawnValidator() {
            @Override
            public boolean isValid(double x, double y, int floor) {
                try {
                    return invokeIsValidParticlePrediction(x, y, floor);
                } catch (Exception e) {
                    throw new RuntimeException(e);
                }
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
                try {
                    return invokeIsValidParticleMotion(
                            previousX,
                            previousY,
                            predictedX,
                            predictedY,
                            previousFloor,
                            predictedFloor
                    );
                } catch (Exception e) {
                    throw new RuntimeException(e);
                }
            }
        };
        return new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                validator,
                new ZeroRandom()
        );
    }

    private CumulativeFloorTracker getTracker() throws Exception {
        Field field = SensorFusion.class.getDeclaredField("cumulativeFloorTracker");
        field.setAccessible(true);
        return (CumulativeFloorTracker) field.get(sensorFusion);
    }

    private void setCoordinateConverter(CoordinateConverter converter) throws Exception {
        Field field = SensorFusion.class.getDeclaredField("coordinateConverter");
        field.setAccessible(true);
        field.set(sensorFusion, converter);
    }

    private void setParticleFilterEngine(ParticleFilterEngine engine) throws Exception {
        Field field = SensorFusion.class.getDeclaredField("particleFilterEngine");
        field.setAccessible(true);
        field.set(sensorFusion, engine);
    }

    private ParticleFilterEngine getParticleFilterEngine() throws Exception {
        Field field = SensorFusion.class.getDeclaredField("particleFilterEngine");
        field.setAccessible(true);
        return (ParticleFilterEngine) field.get(sensorFusion);
    }

    private void setLatestFusedPose(FusedPose fusedPose) throws Exception {
        Field field = SensorFusion.class.getDeclaredField("latestFusedPose");
        field.setAccessible(true);
        field.set(sensorFusion, fusedPose);
    }

    private FusedPose getLatestFusedPose() throws Exception {
        Field field = SensorFusion.class.getDeclaredField("latestFusedPose");
        field.setAccessible(true);
        return (FusedPose) field.get(sensorFusion);
    }

    private void setPfInitialized(boolean value) throws Exception {
        Field field = SensorFusion.class.getDeclaredField("pfInitialized");
        field.setAccessible(true);
        field.set(sensorFusion, value);
    }

    private boolean getPfInitialized() throws Exception {
        Field field = SensorFusion.class.getDeclaredField("pfInitialized");
        field.setAccessible(true);
        return (boolean) field.get(sensorFusion);
    }

    private boolean hasPendingConstrainedAbsoluteFix() throws Exception {
        Field field = SensorFusion.class.getDeclaredField("pendingConstrainedAbsoluteFix");
        field.setAccessible(true);
        return field.get(sensorFusion) != null;
    }

    private boolean getParticleCloudTrustedUnderConstraints() throws Exception {
        Field field = SensorFusion.class.getDeclaredField("particleCloudTrustedUnderConstraints");
        field.setAccessible(true);
        return field.getBoolean(sensorFusion);
    }

    private Object getFieldValue(String fieldName) throws Exception {
        Field field = SensorFusion.class.getDeclaredField(fieldName);
        field.setAccessible(true);
        return field.get(sensorFusion);
    }

    private void setField(String fieldName, Object value) throws Exception {
        Field field = SensorFusion.class.getDeclaredField(fieldName);
        field.setAccessible(true);
        field.set(sensorFusion, value);
    }

    private void setAbsoluteObservationField(
            String fieldName,
            double x,
            double y,
            Integer floor,
            float accuracyMeters,
            long timestampMs
    ) throws Exception {
        Class<?> snapshotClass = Class.forName(
                "com.openpositioning.PositionMe.sensors.SensorFusion$AbsoluteObservationSnapshot"
        );
        java.lang.reflect.Constructor<?> constructor = snapshotClass.getDeclaredConstructor(
                double.class,
                double.class,
                Integer.class,
                float.class,
                long.class
        );
        constructor.setAccessible(true);
        Object snapshot = constructor.newInstance(x, y, floor, accuracyMeters, timestampMs);
        setField(fieldName, snapshot);
    }

    private boolean invokeApplyAbsoluteFloorKeepingCurrentXy(
            int absoluteFloor,
            long timestampMs
    ) throws Exception {
        Method method = SensorFusion.class.getDeclaredMethod(
                "applyAbsoluteFloorKeepingCurrentXy",
                int.class,
                long.class
        );
        method.setAccessible(true);
        return (boolean) method.invoke(sensorFusion, absoluteFloor, timestampMs);
    }

    private void invokeActivateFloorOnlyResyncWindow(
            int absoluteFloor,
            long timestampMs
    ) throws Exception {
        Method method = SensorFusion.class.getDeclaredMethod(
                "activateFloorOnlyResyncWindow",
                int.class,
                long.class
        );
        method.setAccessible(true);
        method.invoke(sensorFusion, absoluteFloor, timestampMs);
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

    private Integer invokeResolveStep1InitializationFloor(Integer preferredFloor) throws Exception {
        Method method = SensorFusion.class.getDeclaredMethod(
                "resolveStep1InitializationFloor",
                Integer.class
        );
        method.setAccessible(true);
        return (Integer) method.invoke(sensorFusion, preferredFloor);
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

    private static java.util.List<LatLng> localRectangle(
            CoordinateConverter converter,
            double westXMeters,
            double southYMeters,
            double widthMeters,
            double heightMeters
    ) {
        return java.util.List.of(
                converter.toLatLng(westXMeters, southYMeters),
                converter.toLatLng(westXMeters + widthMeters, southYMeters),
                converter.toLatLng(westXMeters + widthMeters, southYMeters + heightMeters),
                converter.toLatLng(westXMeters, southYMeters + heightMeters)
        );
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
}
