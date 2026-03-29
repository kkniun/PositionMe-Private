package com.openpositioning.PositionMe.sensors;

import com.openpositioning.PositionMe.utils.CoordinateConverter;
import com.openpositioning.PositionMe.utils.MapConstraintRepository;

import org.junit.After;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.Collections;
import java.util.Random;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

public class SensorFusionInitialFloorBootstrapTest {

    private final SensorFusion sensorFusion = SensorFusion.getInstance();

    @After
    public void tearDown() throws Exception {
        MapConstraintRepository.clear();
        setField("pendingWifiBootstrapFloor", null);
        setField("pendingWifiBootstrapCount", 0);
        setField("pendingWifiBootstrapFirstTimestampMs", Long.MIN_VALUE);
        setField("lastWifiBootstrapCandidateTimestampMs", Long.MIN_VALUE);
        setField("pendingStableAbsoluteFloorCandidate", null);
        setField("pendingStableAbsoluteFloorCount", 0);
        setField("pendingStableAbsoluteFloorFirstTimestampMs", Long.MIN_VALUE);
        setField("pendingStableAbsoluteFloorLastTimestampMs", Long.MIN_VALUE);
        setField("isFloorOffsetInitialized", false);
        setField("pdrFloorOffset", 0);
        setField("latestFusedPose", null);
        setField("coordinateConverter", null);
        setField("provisionalFloorBootstrapActive", false);
        setField("provisionalFloorBootstrapFloor", Integer.MIN_VALUE);
        setField("provisionalFloorBootstrapSource", "none");
        setField("allowProvisionalFloorReconcileForCurrentFix", false);
        setField("provisionalFloorReconcileTargetFloor", Integer.MIN_VALUE);
    }

    @Test
    public void stationaryWifiBootstrapAllowedBeforeFloorOffsetIsInitialized() {
        assertTrue(SensorFusion.shouldAcceptStationaryWifiFloorBootstrap(1, 0, false));
    }

    @Test
    public void stationaryWifiBootstrapAllowedWhenReportedFloorDiffersFromCurrentFloor() {
        assertTrue(SensorFusion.shouldAcceptStationaryWifiFloorBootstrap(1, 0, true));
    }

    @Test
    public void stationaryWifiBootstrapRejectedWhenSameFloorAndAlreadyInitialized() {
        assertFalse(SensorFusion.shouldAcceptStationaryWifiFloorBootstrap(0, 0, true));
    }

    @Test
    public void stationaryWifiBootstrapRejectedWhenFloorIsUnknown() {
        assertFalse(SensorFusion.shouldAcceptStationaryWifiFloorBootstrap(null, 0, false));
    }

    @Test
    public void forceWifiBootstrapWhenPfWasSeededWithoutAbsoluteFloor() {
        assertTrue(SensorFusion.shouldForceWifiFloorBootstrap("WIFI", 1, true, false, 3, 8f));
    }

    @Test
    public void doesNotForceWifiBootstrapAfterAbsoluteFloorWasInitialized() {
        assertFalse(SensorFusion.shouldForceWifiFloorBootstrap("WIFI", 1, true, true, 3, 8f));
        assertFalse(SensorFusion.shouldForceWifiFloorBootstrap("GNSS", 1, true, false, 3, 8f));
        assertFalse(SensorFusion.shouldForceWifiFloorBootstrap("WIFI", null, true, false, 3, 8f));
        assertFalse(SensorFusion.shouldForceWifiFloorBootstrap("WIFI", 1, true, false, 2, 8f));
    }

    @Test
    public void highConfidenceWifiFixStillRequiresBootstrapConsensus() {
        assertFalse(SensorFusion.shouldForceWifiFloorBootstrap("WIFI", 1, true, false, 1, 4.5f));
    }

    @Test
    public void nonGroundFloorBootstrapBehaviorRemainsUnchanged() {
        sensorFusion.resetWifiFloorBootstrapConsensusForTesting();

        assertFalse(sensorFusion.shouldAcceptWifiBootstrapConsensus(2, 8f, 1_000L));
        assertFalse(sensorFusion.shouldAcceptWifiBootstrapConsensus(2, 8f, 2_000L));
        assertTrue(sensorFusion.shouldAcceptWifiBootstrapConsensus(2, 8f, 3_000L));
    }

    @Test
    public void wifiBootstrapConsensusResetsWhenFloorChanges() {
        sensorFusion.resetWifiFloorBootstrapConsensusForTesting();

        assertFalse(sensorFusion.shouldAcceptWifiBootstrapConsensus(0, 8f, 1_000L));
        assertFalse(sensorFusion.shouldAcceptWifiBootstrapConsensus(0, 8f, 2_000L));
        assertFalse(sensorFusion.shouldAcceptWifiBootstrapConsensus(1, 8f, 3_000L));
        assertFalse(sensorFusion.shouldAcceptWifiBootstrapConsensus(1, 8f, 4_000L));
        assertTrue(sensorFusion.shouldAcceptWifiBootstrapConsensus(1, 8f, 5_000L));
    }

    @Test
    public void wifiUnknownFloorDoesNotTriggerBootstrapConsensus() {
        assertNull(invokeResolveBootstrapReadyFloorPrior(null, 8f, 1_000L));
        assertNull(invokeResolveBootstrapReadyFloorPrior(null, 8f, 2_000L));
        assertNull(invokeResolveBootstrapReadyFloorPrior(null, 8f, 3_000L));
    }

    @Test
    public void untrustedBootstrapDoesNotReuseDefaultAbsoluteFloorAsInitializationSeed() throws Exception {
        setField("isFloorOffsetInitialized", false);
        setField("pdrFloorOffset", 0);

        assertEquals(Integer.valueOf(2), invokeResolveStep1InitializationFloor(2));
        assertNull(invokeResolveStep1InitializationFloor(null));

        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(1, "1", Collections.emptyList(), null);
        setField("pendingStableAbsoluteFloorCandidate", 1);
        assertEquals(Integer.valueOf(1), invokeResolveStep1InitializationFloor(null));
    }

    @Test
    public void unknownInitializationSeedDoesNotInitializePfOnFloorZero() throws Exception {
        setField("isFloorOffsetInitialized", false);
        setField("pdrFloorOffset", 0);
        setField("pendingStableAbsoluteFloorCandidate", null);
        setField("pendingWifiBootstrapFloor", null);
        setField("latestFusedPose", null);

        assertNull(invokeResolveStep1InitializationFloor(null));

        CoordinateConverter converter = new CoordinateConverter(55.9444, -3.1878);
        ParticleFilterEngine engine = createDeterministicEngine();
        FusedPose pose = SensorFusion.applyAbsoluteFixForStep1(
                engine,
                converter,
                false,
                55.9445,
                -3.1877,
                null,
                null,
                1_000L,
                5.0f,
                0.0f
        );

        assertNull(pose);
        assertFalse(engine.hasParticles());
    }

    @Test
    public void activeMapFloorProvidesOnlyUntrustedBootstrapSeedWhenBarometerBaselineExists() throws Exception {
        setField("isFloorOffsetInitialized", false);
        setField("pdrFloorOffset", 0);
        setField("pendingStableAbsoluteFloorCandidate", null);
        setField("pendingWifiBootstrapFloor", null);
        setField("latestFusedPose", null);
        setRelativeFloor(0);

        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(2, "2", Collections.emptyList(), null);
        MapConstraintRepository.setActiveFloor(2);
        setCumulativeFloorTrackerInitialized(true);

        assertEquals(Integer.valueOf(2), invokeResolveStep1InitializationFloor(null));
        assertFalse((Boolean) getField("isFloorOffsetInitialized"));
    }

    @Test
    public void provisionalBootstrapSeedRemainsUntrustedUntilFloorLockIsEstablished() throws Exception {
        invokeNoteBootstrapFloorSeed(0, false, "fallback_seed", 1_000L);

        assertTrue((Boolean) getField("provisionalFloorBootstrapActive"));
        assertEquals(0, getField("provisionalFloorBootstrapFloor"));
        assertEquals("fallback_seed", getField("provisionalFloorBootstrapSource"));
        assertFalse((Boolean) getField("isFloorOffsetInitialized"));
    }

    @Test
    public void provisionalFloorReconcileAllowsTrustedWifiFloorToReplaceBootstrapFloor() throws Exception {
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(0, "0", Collections.emptyList(), null);
        MapConstraintRepository.setConstraintsForFloor(1, "1", Collections.emptyList(), null);

        CoordinateConverter converter = new CoordinateConverter(55.9444, -3.1878);
        setField("coordinateConverter", converter);
        setField("latestFusedPose", new FusedPose(0.0, 0.0, 0, 0.15, 1_000L));
        setField("provisionalFloorBootstrapActive", true);
        setField("provisionalFloorBootstrapFloor", 0);
        setField("provisionalFloorBootstrapSource", "fallback_seed");
        setField("allowProvisionalFloorReconcileForCurrentFix", true);
        setField("provisionalFloorReconcileTargetFloor", 1);

        FusedPose previousPose = new FusedPose(0.0, 0.0, 0, 0.15, 1_000L);
        FusedPose candidatePose = new FusedPose(0.0, 0.0, 1, 0.55, 2_000L);

        FusedPose resolvedPose = invokeResolveConstrainedFusedPose(previousPose, candidatePose, "WIFI");

        assertSame(candidatePose, resolvedPose);
    }

    @Test
    public void strongBarometerConflictBlocksProvisionalFloorReconcile() throws Exception {
        seedKnownFloors();
        setField("pfInitialized", true);
        setField("isFloorOffsetInitialized", false);
        setField("provisionalFloorBootstrapActive", true);
        setField("provisionalFloorBootstrapFloor", 0);
        setField("provisionalFloorBootstrapSource", "fallback_seed");
        setTrackerField("initialized", true);
        setTrackerField("lastTransitionDirection", -1);
        setTrackerField("lastTransitionStrong", true);
        setTrackerField("lastTransitionTimestampMs", 5_000L);

        assertNull(invokeResolveAcceptedAbsoluteFloorPrior(1, 6_000L));
        assertEquals("initial_lock_pending", getField("lastFloorConsensus"));
    }

    @Test
    public void trustedIllegalFloorTransitionStillRejectsWithoutProvisionalBypass() throws Exception {
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(
                0,
                "0",
                Collections.singletonList(Arrays.asList(
                        new com.google.android.gms.maps.model.LatLng(55.9450, -3.1885),
                        new com.google.android.gms.maps.model.LatLng(55.9450, -3.1884),
                        new com.google.android.gms.maps.model.LatLng(55.9449, -3.1884)
                )),
                null
        );
        MapConstraintRepository.setConstraintsForFloor(
                1,
                "1",
                Collections.singletonList(Arrays.asList(
                        new com.google.android.gms.maps.model.LatLng(55.9450, -3.1885),
                        new com.google.android.gms.maps.model.LatLng(55.9450, -3.1884),
                        new com.google.android.gms.maps.model.LatLng(55.9449, -3.1884)
                )),
                null
        );

        CoordinateConverter converter = new CoordinateConverter(55.9444, -3.1878);
        setField("coordinateConverter", converter);
        setField("latestFusedPose", new FusedPose(0.0, 0.0, 0, 0.80, 1_000L));
        setField("isFloorOffsetInitialized", true);

        FusedPose previousPose = new FusedPose(0.0, 0.0, 0, 0.80, 1_000L);
        FusedPose candidatePose = new FusedPose(0.0, 0.0, 1, 0.85, 2_000L);

        FusedPose resolvedPose = invokeResolveConstrainedFusedPose(previousPose, candidatePose, "WIFI");

        assertSame(previousPose, resolvedPose);
    }

    @Test
    public void untrustedFloorCandidateCannotInitializeFloorCalibration() throws Exception {
        setField("isFloorOffsetInitialized", false);
        setField("pdrFloorOffset", 4);

        invokeMaybeCalibrateFloorOffset(0, false, 2_000L);

        assertFalse((Boolean) getField("isFloorOffsetInitialized"));
        assertEquals(4, getField("pdrFloorOffset"));
    }

    @Test
    public void groundFloorBootstrapRequiresStrongerEvidenceThanDefaultConsensus() {
        sensorFusion.resetWifiFloorBootstrapConsensusForTesting();

        assertFalse(sensorFusion.shouldAcceptWifiBootstrapConsensus(0, 8f, 1_000L));
        assertFalse(sensorFusion.shouldAcceptWifiBootstrapConsensus(0, 8f, 2_000L));
        assertFalse(sensorFusion.shouldAcceptWifiBootstrapConsensus(0, 8f, 3_000L));
        assertFalse(sensorFusion.shouldAcceptWifiBootstrapConsensus(0, 8f, 4_000L));
        assertFalse(sensorFusion.shouldAcceptWifiBootstrapConsensus(0, 8f, 5_000L));
        assertTrue(sensorFusion.shouldAcceptWifiBootstrapConsensus(0, 8f, 7_000L));
    }

    @Test
    public void trustedGroundFloorBootstrapStillSucceedsWhenEvidenceIsStrong() throws Exception {
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(0, "GF", Collections.emptyList(), null);

        assertTrue(sensorFusion.isGroundFloorWifiBootstrapContextReady());

        sensorFusion.resetWifiFloorBootstrapConsensusForTesting();
        assertFalse(sensorFusion.shouldAcceptWifiBootstrapConsensus(0, 8f, 1_000L));
        assertFalse(sensorFusion.shouldAcceptWifiBootstrapConsensus(0, 8f, 2_000L));
        assertFalse(sensorFusion.shouldAcceptWifiBootstrapConsensus(0, 8f, 3_000L));
        assertFalse(sensorFusion.shouldAcceptWifiBootstrapConsensus(0, 8f, 4_000L));
        assertFalse(sensorFusion.shouldAcceptWifiBootstrapConsensus(0, 8f, 5_000L));
        assertTrue(sensorFusion.shouldAcceptWifiBootstrapConsensus(0, 8f, 7_000L));
    }

    @Test
    public void groundFloorBootstrapRejectedWhenVenueDoesNotExposeGroundFloor() {
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(1, "1", Collections.emptyList(), null);

        assertFalse(sensorFusion.isGroundFloorWifiBootstrapContextReady());
    }

    @Test
    public void groundFloorBootstrapCandidateIsSkippedWithoutTrustedVenueContext() {
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(1, "1", Collections.emptyList(), null);

        assertNull(invokeResolveBootstrapReadyFloorPrior(0, 8f, 1_000L));
        assertNull(invokeResolveBootstrapReadyFloorPrior(0, 8f, 2_000L));
        assertNull(invokeResolveBootstrapReadyFloorPrior(0, 8f, 3_000L));
        assertNull(invokeResolveBootstrapReadyFloorPrior(0, 8f, 7_000L));
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

    private void setRelativeFloor(int relativeFloor) throws Exception {
        Field trackerField = SensorFusion.class.getDeclaredField("cumulativeFloorTracker");
        trackerField.setAccessible(true);
        Object tracker = trackerField.get(sensorFusion);
        Field relativeFloorField = tracker.getClass().getDeclaredField("relativeFloor");
        relativeFloorField.setAccessible(true);
        relativeFloorField.setInt(tracker, relativeFloor);
    }

    private void setCumulativeFloorTrackerInitialized(boolean initialized) throws Exception {
        Field trackerField = SensorFusion.class.getDeclaredField("cumulativeFloorTracker");
        trackerField.setAccessible(true);
        Object tracker = trackerField.get(sensorFusion);
        Field initializedField = tracker.getClass().getDeclaredField("initialized");
        initializedField.setAccessible(true);
        initializedField.setBoolean(tracker, initialized);
    }

    private Integer invokeResolveBootstrapReadyFloorPrior(
            Integer floorPrior,
            float accuracyMeters,
            long timestampMs
    ) {
        try {
            Method method = SensorFusion.class.getDeclaredMethod(
                    "resolveBootstrapReadyFloorPrior",
                    String.class,
                    Integer.class,
                    float.class,
                    long.class
            );
            method.setAccessible(true);
            return (Integer) method.invoke(sensorFusion, "WIFI", floorPrior, accuracyMeters, timestampMs);
        } catch (Exception e) {
            throw new AssertionError(e);
        }
    }

    private Integer invokeResolveAcceptedAbsoluteFloorPrior(int reportedFloor, long timestampMs) {
        try {
            Method method = SensorFusion.class.getDeclaredMethod(
                    "resolveAcceptedAbsoluteFloorPrior",
                    double.class,
                    double.class,
                    Integer.class,
                    long.class
            );
            method.setAccessible(true);
            return (Integer) method.invoke(sensorFusion, 55.9444, -3.1878, reportedFloor, timestampMs);
        } catch (Exception e) {
            throw new AssertionError(e);
        }
    }

    private Integer invokeResolveStep1InitializationFloor(Integer preferredFloor) {
        try {
            Method method = SensorFusion.class.getDeclaredMethod(
                    "resolveStep1InitializationFloor",
                    Integer.class
            );
            method.setAccessible(true);
            return (Integer) method.invoke(sensorFusion, preferredFloor);
        } catch (Exception e) {
            throw new AssertionError(e);
        }
    }

    private FusedPose invokeResolveConstrainedFusedPose(
            FusedPose previousPose,
            FusedPose candidatePose,
            String debugSource
    ) {
        try {
            Method method = SensorFusion.class.getDeclaredMethod(
                    "resolveConstrainedFusedPose",
                    FusedPose.class,
                    FusedPose.class,
                    String.class,
                    PdrDelta.class
            );
            method.setAccessible(true);
            return (FusedPose) method.invoke(sensorFusion, previousPose, candidatePose, debugSource, null);
        } catch (Exception e) {
            throw new AssertionError(e);
        }
    }

    private void invokeNoteBootstrapFloorSeed(
            int floor,
            boolean trustedFloorSeed,
            String source,
            long timestampMs
    ) {
        try {
            Method method = SensorFusion.class.getDeclaredMethod(
                    "noteBootstrapFloorSeed",
                    int.class,
                    boolean.class,
                    String.class,
                    long.class
            );
            method.setAccessible(true);
            method.invoke(sensorFusion, floor, trustedFloorSeed, source, timestampMs);
        } catch (Exception e) {
            throw new AssertionError(e);
        }
    }

    private void invokeMaybeCalibrateFloorOffset(
            Integer absoluteFloor,
            boolean trustedAbsoluteFloor,
            long timestampMs
    ) {
        try {
            Method method = SensorFusion.class.getDeclaredMethod(
                    "maybeCalibrateFloorOffset",
                    Integer.class,
                    boolean.class,
                    long.class
            );
            method.setAccessible(true);
            method.invoke(sensorFusion, absoluteFloor, trustedAbsoluteFloor, timestampMs);
        } catch (Exception e) {
            throw new AssertionError(e);
        }
    }

    private ParticleFilterEngine createDeterministicEngine() {
        return new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                ParticleInitializer.allowAll(),
                new ZeroRandom()
        );
    }

    private void seedKnownFloors() {
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(0, "0", Collections.emptyList(), null);
        MapConstraintRepository.setConstraintsForFloor(1, "1", Collections.emptyList(), null);
    }

    private void setTrackerField(String fieldName, Object value) throws Exception {
        Field trackerField = SensorFusion.class.getDeclaredField("cumulativeFloorTracker");
        trackerField.setAccessible(true);
        Object tracker = trackerField.get(sensorFusion);
        Field field = tracker.getClass().getDeclaredField(fieldName);
        field.setAccessible(true);
        field.set(tracker, value);
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
