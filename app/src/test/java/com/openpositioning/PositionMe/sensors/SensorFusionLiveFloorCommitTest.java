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

public class SensorFusionLiveFloorCommitTest {

    private static final double ORIGIN_LAT = 55.9444;
    private static final double ORIGIN_LON = -3.1878;

    private final SensorFusion sensorFusion = SensorFusion.getInstance();

    @After
    public void tearDown() throws Exception {
        MapConstraintRepository.clear();
        sensorFusion.resetAbsoluteAnchorStateForTesting();
        setField("coordinateConverter", null);
        setField("latestFusedPose", null);
        setField("particleFilterEngine", null);
        setField("pfInitialized", false);
        setField("isFloorOffsetInitialized", false);
        setField("pdrFloorOffset", 0);
        setField("liveCurrentFloorAbsolute", Integer.MIN_VALUE);
        setField("committedDisplayFloorAbsolute", Integer.MIN_VALUE);
        setField("pendingCommittedDisplayFloorAbsolute", Integer.MIN_VALUE);
        setField("floorSwitchPending", false);
        setField("lastDisplayedFusedMarkerLatLng", null);
        setField("latitude", 0f);
        setField("longitude", 0f);
    }

    @Test
    public void committedFloorTransitionSynchronizesLiveAndVisibleFloor() throws Exception {
        CoordinateConverter converter = seedOpenFloors();
        setField("coordinateConverter", converter);
        setField("latestFusedPose", new FusedPose(2.0, 2.0, 1, 0.90, 1_000L));
        setField("liveCurrentFloorAbsolute", 1);

        boolean committed = invokeCommitLiveFloorTransition(2, 2_000L, null, "test_floor_commit");

        assertTrue(committed);
        assertEquals(2, sensorFusion.getCurrentFloor());
        assertEquals(2, sensorFusion.getUserVisibleFloor());
        FusedPose fusedPose = sensorFusion.getLatestFusedPose();
        assertNotNull(fusedPose);
        assertEquals(2, fusedPose.getFloor());
    }

    @Test
    public void reverseFloorTransitionReseedsToTargetTransitionAnchorWhenOldFloorPoseIsIllegal()
            throws Exception {
        CoordinateConverter converter = seedReverseTransitionFloors();
        setField("coordinateConverter", converter);
        setField("latestFusedPose", new FusedPose(8.0, 8.0, 2, 0.80, 3_000L));
        setField("liveCurrentFloorAbsolute", 2);

        boolean committed = invokeCommitLiveFloorTransition(1, 4_000L, null, "test_reverse_commit");

        assertTrue(committed);
        assertEquals(1, sensorFusion.getCurrentFloor());
        assertEquals(1, sensorFusion.getUserVisibleFloor());
        FusedPose fusedPose = sensorFusion.getLatestFusedPose();
        assertNotNull(fusedPose);
        assertEquals(1, fusedPose.getFloor());
        assertTrue(Math.abs(fusedPose.getX() - 1.0) < 2.5);
        assertTrue(Math.abs(fusedPose.getY() - 1.0) < 2.5);
    }

    @Test
    public void reverseFloorTransitionFallsBackToLegalGnssReferenceWhenDisplayedPoseIsStale()
            throws Exception {
        CoordinateConverter converter = seedReverseTransitionFloors();
        setField("coordinateConverter", converter);
        setField("latestFusedPose", new FusedPose(8.0, 8.0, 2, 0.80, 3_000L));
        setField("liveCurrentFloorAbsolute", 2);
        setField("lastDisplayedFusedMarkerLatLng", converter.toLatLng(8.0, 8.0));
        LatLng targetFloorReference = converter.toLatLng(1.0, 1.0);
        setField("latitude", (float) targetFloorReference.latitude);
        setField("longitude", (float) targetFloorReference.longitude);

        boolean committed = invokeCommitLiveFloorTransition(1, 4_000L, null, "test_gnss_fallback");

        assertTrue(committed);
        FusedPose fusedPose = sensorFusion.getLatestFusedPose();
        assertNotNull(fusedPose);
        assertEquals(1, fusedPose.getFloor());
        assertTrue(Math.abs(fusedPose.getX() - 1.0) < 0.25);
        assertTrue(Math.abs(fusedPose.getY() - 1.0) < 0.25);
    }

    @Test
    public void displayFloorCommitCanRecoverAfterViewStateLossWhenAuthoritativeFloorAlreadyChanged()
            throws Exception {
        setField("latestFusedPose", new FusedPose(1.0, 1.0, 2, 0.75, 4_000L));
        setField("liveCurrentFloorAbsolute", 2);
        setField("committedDisplayFloorAbsolute", 1);
        setField("pendingCommittedDisplayFloorAbsolute", Integer.MIN_VALUE);
        setField("floorSwitchPending", false);

        assertTrue(sensorFusion.isDisplayFloorChangeReady(2, 5_000L));
        assertTrue(sensorFusion.commitDisplayFloorIfReady(2, 5_000L, null));
        assertEquals(2, getIntField("committedDisplayFloorAbsolute"));
    }

    @Test
    public void baselineRecalibrationNeverLeavesLiveFloorSplitWhenCommitCannotMigratePose()
            throws Exception {
        setField("coordinateConverter", null);
        setField("latestFusedPose", new FusedPose(8.0, 8.0, 2, 0.60, 3_000L));
        setField("liveCurrentFloorAbsolute", 2);

        invokeRecalibrateAbsoluteFloorBaselineInternal(1, 4_000L);

        assertEquals(2, sensorFusion.getCurrentFloor());
        assertEquals(2, sensorFusion.getLatestFusedPose().getFloor());
        assertEquals("blocked_live_floor_commit", getObjectField("lastFloorAnchorState"));
        assertFalse(sensorFusion.isDisplayFloorChangeReady(1, 5_000L));
    }

    private CoordinateConverter seedOpenFloors() {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        MapConstraintRepository.replaceVenueConstraints(
                "venue",
                square(converter, -20.0, -20.0, 80.0)
        );
        java.util.List<java.util.List<LatLng>> farWall = Collections.singletonList(
                square(converter, -15.0, -15.0, 1.0)
        );
        MapConstraintRepository.setConstraintsForFloor(1, "1", farWall, null);
        MapConstraintRepository.setConstraintsForFloor(2, "2", farWall, null);
        return converter;
    }

    private CoordinateConverter seedReverseTransitionFloors() {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        MapConstraintRepository.replaceVenueConstraints(
                "venue",
                square(converter, -20.0, -20.0, 80.0)
        );
        MapConstraintRepository.setConstraintsForFloor(
                1,
                "1",
                Collections.singletonList(square(converter, 7.0, 7.0, 3.0)),
                null
        );
        MapConstraintRepository.setConstraintsForFloor(
                2,
                "2",
                Collections.singletonList(square(converter, -15.0, -15.0, 1.0)),
                null
        );
        MapConstraintRepository.setStairsForFloor(
                1,
                Collections.singletonList(square(converter, 0.0, 0.0, 2.0))
        );
        return converter;
    }

    private boolean invokeCommitLiveFloorTransition(
            int absoluteFloor,
            long timestampMs,
            LatLng preferredLandingLatLng,
            String anchorSource
    ) throws Exception {
        Method method = SensorFusion.class.getDeclaredMethod(
                "commitLiveFloorTransition",
                int.class,
                long.class,
                LatLng.class,
                String.class
        );
        method.setAccessible(true);
        return (boolean) method.invoke(
                sensorFusion,
                absoluteFloor,
                timestampMs,
                preferredLandingLatLng,
                anchorSource
        );
    }

    private void invokeRecalibrateAbsoluteFloorBaselineInternal(int absoluteFloor, long timestampMs)
            throws Exception {
        Method method = SensorFusion.class.getDeclaredMethod(
                "recalibrateAbsoluteFloorBaselineInternal",
                int.class,
                long.class
        );
        method.setAccessible(true);
        method.invoke(sensorFusion, absoluteFloor, timestampMs);
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

    private void setField(String fieldName, Object value) throws Exception {
        Field field = SensorFusion.class.getDeclaredField(fieldName);
        field.setAccessible(true);
        field.set(sensorFusion, value);
    }

    private int getIntField(String fieldName) throws Exception {
        Field field = SensorFusion.class.getDeclaredField(fieldName);
        field.setAccessible(true);
        return field.getInt(sensorFusion);
    }

    private Object getObjectField(String fieldName) throws Exception {
        Field field = SensorFusion.class.getDeclaredField(fieldName);
        field.setAccessible(true);
        return field.get(sensorFusion);
    }
}
