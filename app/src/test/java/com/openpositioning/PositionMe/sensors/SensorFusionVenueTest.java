package com.openpositioning.PositionMe.sensors;

import com.openpositioning.PositionMe.Traj;

import org.junit.After;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Method;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

/**
 * Minimal unit test to ensure venue_id is applied into trajectory protobuf when set.
 */
public class SensorFusionVenueTest {

    private final SensorFusion sensorFusion = SensorFusion.getInstance();

    @After
    public void tearDown() throws Exception {
        sensorFusion.resetAbsoluteAnchorStateForTesting();
        setField("collectionVenue", "default");
        setField("liveCurrentFloorAbsolute", Integer.MIN_VALUE);
        setField("committedDisplayFloorAbsolute", Integer.MIN_VALUE);
        setField("pendingCommittedDisplayFloorAbsolute", Integer.MIN_VALUE);
        setField("floorSwitchPending", false);
        setField("pdrFloorOffset", 0);
        setField("isFloorOffsetInitialized", false);
        setField("blockHistoricalPoseFloorSeedUntilTrustedFix", false);
        setField("pendingStableAbsoluteFloorCandidate", null);
        setField("pendingStableAbsoluteFloorCount", 0);
        setField("pendingStableAbsoluteFloorFirstTimestampMs", Long.MIN_VALUE);
        setField("pendingStableAbsoluteFloorLastTimestampMs", Long.MIN_VALUE);
        setField("pendingWifiBootstrapFloor", null);
        setField("pendingWifiBootstrapCount", 0);
        setField("pendingWifiBootstrapFirstTimestampMs", Long.MIN_VALUE);
        setField("lastWifiBootstrapCandidateTimestampMs", Long.MIN_VALUE);
        setField("floorOnlyResyncAbsoluteFloor", Integer.MIN_VALUE);
        setField("floorOnlyResyncWindowUntilMs", Long.MIN_VALUE);
        setField("pendingDisplayFloorResetAbsoluteFloor", Integer.MIN_VALUE);
        setField("pendingDisplayFloorResetUntilMs", Long.MIN_VALUE);
        setField("latestFusedPose", null);
        setField("lastFloorConsensus", "reset");
        setField("lastFloorSource", "relative_only");
        setField("lastFloorAnchorState", "unresolved");
        setField("postLiftExpectedAbsoluteFloor", Integer.MIN_VALUE);
        setRelativeFloor(0);
    }

    @Test
    public void applyVenueId_setsFieldOnBuilder() {
        Traj.Trajectory.Builder builder = Traj.Trajectory.newBuilder();
        builder.setTrajectoryId("20240202_aaaa");

        SensorFusion.applyVenuePrefixToTrajectoryId(builder, "abc");

        assertEquals("abc_20240202_aaaa", builder.getTrajectoryId());
    }

    @Test
    public void switchingVenueInvalidatesTrustedFloorAndBlocksOldFloorSeedReuse() throws Exception {
        setField("collectionVenue", "old_venue");
        setField("pdrFloorOffset", 7);
        setField("isFloorOffsetInitialized", true);
        setField("pendingStableAbsoluteFloorCandidate", 0);
        setField("pendingStableAbsoluteFloorCount", 2);
        setField("pendingWifiBootstrapFloor", 0);
        setField("pendingWifiBootstrapCount", 2);
        setField("floorOnlyResyncAbsoluteFloor", 0);
        setField("pendingDisplayFloorResetAbsoluteFloor", 0);
        setField("latestFusedPose", new FusedPose(1.0, 2.0, 0, 0.95, 1_000L));
        setRelativeFloor(-7);

        assertTrue(sensorFusion.isFloorCalibrated());
        assertEquals(0, sensorFusion.getCurrentFloor());
        assertEquals(Integer.valueOf(0), invokeResolveStep1InitializationFloor(null));

        sensorFusion.setCollectionVenue("new_venue");

        assertFalse(sensorFusion.isFloorCalibrated());
        assertEquals(0, getField("pdrFloorOffset"));
        assertTrue((Boolean) getField("blockHistoricalPoseFloorSeedUntilTrustedFix"));
        assertNull(invokeResolveStep1InitializationFloor(null));
        assertNull(getField("pendingStableAbsoluteFloorCandidate"));
        assertNull(getField("pendingWifiBootstrapFloor"));
        assertEquals(Integer.MIN_VALUE, getField("floorOnlyResyncAbsoluteFloor"));
        assertEquals(Integer.MIN_VALUE, getField("pendingDisplayFloorResetAbsoluteFloor"));
    }

    @Test
    public void redundantVenueUpdateDoesNotInvalidateExistingTrustedFloor() throws Exception {
        setField("collectionVenue", "same_venue");
        setField("pdrFloorOffset", 3);
        setField("isFloorOffsetInitialized", true);
        setRelativeFloor(-2);

        sensorFusion.setCollectionVenue("same_venue");

        assertTrue(sensorFusion.isFloorCalibrated());
        assertEquals(3, getField("pdrFloorOffset"));
        assertEquals(Integer.valueOf(1), invokeResolveStep1InitializationFloor(null));
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

    private void setRelativeFloor(int relativeFloor) throws Exception {
        Field trackerField = SensorFusion.class.getDeclaredField("cumulativeFloorTracker");
        trackerField.setAccessible(true);
        Object tracker = trackerField.get(sensorFusion);
        Field relativeFloorField = tracker.getClass().getDeclaredField("relativeFloor");
        relativeFloorField.setAccessible(true);
        relativeFloorField.setInt(tracker, relativeFloor);
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
}
