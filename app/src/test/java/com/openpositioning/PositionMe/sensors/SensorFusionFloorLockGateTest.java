package com.openpositioning.PositionMe.sensors;

import com.openpositioning.PositionMe.utils.MapConstraintRepository;

import org.junit.After;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.Collections;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

public class SensorFusionFloorLockGateTest {

    private final SensorFusion sensorFusion = SensorFusion.getInstance();

    @After
    public void tearDown() throws Exception {
        MapConstraintRepository.clear();
        sensorFusion.resetAbsoluteAnchorStateForTesting();
        setField("pendingStableAbsoluteFloorCandidate", null);
        setField("pendingStableAbsoluteFloorCount", 0);
        setField("pendingStableAbsoluteFloorFirstTimestampMs", Long.MIN_VALUE);
        setField("pendingStableAbsoluteFloorLastTimestampMs", Long.MIN_VALUE);
        setField("pendingWifiBootstrapFloor", null);
        setField("pendingWifiBootstrapCount", 0);
        setField("pendingWifiBootstrapFirstTimestampMs", Long.MIN_VALUE);
        setField("lastWifiBootstrapCandidateTimestampMs", Long.MIN_VALUE);
        setField("lastFloorCalibrationInvalidatedTimestampMs", Long.MIN_VALUE);
        setField("lastFloorCalibrationInvalidationReason", "none");
        setField("committedDisplayFloorAbsolute", Integer.MIN_VALUE);
        setField("pendingCommittedDisplayFloorAbsolute", Integer.MIN_VALUE);
        setField("liveCurrentFloorAbsolute", Integer.MIN_VALUE);
        setField("floorSwitchPending", false);
        setField("pdrFloorOffset", 0);
        setField("pfInitialized", false);
        setField("isFloorOffsetInitialized", false);
        setField("latestFusedPose", null);
        setField("lastFloorConsensus", "reset");
        setField("lastFloorSource", "relative_only");
        setField("lastFloorAnchorState", "unresolved");
        setField("lastLiftTransferState", "inactive");
        setField("lastPostLiftState", "inactive");
        setField("postLiftExpectedAbsoluteFloor", Integer.MIN_VALUE);
        setField("postLiftDestinationFixWindowUntilMs", Long.MIN_VALUE);
    }

    @Test
    public void startupSingleFloorPriorDoesNotDirectlyLockCurrentFloor() throws Exception {
        seedKnownFloors();
        setField("pfInitialized", true);
        setField("isFloorOffsetInitialized", false);

        assertNull(invokeResolveAcceptedAbsoluteFloorPrior(1, 1_000L));
        assertEquals("initial_lock_pending", getFieldValue("lastFloorConsensus"));
    }

    @Test
    public void startupConsistentWifiBackedFloorEvidenceLocksEarlier() throws Exception {
        seedKnownFloors();
        setField("pfInitialized", true);
        setField("isFloorOffsetInitialized", false);
        setField("pendingWifiBootstrapFloor", 1);

        assertNull(invokeResolveAcceptedAbsoluteFloorPrior(1, 1_000L));
        assertEquals(Integer.valueOf(1), invokeResolveAcceptedAbsoluteFloorPrior(1, 2_000L));
        assertEquals("initial_lock_acquired", getFieldValue("lastFloorConsensus"));
    }

    @Test
    public void startupConsistentAbsoluteFloorEvidenceLocksEarlierWithoutFullMapReadiness() throws Exception {
        seedKnownFloors();
        setField("pfInitialized", true);
        setField("isFloorOffsetInitialized", false);

        assertNull(invokeResolveAcceptedAbsoluteFloorPrior(1, 1_000L));
        assertEquals(Integer.valueOf(1), invokeResolveAcceptedAbsoluteFloorPrior(1, 2_000L));
        assertEquals("initial_lock_acquired", getFieldValue("lastFloorConsensus"));
    }

    @Test
    public void recentInvalidationAllowsAcceptedWifiBootstrapToLockImmediately() throws Exception {
        seedKnownFloors();
        setField("pfInitialized", true);
        setField("isFloorOffsetInitialized", false);
        setField("pendingWifiBootstrapFloor", 1);
        setField("pendingWifiBootstrapCount", SensorFusion.resolveWifiBootstrapRequiredConfirmations(1));
        setField("pendingWifiBootstrapFirstTimestampMs", 1_000L);
        setField("lastWifiBootstrapCandidateTimestampMs", 2_000L);
        setField("lastFloorCalibrationInvalidatedTimestampMs", 2_000L);
        setField("lastFloorCalibrationInvalidationReason", "venue_switch");

        assertEquals(Integer.valueOf(1), invokeResolveAcceptedAbsoluteFloorPrior(1, 2_500L));
        assertEquals("initial_lock_acquired", getFieldValue("lastFloorConsensus"));
    }

    @Test
    public void wrongLockedFloorCanRecoverAfterSustainedConsistentEvidenceEvenWhenCurrentFloorIsHighConfidence() throws Exception {
        seedKnownFloors();
        setField("pfInitialized", true);
        setField("isFloorOffsetInitialized", true);
        setField("pdrFloorOffset", 0);
        setField("latestFusedPose", new FusedPose(0.0, 0.0, 0, 0.90, 500L));

        long now = 10_000L;
        assertNull(invokeResolveAcceptedAbsoluteFloorPrior(1, now));
        assertNull(invokeResolveAcceptedAbsoluteFloorPrior(1, now + 3_000L));
        assertNull(invokeResolveAcceptedAbsoluteFloorPrior(1, now + 6_000L));
        assertEquals(Integer.valueOf(1), invokeResolveAcceptedAbsoluteFloorPrior(1, now + 9_000L));
        assertEquals("recovered_from_wrong_lock", getFieldValue("lastFloorConsensus"));
    }

    private void seedKnownFloors() {
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(0, "0", Collections.emptyList(), null);
        MapConstraintRepository.setConstraintsForFloor(1, "1", Collections.emptyList(), null);
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
}
