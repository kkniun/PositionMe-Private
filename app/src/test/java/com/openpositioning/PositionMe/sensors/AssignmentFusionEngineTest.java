package com.openpositioning.PositionMe.sensors;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class AssignmentFusionEngineTest {

    @Test
    public void initFromAbsoluteSetsPose() {
        AssignmentFusionEngine engine = new AssignmentFusionEngine();
        FusedPose pose = engine.initFromAbsolute(10.0, -3.5, 4, 1000L, 0.1);
        assertNotNull(pose);
        assertEquals(10.0, pose.getX(), 1e-9);
        assertEquals(-3.5, pose.getY(), 1e-9);
        assertEquals(4, pose.getFloor());
    }

    @Test
    public void predictWithOpenWalkabilityMovesAlongNorth() {
        AssignmentFusionEngine engine = new AssignmentFusionEngine();
        engine.initFromAbsolute(0.0, 0.0, 0, 1000L, 0.0);
        AssignmentFusionEngine.Walkability open = (x, y, f) -> true;
        FusedPose pose = engine.predictFromPdr(
                new PdrDelta(1.0f, 0.0f, 0.0f),
                0,
                1100L,
                null,
                open
        );
        assertEquals(0.0, pose.getX(), 1e-6);
        assertEquals(1.0, pose.getY(), 1e-6);
    }

    @Test
    public void applyAbsoluteObservationPullsTowardFix() {
        AssignmentFusionEngine engine = new AssignmentFusionEngine();
        engine.initFromAbsolute(0.0, 0.0, 0, 1000L, 0.0);
        engine.predictFromPdr(new PdrDelta(2.0f, 0.0f, 0.0f), 0, 1100L, null, (x, y, f) -> true);
        FusedPose pulled = engine.applyAbsoluteObservation(0.0, 0.0, 0, 4.0f, 1200L);
        assertTrue(Math.abs(pulled.getY()) < 1.5);
    }
}
