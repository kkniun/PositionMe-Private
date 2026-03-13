package com.openpositioning.PositionMe.sensors;

import com.openpositioning.PositionMe.Traj;

import org.junit.Test;

import static org.junit.Assert.*;

/**
 * Minimal tests for initial_position wiring in SensorFusion.startRecording helper.
 */
public class SensorFusionInitialPositionTest {

    @Test
    public void maybeSetInitialPosition_setsWhenValid() {
        Traj.Trajectory.Builder builder = Traj.Trajectory.newBuilder();
        float[] start = new float[]{55.0f, -3.2f};

        SensorFusion.maybeSetInitialPosition(builder, start);

        assertTrue(builder.hasInitialPosition());
        assertEquals(55.0, builder.getInitialPosition().getLatitude(), 1e-6);
        assertEquals(-3.2, builder.getInitialPosition().getLongitude(), 1e-6);
    }

    @Test
    public void maybeSetInitialPosition_skipsWhenNullOrShort() {
        Traj.Trajectory.Builder builder = Traj.Trajectory.newBuilder();

        SensorFusion.maybeSetInitialPosition(builder, null);
        assertFalse(builder.hasInitialPosition());

        SensorFusion.maybeSetInitialPosition(builder, new float[]{1.0f});
        assertFalse(builder.hasInitialPosition());
    }

    @Test
    public void maybeSetInitialPositionIfAbsent_setsFromFirstAbsoluteFix() {
        Traj.Trajectory.Builder builder = Traj.Trajectory.newBuilder();

        SensorFusion.maybeSetInitialPositionIfAbsent(builder, 55.9444, -3.1883);

        assertTrue(builder.hasInitialPosition());
        assertEquals(55.9444, builder.getInitialPosition().getLatitude(), 1e-6);
        assertEquals(-3.1883, builder.getInitialPosition().getLongitude(), 1e-6);
    }

    @Test
    public void maybeSetInitialPositionIfAbsent_preservesExistingLegacyInitialPosition() {
        Traj.Trajectory.Builder builder = Traj.Trajectory.newBuilder();
        SensorFusion.maybeSetInitialPosition(builder, new float[]{55.0f, -3.2f});

        SensorFusion.maybeSetInitialPositionIfAbsent(builder, 55.9444, -3.1883);

        assertTrue(builder.hasInitialPosition());
        assertEquals(55.0, builder.getInitialPosition().getLatitude(), 1e-6);
        assertEquals(-3.2, builder.getInitialPosition().getLongitude(), 1e-6);
    }
}
