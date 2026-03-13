package com.openpositioning.PositionMe.sensors;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class SensorFusionTrajectoryNameTest {

    @Test
    public void buildTrajectoryName_preservesExplicitUserInput() {
        assertEquals("Assignment 1 Demo", SensorFusion.buildTrajectoryName("Assignment 1 Demo", 0L));
    }

    @Test
    public void buildTrajectoryName_generatesDefaultWhenBlank() {
        String generated = SensorFusion.buildTrajectoryName("   ", 0L);

        assertTrue(generated.startsWith("Trajectory "));
    }
}
