package com.openpositioning.PositionMe.sensors;

import android.hardware.Sensor;

import org.junit.Test;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class SensorEventHandlerTest {

    @Test
    public void shouldProcessLinearAcceleration_onlyAcceptsLinearAccelerationSensor() {
        assertTrue(SensorEventHandler.shouldProcessLinearAcceleration(
                Sensor.TYPE_LINEAR_ACCELERATION
        ));
        assertFalse(SensorEventHandler.shouldProcessLinearAcceleration(
                Sensor.TYPE_GYROSCOPE
        ));
    }

    @Test
    public void applyGyroscopeValues_doesNotTouchLinearAccelerationState() {
        SensorState state = new SensorState();
        state.filteredAcc[0] = 4f;
        state.filteredAcc[1] = 5f;
        state.filteredAcc[2] = 6f;

        SensorEventHandler.applyGyroscopeValues(state, new float[]{1f, 2f, 3f});

        assertArrayEquals(new float[]{1f, 2f, 3f}, state.angularVelocity, 0f);
        assertArrayEquals(new float[]{4f, 5f, 6f}, state.filteredAcc, 0f);
    }
}
