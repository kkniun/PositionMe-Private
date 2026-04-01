package com.openpositioning.PositionMe.sensors;

import com.google.android.gms.maps.model.LatLng;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class WifiObservationSmootherTest {

    @Test
    public void observe_singleReading_returnsRawObservation() {
        WifiObservationSmoother smoother = new WifiObservationSmoother();

        WifiObservationSmoother.SmoothedObservation observation = smoother.observe(
                new LatLng(55.9440, -3.1870),
                1,
                1_000L
        );

        assertEquals(55.9440, observation.getLocation().latitude, 1e-9);
        assertEquals(-3.1870, observation.getLocation().longitude, 1e-9);
        assertEquals(1, observation.getFloor());
        assertEquals(1, observation.getSupportCount());
    }

    @Test
    public void observe_twoNearbySameFloorReadings_returnsAveragedObservation() {
        WifiObservationSmoother smoother = new WifiObservationSmoother();
        smoother.observe(new LatLng(55.944000, -3.187000), 1, 1_000L);

        WifiObservationSmoother.SmoothedObservation observation = smoother.observe(
                new LatLng(55.944018, -3.186982),
                1,
                1_800L
        );

        assertEquals(55.944009, observation.getLocation().latitude, 1e-9);
        assertEquals(-3.186991, observation.getLocation().longitude, 1e-9);
        assertEquals(1, observation.getFloor());
        assertEquals(2, observation.getSupportCount());
    }

    @Test
    public void observe_largeCrossFloorJump_resetsCluster() {
        WifiObservationSmoother smoother = new WifiObservationSmoother();
        smoother.observe(new LatLng(55.944000, -3.187000), 1, 1_000L);
        smoother.observe(new LatLng(55.944018, -3.186982), 1, 1_800L);

        WifiObservationSmoother.SmoothedObservation observation = smoother.observe(
                new LatLng(55.944250, -3.186600),
                2,
                2_600L
        );

        assertEquals(55.944250, observation.getLocation().latitude, 1e-9);
        assertEquals(-3.186600, observation.getLocation().longitude, 1e-9);
        assertEquals(2, observation.getFloor());
        assertEquals(1, observation.getSupportCount());
    }
}
