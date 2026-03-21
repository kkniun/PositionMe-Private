package com.openpositioning.PositionMe.utils;

import com.google.android.gms.maps.model.LatLng;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class CoordinateConverterTest {

    private static final double EDINBURGH_LATITUDE = 55.9229;
    private static final double EDINBURGH_LONGITUDE = -3.1721;

    @Test
    public void roundTripPreservesCoordinates() {
        CoordinateConverter converter = new CoordinateConverter(EDINBURGH_LATITUDE, EDINBURGH_LONGITUDE);

        double[] local = converter.toLocalMeters(55.9232, -3.1714);
        LatLng roundTrip = converter.toLatLng(local[0], local[1]);

        assertEquals(55.9232, roundTrip.latitude, 1e-6);
        assertEquals(-3.1714, roundTrip.longitude, 1e-6);
    }

    @Test
    public void latitudeAndLongitudeMetricScalesAreReasonableNearEdinburgh() {
        double latitudeMeters = UtilFunctions.degreesToMetersLat(0.001);
        double longitudeMeters = UtilFunctions.degreesToMetersLng(0.001, EDINBURGH_LATITUDE);

        assertEquals(111.111, latitudeMeters, 0.001);
        assertEquals(62.256, longitudeMeters, 0.5);
        assertTrue("Longitude scale should be smaller than latitude scale near Edinburgh",
                longitudeMeters < latitudeMeters);
        assertTrue("Longitude scale must not be inflated to ~200 m",
                longitudeMeters < 100.0);
    }

    @Test
    public void toLocalMetersKeepsEastAndNorthAxesSeparate() {
        CoordinateConverter converter = new CoordinateConverter(EDINBURGH_LATITUDE, EDINBURGH_LONGITUDE);

        double[] eastOnly = converter.toLocalMeters(EDINBURGH_LATITUDE, EDINBURGH_LONGITUDE + 0.001);
        double[] northOnly = converter.toLocalMeters(EDINBURGH_LATITUDE + 0.001, EDINBURGH_LONGITUDE);

        assertEquals(62.256, eastOnly[0], 0.5);
        assertEquals(0.0, eastOnly[1], 0.001);
        assertEquals(0.0, northOnly[0], 0.001);
        assertEquals(111.111, northOnly[1], 0.001);
    }

    @Test
    public void calculateNewPosUsesTheSameConversionAsCoordinateConverter() {
        LatLng origin = new LatLng(EDINBURGH_LATITUDE, EDINBURGH_LONGITUDE);
        CoordinateConverter converter = new CoordinateConverter(origin.latitude, origin.longitude);

        float[] localMoveMeters = new float[]{62.256f, 111.111f};
        LatLng moved = UtilFunctions.calculateNewPos(origin, localMoveMeters);
        double[] local = converter.toLocalMeters(moved.latitude, moved.longitude);

        assertEquals(localMoveMeters[0], local[0], 0.5);
        assertEquals(localMoveMeters[1], local[1], 0.001);
    }

    @Test
    public void localEastNorthDirectionsMatchWgs84Directions() {
        CoordinateConverter converter = new CoordinateConverter(EDINBURGH_LATITUDE, EDINBURGH_LONGITUDE);

        LatLng moved = converter.toLatLng(25.0, 40.0);

        assertTrue("Moving east in local meters should increase longitude",
                moved.longitude > EDINBURGH_LONGITUDE);
        assertTrue("Moving north in local meters should increase latitude",
                moved.latitude > EDINBURGH_LATITUDE);

        double[] local = converter.toLocalMeters(moved.latitude, moved.longitude);

        assertTrue("Round-trip local easting should stay positive in the same direction",
                local[0] > 0.0);
        assertTrue("Round-trip local northing should stay positive in the same direction",
                local[1] > 0.0);
        assertEquals(25.0, local[0], 0.001);
        assertEquals(40.0, local[1], 0.001);
    }
}
