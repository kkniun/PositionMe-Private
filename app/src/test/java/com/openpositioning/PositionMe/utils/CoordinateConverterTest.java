package com.openpositioning.PositionMe.utils;

import com.google.android.gms.maps.model.LatLng;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class CoordinateConverterTest {

    @Test
    public void roundTripPreservesCoordinates() {
        CoordinateConverter converter = new CoordinateConverter(55.9229, -3.1721);

        double[] local = converter.toLocalMeters(55.9232, -3.1714);
        LatLng roundTrip = converter.toLatLng(local[0], local[1]);

        assertEquals(55.9232, roundTrip.latitude, 1e-6);
        assertEquals(-3.1714, roundTrip.longitude, 1e-6);
    }
}
