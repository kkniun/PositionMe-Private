package com.openpositioning.PositionMe.utils;

import androidx.annotation.NonNull;

import com.google.android.gms.maps.model.LatLng;

/**
 * Converts WGS84 coordinates to a local metric frame centered on a chosen origin.
 */
public final class CoordinateConverter {

    private static final double DEGREE_IN_METERS = 111111.0;

    private final double originLatitudeDeg;
    private final double originLongitudeDeg;

    public CoordinateConverter(double originLatitudeDeg, double originLongitudeDeg) {
        this.originLatitudeDeg = originLatitudeDeg;
        this.originLongitudeDeg = originLongitudeDeg;
    }

    public double getOriginLatitudeDeg() {
        return originLatitudeDeg;
    }

    public double getOriginLongitudeDeg() {
        return originLongitudeDeg;
    }

    @NonNull
    public double[] toLocalMeters(double latitudeDeg, double longitudeDeg) {
        double xMeters = UtilFunctions.degreesToMetersLng(longitudeDeg - originLongitudeDeg, originLatitudeDeg);
        double yMeters = UtilFunctions.degreesToMetersLat(latitudeDeg - originLatitudeDeg);
        return new double[]{xMeters, yMeters};
    }

    @NonNull
    public LatLng toLatLng(double xMeters, double yMeters) {
        double latitudeDeg = originLatitudeDeg + (yMeters / DEGREE_IN_METERS);
        double longitudeDeg = originLongitudeDeg
                + (xMeters * Math.cos(Math.toRadians(originLatitudeDeg)) / DEGREE_IN_METERS);
        return new LatLng(latitudeDeg, longitudeDeg);
    }
}
