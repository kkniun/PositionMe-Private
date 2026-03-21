package com.openpositioning.PositionMe.utils;

import androidx.annotation.NonNull;

import com.google.android.gms.maps.model.LatLng;

/**
 * Converts WGS84 coordinates to and from a local easting/northing frame
 * centered on a chosen WGS84 origin.
 */
public final class CoordinateConverter {

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
        double xMeters = UtilFunctions.degreesToMetersLng(
                longitudeDeg - originLongitudeDeg,
                originLatitudeDeg
        );
        double yMeters = UtilFunctions.degreesToMetersLat(latitudeDeg - originLatitudeDeg);
        return new double[]{xMeters, yMeters};
    }

    @NonNull
    public LatLng toLatLng(double xMeters, double yMeters) {
        double latitudeDeg = originLatitudeDeg + UtilFunctions.metersToDegreesLat(yMeters);
        double longitudeDeg = originLongitudeDeg
                + UtilFunctions.metersToDegreesLng(xMeters, originLatitudeDeg);
        return new LatLng(latitudeDeg, longitudeDeg);
    }
}
