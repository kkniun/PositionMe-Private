package com.openpositioning.PositionMe.utils;

import androidx.annotation.NonNull;

import com.google.android.gms.maps.model.LatLng;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

final class ConstraintGeometryUtils {

    private ConstraintGeometryUtils() {
    }

    @NonNull
    static List<LatLng> buildBufferedSegmentPolygon(
            @NonNull LatLng start,
            @NonNull LatLng end,
            double halfWidthMeters
    ) {
        if (halfWidthMeters <= 0.0) {
            return Collections.emptyList();
        }

        double averageLatitudeRad = Math.toRadians((start.latitude + end.latitude) * 0.5);
        double metersPerDegreeLat = 111_320.0;
        double metersPerDegreeLon = Math.max(1e-6, Math.cos(averageLatitudeRad) * 111_320.0);
        double eastMeters = (end.longitude - start.longitude) * metersPerDegreeLon;
        double northMeters = (end.latitude - start.latitude) * metersPerDegreeLat;
        double lengthMeters = Math.hypot(eastMeters, northMeters);
        if (lengthMeters < 1e-6) {
            return Collections.emptyList();
        }

        double offsetEastMeters = -northMeters / lengthMeters * halfWidthMeters;
        double offsetNorthMeters = eastMeters / lengthMeters * halfWidthMeters;
        double offsetLat = offsetNorthMeters / metersPerDegreeLat;
        double offsetLon = offsetEastMeters / metersPerDegreeLon;

        List<LatLng> polygon = new ArrayList<>(4);
        polygon.add(new LatLng(start.latitude + offsetLat, start.longitude + offsetLon));
        polygon.add(new LatLng(end.latitude + offsetLat, end.longitude + offsetLon));
        polygon.add(new LatLng(end.latitude - offsetLat, end.longitude - offsetLon));
        polygon.add(new LatLng(start.latitude - offsetLat, start.longitude - offsetLon));
        return polygon;
    }
}
