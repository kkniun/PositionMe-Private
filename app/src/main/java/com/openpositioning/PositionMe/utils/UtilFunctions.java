package com.openpositioning.PositionMe.utils;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.drawable.Drawable;

import androidx.core.content.ContextCompat;

import com.google.android.gms.maps.model.LatLng;
import com.openpositioning.PositionMe.presentation.fragment.RecordingFragment;

/**
 * Utility functions shared across the app.
 *
 * <p>The coordinate helpers use a small-area WGS84 approximation:
 * one degree of latitude is treated as a constant metric distance and
 * longitude is scaled by cos(latitude). This is appropriate for the
 * short indoor/outdoor distances handled by the coursework.</p>
 *
 * @see RecordingFragment Currently used by RecordingFragment
 */
public class UtilFunctions {
    private static final double METERS_PER_DEGREE_LAT = 111_111.0;

    /**
     * Simple function to calculate the angle between two close points
     * @param pointA Starting point
     * @param pointB Ending point
     * @return Angle between the points
     */
    public static double calculateAngleSimple(LatLng pointA, LatLng pointB) {
        // Simple formula for close-by points
        return Math.toDegrees( Math.atan2(pointB.latitude-pointA.latitude,
                (pointB.longitude- pointA.longitude)*Math.cos(Math.toRadians(pointA.latitude))));
    }

    /**
     * Calculates a new WGS84 position from local east/north movement in meters.
     *
     * @param initialLocation current WGS84 location
     * @param pdrMoved movement in the local frame, where x is east and y is north
     * @return updated WGS84 location
     */
    public static LatLng calculateNewPos(LatLng initialLocation, float[] pdrMoved) {
        double newLatitude = initialLocation.latitude + metersToDegreesLat(pdrMoved[1]);
        double newLongitude = initialLocation.longitude
                + metersToDegreesLng(pdrMoved[0], initialLocation.latitude);
        return new LatLng(newLatitude, newLongitude);
    }

    /**
     * Converts a latitude delta in degrees into northing in meters.
     *
     * @param degreeVal latitude delta in degrees
     * @return northing in meters
     */
    public static double degreesToMetersLat(double degreeVal) {
        return degreeVal * METERS_PER_DEGREE_LAT;
    }

    /**
     * Converts a longitude delta in degrees into easting in meters.
     *
     * <p>Longitude degrees shrink with latitude, so the metric scale must be
     * multiplied by cos(latitude).</p>
     *
     * @param degreeVal longitude delta in degrees
     * @param latitude reference latitude in degrees
     * @return easting in meters
     */
    public static double degreesToMetersLng(double degreeVal, double latitude) {
        return degreeVal * metersPerDegreeLongitude(latitude);
    }

    /**
     * Converts northing in meters into a latitude delta in degrees.
     *
     * @param meters northing in meters
     * @return latitude delta in degrees
     */
    public static double metersToDegreesLat(double meters) {
        return meters / METERS_PER_DEGREE_LAT;
    }

    /**
     * Converts easting in meters into a longitude delta in degrees.
     *
     * @param meters easting in meters
     * @param latitude reference latitude in degrees
     * @return longitude delta in degrees
     */
    public static double metersToDegreesLng(double meters, double latitude) {
        return meters / metersPerDegreeLongitude(latitude);
    }

    /**
     * Calculates the approximate distance between two nearby WGS84 points.
     *
     * <p>This uses the same small-area approximation as the local coordinate pipeline.</p>
     *
     * @param pointA initial point
     * @param pointB final point
     * @return distance between the two points in meters
     */
    public static double distanceBetweenPoints(LatLng pointA, LatLng pointB) {
        double referenceLatitude = (pointA.latitude + pointB.latitude) * 0.5;
        double northMeters = degreesToMetersLat(pointA.latitude - pointB.latitude);
        double eastMeters = degreesToMetersLng(pointA.longitude - pointB.longitude, referenceLatitude);
        return Math.sqrt(Math.pow(northMeters, 2) + Math.pow(eastMeters, 2));
    }

    /**
     * Creates a bitmap from a vector
     * @param context Context of activity being used
     * @param vectorResourceID Resource id whose vector get converted to a Bitmap
     * @return Bitmap of the resource vector
     */
    public static Bitmap getBitmapFromVector(Context context, int vectorResourceID) {
        // Get drawable vector
        Drawable vectorDrawable = ContextCompat.getDrawable(context, vectorResourceID);
        // Bitmap created to draw the vector in
        Bitmap bitmap = Bitmap.createBitmap(vectorDrawable.getIntrinsicWidth(), vectorDrawable.getIntrinsicHeight(), Bitmap.Config.ARGB_8888);
        // Canvas to draw the bitmap on
        Canvas canvas = new Canvas(bitmap);
        // Drawing on canvas
        vectorDrawable.setBounds(0, 0, canvas.getWidth(), canvas.getHeight());
        vectorDrawable.draw(canvas);
        return bitmap;
    }

    private static double metersPerDegreeLongitude(double latitudeDeg) {
        return METERS_PER_DEGREE_LAT * Math.cos(Math.toRadians(latitudeDeg));
    }

}
