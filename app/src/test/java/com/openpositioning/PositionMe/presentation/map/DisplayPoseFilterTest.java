package com.openpositioning.PositionMe.presentation.map;

import com.google.android.gms.maps.model.LatLng;
import com.openpositioning.PositionMe.utils.CoordinateConverter;
import com.openpositioning.PositionMe.utils.UtilFunctions;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class DisplayPoseFilterTest {

    private static final double ORIGIN_LAT = 55.9444;
    private static final double ORIGIN_LON = -3.1878;

    @Test
    public void stationaryJitterIsHeldInsideDeadband() {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        DisplayPoseFilter filter = new DisplayPoseFilter(DisplayPoseFilter.defaultMarkerConfig());

        LatLng anchor = converter.toLatLng(0.0, 0.0);
        LatLng tinyJitter = converter.toLatLng(0.15, 0.10);

        filter.updatePosition(anchor, false, false);
        LatLng filtered = filter.updatePosition(tinyJitter, true, false);

        assertTrue(UtilFunctions.distanceBetweenPoints(anchor, filtered) < 1e-3);
    }

    @Test
    public void sustainedMovementStillTracksWithoutExcessiveLag() {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        DisplayPoseFilter filter = new DisplayPoseFilter(DisplayPoseFilter.defaultMarkerConfig());

        LatLng anchor = converter.toLatLng(0.0, 0.0);
        filter.updatePosition(anchor, false, false);

        LatLng filtered = anchor;
        LatLng latestRaw = anchor;
        for (int step = 1; step <= 4; step++) {
            latestRaw = converter.toLatLng(0.0, step * 1.5);
            filtered = filter.updatePosition(latestRaw, false, false);
        }

        assertTrue(UtilFunctions.distanceBetweenPoints(anchor, filtered) > 4.0);
        assertTrue(UtilFunctions.distanceBetweenPoints(filtered, latestRaw) < 1.2);
    }

    @Test
    public void movingHeadingPrefersRecentMotionHeading() {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        DisplayPoseFilter filter = new DisplayPoseFilter(DisplayPoseFilter.defaultMarkerConfig());

        LatLng anchor = converter.toLatLng(0.0, 0.0);
        LatLng eastward = converter.toLatLng(3.2, 0.0);

        filter.updatePosition(anchor, false, false);
        filter.updatePosition(eastward, false, false);
        float renderedHeading = filter.updateHeading(0f, true, false);
        DisplayPoseFilter.HeadingDebugSnapshot debug = filter.getHeadingDebugSnapshot();

        assertEquals("motion", debug.headingSource);
        assertTrue(Math.abs(shortestAngularDifference(renderedHeading, 90f)) < 1e-3f);
    }

    @Test
    public void stationaryHeadingHoldsLastStableHeading() {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        DisplayPoseFilter filter = new DisplayPoseFilter(DisplayPoseFilter.defaultMarkerConfig());

        LatLng anchor = converter.toLatLng(0.0, 0.0);
        LatLng tinyJitter = converter.toLatLng(0.08, 0.05);

        filter.updatePosition(anchor, true, false);
        filter.updatePosition(tinyJitter, true, false);
        filter.updateHeading(18f, true, true);
        float renderedHeading = filter.updateHeading(142f, false, false, "stationary");
        DisplayPoseFilter.HeadingDebugSnapshot debug = filter.getHeadingDebugSnapshot();

        assertTrue("hold_last_stable".equals(debug.headingSource));
        assertTrue(Math.abs(shortestAngularDifference(renderedHeading, 18f)) < 1e-3f);
    }

    @Test
    public void motionHeadingOverridesLowConfidenceFreezeWhenWalking() {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        DisplayPoseFilter filter = new DisplayPoseFilter(DisplayPoseFilter.defaultMarkerConfig());

        LatLng anchor = converter.toLatLng(0.0, 0.0);
        LatLng eastward = converter.toLatLng(1.0, 0.0);

        filter.updatePosition(anchor, false, false);
        filter.updatePosition(eastward, false, false);
        filter.updateHeading(10f, true, true);
        float renderedHeading = filter.updateHeading(180f, true, false, "low_confidence");
        DisplayPoseFilter.HeadingDebugSnapshot debug = filter.getHeadingDebugSnapshot();

        assertEquals("motion", debug.headingSource);
        assertTrue(Math.abs(shortestAngularDifference(renderedHeading, 90f)) < 1e-3f);
    }

    @Test
    public void headingOnlyTurnDoesNotFreezeWhenNotStationary() {
        DisplayPoseFilter filter = new DisplayPoseFilter(DisplayPoseFilter.defaultMarkerConfig());

        filter.updateHeading(15f, true, true);
        float renderedHeading = filter.updateHeading(105f, false, false, "awaiting_motion");
        DisplayPoseFilter.HeadingDebugSnapshot debug = filter.getHeadingDebugSnapshot();

        assertEquals("device", debug.headingSource);
        assertTrue(Math.abs(shortestAngularDifference(renderedHeading, 105f)) < 1e-3f);
    }

    @Test
    public void shortDisplacementTurnDoesNotStayLockedOnOldHeading() {
        DisplayPoseFilter filter = new DisplayPoseFilter(DisplayPoseFilter.defaultMarkerConfig());

        filter.updateHeading(0f, true, true);
        float renderedHeading = filter.updateHeading(48f, false, false, "insufficient_movement");
        DisplayPoseFilter.HeadingDebugSnapshot debug = filter.getHeadingDebugSnapshot();

        assertEquals("device", debug.headingSource);
        assertTrue(renderedHeading > 0f);
        assertTrue(Math.abs(shortestAngularDifference(renderedHeading, 48f)) <= 16f);
    }

    @Test
    public void moderateMovingTurnRespondsWithoutVisibleTail() {
        DisplayPoseFilter filter = new DisplayPoseFilter(DisplayPoseFilter.defaultMarkerConfig());

        filter.updateHeading(0f, true, true);
        float renderedHeading = filter.updateHeading(18f, true, false);

        assertTrue(Math.abs(shortestAngularDifference(renderedHeading, 18f)) <= 4.0f);
    }

    @Test
    public void largeMovingTurnSnapsImmediately() {
        DisplayPoseFilter filter = new DisplayPoseFilter(DisplayPoseFilter.defaultMarkerConfig());

        filter.updateHeading(0f, true, true);
        float renderedHeading = filter.updateHeading(90f, true, false);

        assertTrue(Math.abs(shortestAngularDifference(renderedHeading, 90f)) < 1e-3f);
    }

    @Test
    public void headingWrapAroundKeepsShortestTurnDirection() {
        DisplayPoseFilter filter = new DisplayPoseFilter(DisplayPoseFilter.defaultMarkerConfig());

        filter.updateHeading(350f, true, true);
        float renderedHeading = filter.updateHeading(5f, true, false, "awaiting_motion");

        assertTrue(Math.abs(shortestAngularDifference(renderedHeading, 5f)) <= 4.0f);
    }

    @Test
    public void noRecentPoseDoesNotFreezeFreshDeviceHeading() {
        DisplayPoseFilter filter = new DisplayPoseFilter(DisplayPoseFilter.defaultMarkerConfig());

        filter.updateHeading(12f, true, true);
        float renderedHeading = filter.updateHeading(84f, true, false, "no_recent_pose");
        DisplayPoseFilter.HeadingDebugSnapshot debug = filter.getHeadingDebugSnapshot();

        assertEquals("device", debug.headingSource);
        assertTrue(Math.abs(shortestAngularDifference(renderedHeading, 84f)) <= 4.0f);
    }

    private float shortestAngularDifference(float fromDeg, float toDeg) {
        return (fromDeg - toDeg + 180f + 360f) % 360f - 180f;
    }
}
