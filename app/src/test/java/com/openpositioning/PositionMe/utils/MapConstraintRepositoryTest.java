package com.openpositioning.PositionMe.utils;

import com.google.android.gms.maps.model.LatLng;

import org.junit.After;
import org.junit.Test;

import java.util.Collections;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class MapConstraintRepositoryTest {

    @After
    public void tearDown() {
        MapConstraintRepository.clear();
    }

    @Test
    public void doesPathIntersectWallReturnsTrueWhenSegmentCrossesPolygon() {
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(
                0,
                Collections.singletonList(square(0.0, 0.0, 0.0001)),
                null
        );

        assertTrue(MapConstraintRepository.doesPathIntersectWall(
                new LatLng(0.00005, -0.00005),
                new LatLng(0.00005, 0.00015),
                0
        ));
    }

    @Test
    public void doesPathIntersectWallReturnsFalseWhenSegmentStaysOutsidePolygon() {
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(
                0,
                Collections.singletonList(square(0.0, 0.0, 0.0001)),
                null
        );

        assertFalse(MapConstraintRepository.doesPathIntersectWall(
                new LatLng(-0.00005, -0.00005),
                new LatLng(-0.00005, 0.00015),
                0
        ));
    }

    @Test
    public void bufferedLinePolygonProducesRuntimeWallConstraint() {
        List<LatLng> bufferedWall = ConstraintGeometryUtils.buildBufferedSegmentPolygon(
                new LatLng(55.9444, -3.1878),
                new LatLng(55.9444, -3.1876),
                0.5
        );

        assertEquals(4, bufferedWall.size());

        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(
                1,
                Collections.singletonList(bufferedWall),
                null
        );

        assertTrue(MapConstraintRepository.doesPathIntersectWall(
                new LatLng(55.94435, -3.1877),
                new LatLng(55.94445, -3.1877),
                1
        ));
    }

    @Test
    public void doesPathIntersectStairsReturnsTrueWhenCrossingTransitionPolygon() {
        List<List<LatLng>> stairs = Collections.singletonList(square(0.0, 0.0, 0.0001));
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setStairsForFloor(2, stairs);

        assertTrue(MapConstraintRepository.doesPathIntersectStairs(
                new LatLng(0.00005, -0.00005),
                new LatLng(0.00005, 0.00015),
                2
        ));
        assertTrue(MapConstraintRepository.hasTransitionConstraints(2));
    }

    private static List<LatLng> square(double south, double west, double size) {
        return List.of(
                new LatLng(south, west),
                new LatLng(south, west + size),
                new LatLng(south + size, west + size),
                new LatLng(south + size, west)
        );
    }
}
