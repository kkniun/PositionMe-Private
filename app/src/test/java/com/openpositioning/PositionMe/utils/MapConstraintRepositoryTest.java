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
    private static final double ORIGIN_LAT = 55.9444;
    private static final double ORIGIN_LON = -3.1878;

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

    @Test
    public void insideOrNearLiftAcceptsNearbyPointWithinTolerance() {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setLiftsForFloor(
                1,
                Collections.singletonList(localSquare(converter, 0.0, 0.0, 2.0))
        );

        assertTrue(MapConstraintRepository.isPointInsideOrNearLift(
                converter.toLatLng(4.5, 1.0),
                1,
                3.0
        ));
        assertFalse(MapConstraintRepository.isPointInsideOrNearLift(
                converter.toLatLng(5.5, 1.0),
                1,
                3.0
        ));
    }

    @Test
    public void insideOrNearStairsAcceptsNearbyPointWithinTolerance() {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setStairsForFloor(
                3,
                Collections.singletonList(localSquare(converter, -1.0, -1.0, 2.0))
        );

        assertTrue(MapConstraintRepository.isPointInsideOrNearStairs(
                converter.toLatLng(2.5, 0.0),
                3,
                2.0
        ));
        assertFalse(MapConstraintRepository.isPointInsideOrNearStairs(
                converter.toLatLng(4.5, 0.0),
                3,
                2.0
        ));
    }

    @Test
    public void doesPathExitVenueOutlineReturnsTrueForConcaveBoundaryEscape() {
        MapConstraintRepository.replaceVenueConstraints("venue", concaveVenueOutline());

        assertTrue(MapConstraintRepository.doesPathExitVenueOutline(
                new LatLng(2.5, 0.5),
                new LatLng(0.5, 2.5)
        ));
    }

    @Test
    public void isPointLegalRejectsWallInterior() {
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(
                0,
                Collections.singletonList(square(0.0, 0.0, 0.0001)),
                null
        );

        assertFalse(MapConstraintRepository.isPointLegal(new LatLng(0.00005, 0.00005), 0));
        assertTrue(MapConstraintRepository.isPointLegal(new LatLng(0.00015, 0.00015), 0));
    }

    @Test
    public void isPathLegalRejectsWallCrossingSegment() {
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(
                0,
                Collections.singletonList(square(0.0, 0.0, 0.0001)),
                null
        );

        assertFalse(MapConstraintRepository.isPathLegal(
                new LatLng(0.00005, -0.00005),
                new LatLng(0.00005, 0.00015),
                0
        ));
        assertTrue(MapConstraintRepository.isPathLegal(
                new LatLng(-0.00005, -0.00005),
                new LatLng(-0.00005, 0.00015),
                0
        ));
    }

    private static List<LatLng> square(double south, double west, double size) {
        return List.of(
                new LatLng(south, west),
                new LatLng(south, west + size),
                new LatLng(south + size, west + size),
                new LatLng(south + size, west)
        );
    }

    private static List<LatLng> concaveVenueOutline() {
        return List.of(
                new LatLng(0.0, 0.0),
                new LatLng(3.0, 0.0),
                new LatLng(3.0, 1.0),
                new LatLng(1.0, 1.0),
                new LatLng(1.0, 3.0),
                new LatLng(0.0, 3.0)
        );
    }

    private static List<LatLng> localSquare(
            CoordinateConverter converter,
            double westXMeters,
            double southYMeters,
            double sizeMeters
    ) {
        return List.of(
                converter.toLatLng(westXMeters, southYMeters),
                converter.toLatLng(westXMeters + sizeMeters, southYMeters),
                converter.toLatLng(westXMeters + sizeMeters, southYMeters + sizeMeters),
                converter.toLatLng(westXMeters, southYMeters + sizeMeters)
        );
    }
}
