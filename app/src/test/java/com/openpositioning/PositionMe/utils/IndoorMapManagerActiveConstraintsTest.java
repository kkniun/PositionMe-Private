package com.openpositioning.PositionMe.utils;

import com.google.android.gms.maps.model.LatLng;

import org.junit.After;
import org.junit.Test;

import java.util.Arrays;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class IndoorMapManagerActiveConstraintsTest {

    @After
    public void tearDown() {
        MapConstraintRepository.clear();
    }

    @Test
    public void activeConstraintsRequireWallsOnCurrentFloor() {
        MapConstraintRepository.replaceVenueConstraints("venue-a", Arrays.asList(
                new LatLng(55.0, -3.0),
                new LatLng(55.0, -3.001),
                new LatLng(55.001, -3.001)
        ));
        MapConstraintRepository.setConstraintsForFloor(
                2,
                "L2",
                Arrays.asList(Arrays.asList(
                        new LatLng(55.0, -3.0),
                        new LatLng(55.0, -3.0001),
                        new LatLng(55.0001, -3.0001)
                )),
                null
        );

        assertFalse(MapConstraintReadiness.hasFloorLevelActiveMapMatchingConstraints("venue-a", true, 1));
        assertTrue(MapConstraintReadiness.hasFloorLevelActiveMapMatchingConstraints("venue-a", true, 2));
    }

    @Test
    public void outlineOnlyDoesNotCountAsActiveConstraints() {
        MapConstraintRepository.replaceVenueConstraints("venue-a", Arrays.asList(
                new LatLng(55.0, -3.0),
                new LatLng(55.0, -3.001),
                new LatLng(55.001, -3.001)
        ));
        MapConstraintRepository.setConstraintsForFloor(0, "GF", null, null);

        assertFalse(MapConstraintReadiness.hasFloorLevelActiveMapMatchingConstraints("venue-a", true, 0));
    }

    @Test
    public void particleWallReadinessRequiresCoordinateConverterAndWallsOnFloor() {
        MapConstraintRepository.replaceVenueConstraints("venue-a", Arrays.asList(
                new LatLng(55.0, -3.0),
                new LatLng(55.0, -3.001),
                new LatLng(55.001, -3.001)
        ));
        MapConstraintRepository.setConstraintsForFloor(1, "L1", null, null);
        MapConstraintRepository.setConstraintsForFloor(
                2,
                "L2",
                Arrays.asList(Arrays.asList(
                        new LatLng(55.0, -3.0),
                        new LatLng(55.0, -3.0001),
                        new LatLng(55.0001, -3.0001)
                )),
                null
        );

        assertFalse(MapConstraintReadiness.hasFloorLevelParticleWallReadiness(false, 2));
        assertFalse(MapConstraintReadiness.hasFloorLevelParticleWallReadiness(true, 1));
        assertTrue(MapConstraintReadiness.hasFloorLevelParticleWallReadiness(true, 2));
    }
}
