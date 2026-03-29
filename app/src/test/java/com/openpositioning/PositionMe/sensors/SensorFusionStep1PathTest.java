package com.openpositioning.PositionMe.sensors;

import com.google.android.gms.maps.model.LatLng;
import com.openpositioning.PositionMe.utils.CoordinateConverter;
import com.openpositioning.PositionMe.utils.MapConstraintRepository;

import org.junit.After;
import org.junit.Test;

import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class SensorFusionStep1PathTest {

    private static final double ORIGIN_LAT = 55.9444;
    private static final double ORIGIN_LON = -3.1878;

    @After
    public void tearDown() {
        MapConstraintRepository.clear();
    }

    @Test
    public void gnssAbsoluteFixThroughSensorFusionInitializesFusedPoseInLocalFrame() {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        ParticleFilterEngine engine = createDeterministicEngine();

        double latitudeDeg = ORIGIN_LAT + 0.0005;
        double longitudeDeg = ORIGIN_LON + 0.0002;
        FusedPose pose = SensorFusion.applyAbsoluteFixForStep1(
                engine,
                converter,
                false,
                latitudeDeg,
                longitudeDeg,
                3,
                null,
                1000L,
                5.0f,
                0.0f
        );

        double[] expectedLocal = converter.toLocalMeters(latitudeDeg, longitudeDeg);
        assertEquals(expectedLocal[0], pose.getX(), 1e-6);
        assertEquals(expectedLocal[1], pose.getY(), 1e-6);
        assertEquals(3, pose.getFloor());
    }

    @Test
    public void wifiAbsoluteFixThroughSensorFusionReanchorsAndUsesWifiFloor() {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        ParticleFilterEngine engine = createDeterministicEngine();

        SensorFusion.applyAbsoluteFixForStep1(
                engine,
                converter,
                false,
                ORIGIN_LAT,
                ORIGIN_LON,
                0,
                null,
                1000L,
                4.0f,
                0.0f
        );

        LatLng wifiFix = converter.toLatLng(30.0, 15.0);
        FusedPose pose = SensorFusion.applyAbsoluteFixForStep1(
                engine,
                converter,
                true,
                wifiFix.latitude,
                wifiFix.longitude,
                2,
                2,
                1100L,
                4.0f,
                0.0f
        );

        assertEquals(30.0, pose.getX(), 1e-6);
        assertEquals(15.0, pose.getY(), 1e-6);
        assertEquals(2, pose.getFloor());
    }

    @Test
    public void pdrThenGnssSequenceThroughSensorFusionKeepsStep1AxesConsistent() {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        ParticleFilterEngine engine = createDeterministicEngine();

        SensorFusion.applyAbsoluteFixForStep1(
                engine,
                converter,
                false,
                ORIGIN_LAT,
                ORIGIN_LON,
                0,
                null,
                1000L,
                4.0f,
                0.0f
        );

        FusedPose afterPdr = SensorFusion.applyPdrPredictionForStep1(
                engine,
                true,
                new PdrDelta(1.0f, 0.0f, 0.0f),
                0,
                1100L
        );
        assertEquals(0.0, afterPdr.getX(), 1e-6);
        assertEquals(1.0, afterPdr.getY(), 1e-6);

        LatLng gnssFix = converter.toLatLng(0.0, 10.0);
        FusedPose afterGnss = SensorFusion.applyAbsoluteFixForStep1(
                engine,
                converter,
                true,
                gnssFix.latitude,
                gnssFix.longitude,
                5,
                null,
                1200L,
                1.0f,
                0.0f
        );

        assertEquals(0.0, afterGnss.getX(), 1e-6);
        assertEquals(10.0, afterGnss.getY(), 1e-6);
        assertTrue(afterGnss.getTimestampMs() >= 1200L);
    }

    @Test
    public void gnssReanchorThroughSensorFusionKeepsExistingFloorWhenNoFloorPriorIsAvailable() {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        ParticleFilterEngine engine = createDeterministicEngine();

        SensorFusion.applyAbsoluteFixForStep1(
                engine,
                converter,
                false,
                ORIGIN_LAT,
                ORIGIN_LON,
                0,
                null,
                1000L,
                4.0f,
                0.0f
        );

        LatLng gnssFix = converter.toLatLng(40.0, 0.0);
        FusedPose pose = SensorFusion.applyAbsoluteFixForStep1(
                engine,
                converter,
                true,
                gnssFix.latitude,
                gnssFix.longitude,
                7,
                null,
                1100L,
                4.0f,
                0.0f
        );

        assertEquals(40.0, pose.getX(), 1e-6);
        assertEquals(0.0, pose.getY(), 1e-6);
        assertEquals(0, pose.getFloor());
    }

    @Test
    public void clampCandidateToLegalSegmentPrefixShortensWallCrossingStep() {
        CoordinateConverter converter = new CoordinateConverter(ORIGIN_LAT, ORIGIN_LON);
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(
                0,
                Collections.singletonList(localRectangle(converter, -0.5, -1.0, 1.0, 2.0)),
                null
        );

        FusedPose previousPose = new FusedPose(-2.0, 0.0, 0, 0.8, 1000L);
        FusedPose candidatePose = new FusedPose(2.0, 0.0, 0, 0.8, 1100L);

        FusedPose clampedPose = SensorFusion.clampCandidateToLegalSegmentPrefix(
                previousPose,
                candidatePose,
                converter
        );

        assertNotNull(clampedPose);
        assertTrue(clampedPose.getX() > previousPose.getX());
        assertTrue(clampedPose.getX() < candidatePose.getX());
        assertTrue(MapConstraintRepository.isPathLegal(
                converter.toLatLng(previousPose.getX(), previousPose.getY()),
                converter.toLatLng(clampedPose.getX(), clampedPose.getY()),
                previousPose.getFloor()
        ));
    }

    private ParticleFilterEngine createDeterministicEngine() {
        return new ParticleFilterEngine(
                new ParticleInitializer(new ZeroRandom()),
                ParticleInitializer.allowAll(),
                new ZeroRandom()
        );
    }

    private List<LatLng> localRectangle(
            CoordinateConverter converter,
            double westXMeters,
            double southYMeters,
            double widthMeters,
            double heightMeters
    ) {
        return List.of(
                converter.toLatLng(westXMeters, southYMeters),
                converter.toLatLng(westXMeters + widthMeters, southYMeters),
                converter.toLatLng(westXMeters + widthMeters, southYMeters + heightMeters),
                converter.toLatLng(westXMeters, southYMeters + heightMeters)
        );
    }

    private static final class ZeroRandom extends Random {
        @Override
        public double nextGaussian() {
            return 0.0;
        }

        @Override
        public double nextDouble() {
            return 0.5;
        }
    }
}
