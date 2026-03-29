package com.openpositioning.PositionMe.presentation.fragment;

import com.google.android.gms.maps.model.LatLng;
import com.openpositioning.PositionMe.sensors.SensorFusion;
import com.openpositioning.PositionMe.utils.MapConstraintRepository;

import org.junit.After;
import org.junit.Test;

import java.lang.reflect.Field;
import java.util.Collections;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

public class MapDisplayConstraintFilterTest {

    @After
    public void tearDown() throws Exception {
        MapConstraintRepository.clear();
        clearPendingDisplayFloorResetToken();
    }

    @Test
    public void holdsSameFloorUpdateThatWouldCrossWall() {
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(
                0,
                Collections.singletonList(square(0.0, 0.0, 0.0001)),
                null
        );

        assertTrue(MapDisplayConstraintFilter.shouldHoldPositionForIllegalTransition(
                new LatLng(0.00005, -0.00005),
                new LatLng(0.00005, 0.00015),
                0,
                0
        ));
    }

    @Test
    public void allowsSameFloorUpdateThatStaysLegal() {
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(
                0,
                Collections.singletonList(square(0.0, 0.0, 0.0001)),
                null
        );

        assertFalse(MapDisplayConstraintFilter.shouldHoldPositionForIllegalTransition(
                new LatLng(-0.00005, -0.00005),
                new LatLng(-0.00005, 0.00015),
                0,
                0
        ));
    }

    @Test
    public void constrainsSameFloorShortCorrectionToLastLegalPoint() {
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(
                0,
                Collections.singletonList(square(0.0, 0.0, 0.0001)),
                null
        );

        LatLng start = new LatLng(0.00005, -0.000005);
        LatLng candidate = new LatLng(0.00005, 0.000101);

        LatLng constrained = MapDisplayConstraintFilter.constrainToRenderableSegmentPrefix(
                start,
                candidate,
                0,
                0
        );

        assertNotNull(constrained);
        assertTrue(MapDisplayConstraintFilter.isRenderablePoint(constrained, 0));
        assertTrue(MapDisplayConstraintFilter.isRenderableSegment(start, constrained, 0, 0));
        assertFalse(MapDisplayConstraintFilter.isRenderableSegment(start, candidate, 0, 0));
    }

    @Test
    public void doesNotClampAlreadyRenderableSameFloorSegment() {
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(
                0,
                Collections.singletonList(square(0.0, 0.0, 0.0001)),
                null
        );

        LatLng start = new LatLng(-0.00005, -0.00005);
        LatLng candidate = new LatLng(-0.00005, 0.00003);

        LatLng constrained = MapDisplayConstraintFilter.constrainToRenderableSegmentPrefix(
                start,
                candidate,
                0,
                0
        );

        assertEquals(candidate, constrained);
    }

    @Test
    public void doesNotConstrainCrossFloorOrFarCorrection() {
        LatLng start = new LatLng(0.0, 0.0);
        LatLng candidate = new LatLng(0.0, 0.01);

        assertNull(MapDisplayConstraintFilter.constrainToRenderableSegmentPrefix(
                start,
                candidate,
                0,
                1
        ));
        assertNull(MapDisplayConstraintFilter.constrainToRenderableSegmentPrefix(
                start,
                candidate,
                0,
                0
        ));
    }

    @Test
    public void bypassesSmoothingWhenDisplayedInterpolationWouldCrossWall() {
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(
                0,
                Collections.singletonList(square(0.0, 0.0, 0.0001)),
                null
        );

        assertTrue(MapDisplayConstraintFilter.shouldBypassSmoothing(
                new LatLng(0.00005, -0.00005),
                new LatLng(0.00005, 0.00015),
                0,
                0
        ));
    }

    @Test
    public void displayFallbackPrefersRenderableRawCandidateOverIllegalSmoothedPoint() {
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(
                0,
                Collections.singletonList(square(0.0, 0.0, 0.0001)),
                null
        );

        LatLng current = new LatLng(-0.00005, -0.00005);
        LatLng rawCandidate = new LatLng(-0.00005, 0.00015);
        LatLng smoothedCandidateInsideWall = new LatLng(0.00005, 0.00005);

        LatLng resolved = MapDisplayConstraintFilter.resolveBestRenderableDisplayLocation(
                current,
                rawCandidate,
                smoothedCandidateInsideWall,
                0,
                0,
                null
        );

        assertEquals(rawCandidate, resolved);
    }

    @Test
    public void displayFallbackAllowsLegalCrossFloorCandidateWhenTokenFlowHasAlreadyApprovedFloorChange() {
        LatLng current = new LatLng(55.9444, -3.1878);
        LatLng candidate = new LatLng(55.94441, -3.18779);

        LatLng resolved = MapDisplayConstraintFilter.resolveBestRenderableDisplayLocation(
                current,
                candidate,
                candidate,
                1,
                2,
                null
        );

        assertEquals(candidate, resolved);
    }

    @Test
    public void crossFloorDisplayFallbackUsesLegalLandingCorrection() {
        MapConstraintRepository.replaceVenueConstraints("venue", Collections.emptyList());
        MapConstraintRepository.setConstraintsForFloor(
                2,
                Collections.singletonList(square(0.0, 0.0, 0.0001)),
                null
        );

        LatLng current = new LatLng(55.9444, -3.1878);
        LatLng illegalTarget = new LatLng(0.00005, 0.00005);
        LatLng legalLanding = new LatLng(-0.00005, -0.00005);

        LatLng resolved = MapDisplayConstraintFilter.resolveBestRenderableDisplayLocation(
                current,
                illegalTarget,
                null,
                1,
                2,
                legalLanding
        );

        assertEquals(legalLanding, resolved);
    }

    @Test
    public void crossFloorUpdateWithoutResetTokenIsHeld() {
        assertTrue(DisplayFloorResetGate.shouldHoldCrossFloorUpdate(1, 2, false));
    }

    @Test
    public void crossFloorUpdateWithLegalResetTokenIsAllowed() throws Exception {
        long nowMs = 2_000L;
        setPendingDisplayFloorResetToken(2, nowMs + 500L);

        boolean consumed = SensorFusion.getInstance().consumePendingDisplayFloorReset(2, nowMs);

        assertTrue(consumed);
        assertFalse(DisplayFloorResetGate.shouldHoldCrossFloorUpdate(1, 2, consumed));
    }

    @Test
    public void crossFloorUpdateWithMismatchedResetTokenIsHeld() throws Exception {
        long nowMs = 2_000L;
        setPendingDisplayFloorResetToken(3, nowMs + 500L);

        boolean consumed = SensorFusion.getInstance().consumePendingDisplayFloorReset(2, nowMs);

        assertFalse(consumed);
        assertTrue(DisplayFloorResetGate.shouldHoldCrossFloorUpdate(1, 2, consumed));
    }

    @Test
    public void staleResetTokenDoesNotAllowCrossFloorUpdate() throws Exception {
        long nowMs = 2_000L;
        setPendingDisplayFloorResetToken(2, nowMs - 1L);

        boolean consumed = SensorFusion.getInstance().consumePendingDisplayFloorReset(2, nowMs);

        assertFalse(consumed);
        assertTrue(DisplayFloorResetGate.shouldHoldCrossFloorUpdate(1, 2, consumed));
    }

    @Test
    public void sameFloorUpdateStillBehavesNormally() {
        assertFalse(DisplayFloorResetGate.shouldHoldCrossFloorUpdate(1, 1, false));
    }

    private static java.util.List<LatLng> square(double south, double west, double size) {
        return java.util.List.of(
                new LatLng(south, west),
                new LatLng(south, west + size),
                new LatLng(south + size, west + size),
                new LatLng(south + size, west)
        );
    }

    private static void setPendingDisplayFloorResetToken(int floor, long untilMs) throws Exception {
        SensorFusion sensorFusion = SensorFusion.getInstance();
        setSensorFusionField(sensorFusion, "pendingDisplayFloorResetAbsoluteFloor", floor);
        setSensorFusionField(sensorFusion, "pendingDisplayFloorResetUntilMs", untilMs);
    }

    private static void clearPendingDisplayFloorResetToken() throws Exception {
        SensorFusion sensorFusion = SensorFusion.getInstance();
        setSensorFusionField(sensorFusion, "pendingDisplayFloorResetAbsoluteFloor", Integer.MIN_VALUE);
        setSensorFusionField(sensorFusion, "pendingDisplayFloorResetUntilMs", Long.MIN_VALUE);
        setSensorFusionField(sensorFusion, "pendingDisplayFloorResetLandingLatLng", null);
        setSensorFusionField(sensorFusion, "pendingDisplayFloorResetAnchorSource", "none");
        setSensorFusionField(sensorFusion, "pendingCommittedDisplayFloorAbsolute", Integer.MIN_VALUE);
        setSensorFusionField(sensorFusion, "floorSwitchPending", false);
    }

    private static void setSensorFusionField(
            SensorFusion sensorFusion,
            String fieldName,
            Object value
    ) throws Exception {
        Field field = SensorFusion.class.getDeclaredField(fieldName);
        field.setAccessible(true);
        field.set(sensorFusion, value);
    }
}
