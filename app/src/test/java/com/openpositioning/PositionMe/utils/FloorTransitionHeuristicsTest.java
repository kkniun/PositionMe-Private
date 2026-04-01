package com.openpositioning.PositionMe.utils;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class FloorTransitionHeuristicsTest {

    @Test
    public void evaluate_withoutSemanticSupport_rejectsFloorChange() {
        FloorTransitionHeuristics.Decision decision = FloorTransitionHeuristics.evaluate(
                1,
                4.1f,
                4.0f,
                2.2,
                3500L,
                false,
                false,
                false,
                2
        );

        assertEquals(FloorTransitionHeuristics.Mode.NONE, decision.getMode());
        assertEquals(1, decision.getTargetFloor());
    }

    @Test
    public void evaluate_stairsEvidence_onlyMovesOneFloor() {
        FloorTransitionHeuristics.Decision decision = FloorTransitionHeuristics.evaluate(
                2,
                4.2f,
                4.0f,
                3.5,
                4000L,
                true,
                false,
                false,
                null
        );

        assertEquals(FloorTransitionHeuristics.Mode.STAIRS, decision.getMode());
        assertEquals(3, decision.getTargetFloor());
    }

    @Test
    public void evaluate_liftEvidence_allowsMultiFloorJump() {
        FloorTransitionHeuristics.Decision decision = FloorTransitionHeuristics.evaluate(
                1,
                8.4f,
                4.0f,
                0.6,
                5000L,
                false,
                true,
                true,
                null
        );

        assertEquals(FloorTransitionHeuristics.Mode.LIFT, decision.getMode());
        assertEquals(3, decision.getTargetFloor());
    }

    @Test
    public void evaluate_wifiHintAlone_doesNotForceTransition() {
        FloorTransitionHeuristics.Decision decision = FloorTransitionHeuristics.evaluate(
                0,
                4.3f,
                4.0f,
                0.4,
                4500L,
                false,
                false,
                false,
                2
        );

        assertEquals(FloorTransitionHeuristics.Mode.NONE, decision.getMode());
        assertEquals(0, decision.getTargetFloor());
    }

    @Test
    public void evaluate_typicalStairsClimb_transitionsWithLowerButRealisticDelta() {
        FloorTransitionHeuristics.Decision decision = FloorTransitionHeuristics.evaluate(
                1,
                2.7f,
                4.0f,
                1.4,
                3200L,
                true,
                false,
                false,
                null
        );

        assertEquals(FloorTransitionHeuristics.Mode.STAIRS, decision.getMode());
        assertEquals(2, decision.getTargetFloor());
    }

    @Test
    public void evaluate_typicalLiftRide_allowsSingleFloorTransitionWithoutHugeVerticalRate() {
        FloorTransitionHeuristics.Decision decision = FloorTransitionHeuristics.evaluate(
                1,
                2.8f,
                4.0f,
                1.1,
                4200L,
                false,
                true,
                true,
                null
        );

        assertEquals(FloorTransitionHeuristics.Mode.LIFT, decision.getMode());
        assertEquals(2, decision.getTargetFloor());
    }
}
