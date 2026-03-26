package com.openpositioning.PositionMe.sensors;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

public class AbsoluteFloorTransitionResolverTest {

    @Test
    public void acceptsReportedFloorBeforePfInitialization() {
        AbsoluteFloorTransitionResolver resolver = new AbsoluteFloorTransitionResolver();

        Integer accepted = resolver.resolveAcceptedFloor(
                0,
                3,
                false,
                true,
                false
        );

        assertEquals(Integer.valueOf(3), accepted);
    }

    @Test
    public void requiresRepeatedAbsoluteEvidenceWithoutTransitionConstraints() {
        AbsoluteFloorTransitionResolver resolver = new AbsoluteFloorTransitionResolver();

        assertNull(resolver.resolveAcceptedFloor(0, 2, true, false, false));
        assertEquals(
                Integer.valueOf(2),
                resolver.resolveAcceptedFloor(0, 2, true, false, false)
        );
    }

    @Test
    public void rejectsConstraintBreakingFloorJumpWhenTransitionConstraintsExist() {
        AbsoluteFloorTransitionResolver resolver = new AbsoluteFloorTransitionResolver();

        Integer accepted = resolver.resolveAcceptedFloor(
                1,
                2,
                true,
                true,
                false
        );

        assertNull(accepted);
    }

    @Test
    public void acceptsConstraintCompliantFloorJumpImmediately() {
        AbsoluteFloorTransitionResolver resolver = new AbsoluteFloorTransitionResolver();

        Integer accepted = resolver.resolveAcceptedFloor(
                1,
                2,
                true,
                true,
                true
        );

        assertEquals(Integer.valueOf(2), accepted);
    }
}
