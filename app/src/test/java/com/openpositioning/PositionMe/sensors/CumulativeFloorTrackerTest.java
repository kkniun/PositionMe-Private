package com.openpositioning.PositionMe.sensors;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class CumulativeFloorTrackerTest {

    @Test
    public void cumulativeHeightCommitsFloorChangeAfterSustainedClimb() {
        CumulativeFloorTracker tracker = new CumulativeFloorTracker();

        assertEquals(0, tracker.update(0f, 3.6f, 1_000L));
        assertEquals(0, tracker.update(3.10f, 3.6f, 2_000L));
        assertEquals(0, tracker.update(3.30f, 3.6f, 3_000L));
        assertEquals(1, tracker.update(3.30f, 3.6f, 4_000L));
    }

    @Test
    public void cumulativeHeightSupportsDescendingBackDown() {
        CumulativeFloorTracker tracker = new CumulativeFloorTracker();

        tracker.update(0f, 3.6f, 1_000L);
        tracker.update(3.10f, 3.6f, 2_000L);
        tracker.update(3.30f, 3.6f, 3_000L);
        tracker.update(3.30f, 3.6f, 4_000L);

        assertEquals(1, tracker.getRelativeFloor());
        assertEquals(1, tracker.update(0.40f, 3.6f, 5_000L));
        assertEquals(0, tracker.update(0.20f, 3.6f, 6_000L));
    }

    @Test
    public void strongTransitionEvidenceExpiresAfterWindow() {
        CumulativeFloorTracker tracker = new CumulativeFloorTracker();

        tracker.update(0f, 4.0f, 1_000L);
        tracker.update(3.60f, 4.0f, 2_000L);
        tracker.update(3.80f, 4.0f, 3_000L);
        tracker.update(3.80f, 4.0f, 4_000L);

        assertTrue(tracker.hasRecentStrongTransitionEvidence(0, 1, 5_000L));
        assertFalse(tracker.hasRecentStrongTransitionEvidence(0, 1, 15_500L));
        assertFalse(tracker.hasRecentStrongTransitionEvidence(1, 0, 5_000L));
    }

    @Test
    public void descendingTransitionAlsoGeneratesStrongEvidence() {
        CumulativeFloorTracker tracker = new CumulativeFloorTracker();

        tracker.update(0f, 3.6f, 1_000L);
        tracker.update(3.10f, 3.6f, 2_000L);
        tracker.update(3.30f, 3.6f, 3_000L);
        tracker.update(3.30f, 3.6f, 4_000L);

        assertEquals(1, tracker.getRelativeFloor());
        assertEquals(1, tracker.update(0.40f, 3.6f, 5_000L));
        assertEquals(0, tracker.update(0.20f, 3.6f, 6_000L));
        assertTrue(tracker.hasRecentStrongTransitionEvidence(1, 0, 6_500L));
    }

    @Test
    public void clampRelativeFloorPreventsTrackerFromRunningPastVenueBounds() {
        CumulativeFloorTracker tracker = new CumulativeFloorTracker();

        tracker.update(0f, 3.6f, 1_000L);
        tracker.update(3.10f, 3.6f, 2_000L);
        tracker.update(3.30f, 3.6f, 3_000L);
        tracker.update(3.30f, 3.6f, 4_000L);

        assertEquals(1, tracker.getRelativeFloor());
        assertTrue(tracker.clampRelativeFloor(0, 0));
        assertEquals(0, tracker.getRelativeFloor());
        assertEquals(0f, tracker.getPendingHeightDeltaMeters(), 1e-6f);
    }

    @Test
    public void reanchorResetsRelativeFloorAtCurrentElevation() {
        CumulativeFloorTracker tracker = new CumulativeFloorTracker();

        tracker.update(0f, 3.6f, 1_000L);
        tracker.update(3.10f, 3.6f, 2_000L);
        tracker.update(3.30f, 3.6f, 3_000L);
        tracker.update(3.30f, 3.6f, 4_000L);

        assertEquals(1, tracker.getRelativeFloor());

        tracker.reanchorToCurrentElevation(3.30f);

        assertEquals(0, tracker.getRelativeFloor());
        assertEquals(0f, tracker.getPendingHeightDeltaMeters(), 1e-6f);
        assertEquals(0, tracker.update(3.45f, 3.6f, 5_000L));
    }

    @Test
    public void transientBarometerSpikeDoesNotImmediatelyCommitFloorChange() {
        CumulativeFloorTracker tracker = new CumulativeFloorTracker();

        assertEquals(0, tracker.update(0f, 3.6f, 1_000L));
        assertEquals(0, tracker.update(3.40f, 3.6f, 2_000L));
        assertEquals(0, tracker.update(2.20f, 3.6f, 3_000L));
        assertEquals(0, tracker.update(3.40f, 3.6f, 4_000L));
        assertEquals(1, tracker.update(3.40f, 3.6f, 5_000L));
    }
}
