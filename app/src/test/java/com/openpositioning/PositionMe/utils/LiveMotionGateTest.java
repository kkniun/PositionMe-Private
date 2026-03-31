package com.openpositioning.PositionMe.utils;

import com.google.android.gms.maps.model.LatLng;

import org.junit.Test;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class LiveMotionGateTest {

    @Test
    public void trackedDistanceRequiresRealTranslationEvidence() {
        assertFalse(LiveMotionGate.shouldAccumulateTrackedDistance(
                true,
                0.80,
                false,
                "WIFI",
                10_000L,
                Long.MIN_VALUE
        ));

        assertTrue(LiveMotionGate.shouldAccumulateTrackedDistance(
                true,
                0.80,
                false,
                "PDR",
                10_000L,
                Long.MIN_VALUE
        ));

        assertTrue(LiveMotionGate.shouldAccumulateTrackedDistance(
                true,
                0.80,
                false,
                "GNSS",
                10_000L,
                9_200L
        ));

        assertFalse(LiveMotionGate.shouldAccumulateTrackedDistance(
                true,
                0.80,
                true,
                "WIFI",
                10_000L,
                Long.MIN_VALUE
        ));
    }

    @Test
    public void passiveAbsoluteFixHoldIgnoresMotionResumeWithoutConfirmedTranslation() {
        LatLng current = new LatLng(55.9444000, -3.1878000);
        LatLng smallDrift = new LatLng(55.9444100, -3.1878000);
        LatLng largeDrift = new LatLng(55.9444450, -3.1878000);

        assertTrue(LiveMotionGate.shouldHoldPoseForPassiveUpdate(
                current,
                smallDrift,
                false,
                false,
                "WIFI",
                10_000L,
                Long.MIN_VALUE
        ));

        assertTrue(LiveMotionGate.shouldHoldPoseForPassiveUpdate(
                current,
                smallDrift,
                false,
                true,
                "WIFI",
                10_000L,
                Long.MIN_VALUE
        ));

        assertFalse(LiveMotionGate.shouldHoldPoseForPassiveUpdate(
                current,
                largeDrift,
                false,
                false,
                "WIFI",
                10_000L,
                Long.MIN_VALUE
        ));

        assertFalse(LiveMotionGate.shouldHoldPoseForPassiveUpdate(
                current,
                smallDrift,
                false,
                false,
                "GNSS",
                10_000L,
                9_200L
        ));
    }

    @Test
    public void tinyStationaryJitterAlwaysStaysPinned() {
        LatLng current = new LatLng(55.9444000, -3.1878000);
        LatLng jitter = new LatLng(55.9444010, -3.1878000);

        assertTrue(LiveMotionGate.shouldHoldPoseForPassiveUpdate(
                current,
                jitter,
                true,
                false,
                "GNSS",
                5_000L,
                Long.MIN_VALUE
        ));
    }
}
