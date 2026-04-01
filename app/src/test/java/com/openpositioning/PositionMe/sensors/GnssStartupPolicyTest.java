package com.openpositioning.PositionMe.sensors;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class GnssStartupPolicyTest {

    @Test
    public void build_allProvidersEnabled_requestsBothWithoutNotice() {
        GnssStartupPolicy.Plan plan = GnssStartupPolicy.build(true, true, true);

        assertTrue(plan.shouldRequestGps());
        assertTrue(plan.shouldRequestNetwork());
        assertTrue(plan.shouldStartUpdates());
        assertEquals(GnssStartupPolicy.Notice.NONE, plan.getNotice());
    }

    @Test
    public void build_gpsOnly_startsGpsWithoutBlockingInitialization() {
        GnssStartupPolicy.Plan plan = GnssStartupPolicy.build(true, true, false);

        assertTrue(plan.shouldRequestGps());
        assertFalse(plan.shouldRequestNetwork());
        assertTrue(plan.shouldStartUpdates());
        assertEquals(GnssStartupPolicy.Notice.NONE, plan.getNotice());
    }

    @Test
    public void build_networkOnly_usesFallbackAndWarnsAboutGps() {
        GnssStartupPolicy.Plan plan = GnssStartupPolicy.build(true, false, true);

        assertFalse(plan.shouldRequestGps());
        assertTrue(plan.shouldRequestNetwork());
        assertTrue(plan.shouldStartUpdates());
        assertEquals(
                GnssStartupPolicy.Notice.ENABLE_GPS_FOR_BETTER_ACCURACY,
                plan.getNotice()
        );
    }

    @Test
    public void build_noProviders_requestsNothingAndShowsLocationNotice() {
        GnssStartupPolicy.Plan plan = GnssStartupPolicy.build(true, false, false);

        assertFalse(plan.shouldRequestGps());
        assertFalse(plan.shouldRequestNetwork());
        assertFalse(plan.shouldStartUpdates());
        assertEquals(
                GnssStartupPolicy.Notice.ENABLE_LOCATION_SERVICES,
                plan.getNotice()
        );
    }
}
