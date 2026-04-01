package com.openpositioning.PositionMe.sensors;

/**
 * 纯逻辑的 GNSS 启动策略，方便在单元测试中覆盖 provider 组合。
 */
final class GnssStartupPolicy {

    enum Notice {
        NONE,
        ENABLE_LOCATION_SERVICES,
        ENABLE_GPS_FOR_BETTER_ACCURACY
    }

    static final class Plan {
        private final boolean requestGps;
        private final boolean requestNetwork;
        private final Notice notice;

        Plan(boolean requestGps, boolean requestNetwork, Notice notice) {
            this.requestGps = requestGps;
            this.requestNetwork = requestNetwork;
            this.notice = notice;
        }

        boolean shouldRequestGps() {
            return requestGps;
        }

        boolean shouldRequestNetwork() {
            return requestNetwork;
        }

        boolean shouldStartUpdates() {
            return requestGps || requestNetwork;
        }

        Notice getNotice() {
            return notice;
        }
    }

    private GnssStartupPolicy() {
    }

    static Plan build(boolean permissionsGranted, boolean gpsEnabled, boolean networkEnabled) {
        if (!permissionsGranted) {
            return new Plan(false, false, Notice.NONE);
        }

        if (gpsEnabled && networkEnabled) {
            return new Plan(true, true, Notice.NONE);
        }

        if (gpsEnabled) {
            return new Plan(true, false, Notice.NONE);
        }

        if (networkEnabled) {
            return new Plan(false, true, Notice.ENABLE_GPS_FOR_BETTER_ACCURACY);
        }

        return new Plan(false, false, Notice.ENABLE_LOCATION_SERVICES);
    }
}
