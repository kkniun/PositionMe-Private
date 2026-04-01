package com.openpositioning.PositionMe.utils;

/**
 * 负责显示层楼层同步策略，避免 UI 在楼层初始对齐前过早锁死，也避免后续被 WiFi 抖动拖走。
 */
public final class FloorDisplaySyncPolicy {

    public static final class Decision {
        private final int logicalFloor;
        private final boolean requiresStability;
        private final boolean commitToFusion;
        private final boolean completesInitialSync;

        Decision(int logicalFloor,
                 boolean requiresStability,
                 boolean commitToFusion,
                 boolean completesInitialSync) {
            this.logicalFloor = logicalFloor;
            this.requiresStability = requiresStability;
            this.commitToFusion = commitToFusion;
            this.completesInitialSync = completesInitialSync;
        }

        public int getLogicalFloor() {
            return logicalFloor;
        }

        public boolean requiresStability() {
            return requiresStability;
        }

        public boolean shouldCommitToFusion() {
            return commitToFusion;
        }

        public boolean shouldCompleteInitialSync() {
            return completesInitialSync;
        }
    }

    private FloorDisplaySyncPolicy() {
    }

    public static Decision resolve(boolean hasCommittedInitialSync,
                                   boolean hasWifiFloor,
                                   int wifiFloor,
                                   int fusedFloor,
                                   boolean nearTransitionFeature) {
        if (!hasWifiFloor) {
            return new Decision(fusedFloor, false, false, false);
        }

        if (!hasCommittedInitialSync) {
            if (wifiFloor == fusedFloor) {
                return new Decision(fusedFloor, true, false, true);
            }
            return new Decision(wifiFloor, true, true, true);
        }

        if (wifiFloor == fusedFloor) {
            return new Decision(wifiFloor, false, false, false);
        }

        if (nearTransitionFeature) {
            return new Decision(wifiFloor, true, true, false);
        }

        return new Decision(fusedFloor, false, false, false);
    }
}
