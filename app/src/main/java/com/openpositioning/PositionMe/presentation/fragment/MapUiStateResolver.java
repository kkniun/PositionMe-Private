package com.openpositioning.PositionMe.presentation.fragment;

import androidx.annotation.Nullable;

final class MapUiStateResolver {

    @Nullable
    private static Boolean rememberedAutoFloorState;

    private MapUiStateResolver() {
    }

    @Nullable
    static Boolean resolveSavedOrRememberedAutoFloorState(
            @Nullable Boolean savedAutoFloorState,
            boolean autoFloorStateInitialized,
            boolean autoFloorEnabledState
    ) {
        if (savedAutoFloorState != null) {
            return savedAutoFloorState;
        }
        if (autoFloorStateInitialized) {
            return autoFloorEnabledState;
        }
        return rememberedAutoFloorState;
    }

    static boolean resolveRestoredAutoFloorState(
            @Nullable Boolean savedAutoFloorState,
            boolean currentCheckedState
    ) {
        return savedAutoFloorState != null ? savedAutoFloorState : currentCheckedState;
    }

    static void rememberAutoFloorState(boolean enabled) {
        rememberedAutoFloorState = enabled;
    }

    static void clearRememberedAutoFloorStateForTesting() {
        rememberedAutoFloorState = null;
    }
}
