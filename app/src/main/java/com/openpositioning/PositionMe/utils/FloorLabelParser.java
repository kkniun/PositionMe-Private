package com.openpositioning.PositionMe.utils;

import androidx.annotation.Nullable;

import java.util.Locale;

/**
 * 统一解析楼层标签，兼容 GF / 1F / F1 / B1F / L2 等常见写法。
 */
public final class FloorLabelParser {

    private FloorLabelParser() {
    }

    @Nullable
    public static Integer parseLogicalFloorLabel(@Nullable String rawLabel) {
        if (rawLabel == null) {
            return null;
        }

        String normalized = rawLabel.trim().toUpperCase(Locale.UK);
        if (normalized.isEmpty()) {
            return null;
        }

        normalized = normalized
                .replace("FLOOR", "")
                .replace("LEVEL", "")
                .replace("STOREY", "")
                .replace("STORY", "")
                .replace("_", "")
                .replace("-", "")
                .replace(" ", "");

        if ("G".equals(normalized) || "GF".equals(normalized) || "GROUND".equals(normalized)) {
            return 0;
        }
        if ("LG".equals(normalized) || "LOWGROUND".equals(normalized)
                || "LOWERGROUND".equals(normalized)) {
            return -1;
        }
        if ("UG".equals(normalized) || "UPGROUND".equals(normalized)
                || "UPPERGROUND".equals(normalized)) {
            return 1;
        }

        if (normalized.endsWith("F") && normalized.length() > 1) {
            normalized = normalized.substring(0, normalized.length() - 1);
        }

        if (normalized.startsWith("B") && normalized.length() > 1) {
            try {
                return -Integer.parseInt(normalized.substring(1));
            } catch (NumberFormatException ignored) {
                return null;
            }
        }

        if (normalized.startsWith("L") && normalized.length() > 1) {
            try {
                return Integer.parseInt(normalized.substring(1));
            } catch (NumberFormatException ignored) {
                return null;
            }
        }

        if (normalized.startsWith("F") && normalized.length() > 1) {
            try {
                return Integer.parseInt(normalized.substring(1));
            } catch (NumberFormatException ignored) {
                return null;
            }
        }

        try {
            return Integer.parseInt(normalized);
        } catch (NumberFormatException ignored) {
            return null;
        }
    }
}
