package com.openpositioning.PositionMe.utils;

import androidx.annotation.Nullable;

import java.util.Locale;

final class FloorKeyParser {

    private FloorKeyParser() {
    }

    static int parseFloorNameToAbsoluteFloorOrZero(@Nullable String floorName) {
        Integer parsedFloor = tryParseFloorNameToAbsoluteFloor(floorName);
        return parsedFloor != null ? parsedFloor : 0;
    }

    @Nullable
    static Integer parseRawFloorValue(@Nullable Object rawValue) {
        if (rawValue instanceof Number) {
            return (int) Math.round(((Number) rawValue).doubleValue());
        }
        if (rawValue instanceof String) {
            return tryParseFloorNameToAbsoluteFloor((String) rawValue);
        }
        return null;
    }

    @Nullable
    static Integer tryParseFloorNameToAbsoluteFloor(@Nullable String floorName) {
        if (floorName == null) {
            return null;
        }

        String normalized = floorName.trim();
        if (normalized.isEmpty()) {
            return null;
        }

        String upper = normalized.toUpperCase(Locale.US)
                .replace('_', ' ')
                .replaceAll("\\s+", " ")
                .trim();
        try {
            if (upper.matches("-?\\d+")) {
                return Integer.parseInt(upper);
            }
            String semanticUpper = upper.replace('-', ' ');
            if ("GF".equals(semanticUpper) || "G".equals(semanticUpper)
                    || "GROUND".equals(semanticUpper) || "GROUND FLOOR".equals(semanticUpper)) {
                return 0;
            }
            if ("LG".equals(semanticUpper) || "LGF".equals(semanticUpper)
                    || "LOWER GROUND".equals(semanticUpper)
                    || "LOWER GROUND FLOOR".equals(semanticUpper)) {
                return -1;
            }
            if ("UG".equals(semanticUpper) || "UGF".equals(semanticUpper)
                    || "UPPER GROUND".equals(semanticUpper)
                    || "UPPER GROUND FLOOR".equals(semanticUpper)) {
                return 1;
            }
            if ("B".equals(semanticUpper) || "BASEMENT".equals(semanticUpper)) {
                return -1;
            }
            if (semanticUpper.startsWith("B") || semanticUpper.startsWith("BASEMENT")) {
                String basementDigits = semanticUpper.replaceAll("[^\\d]", "");
                if (!basementDigits.isEmpty()) {
                    return -Integer.parseInt(basementDigits);
                }
            }
            if (semanticUpper.startsWith("F")
                    || semanticUpper.startsWith("L")
                    || semanticUpper.startsWith("LEVEL")
                    || semanticUpper.startsWith("FLOOR")) {
                String floorDigits = upper.replaceAll("[^\\d-]", "");
                if (!floorDigits.isEmpty() && !"-".equals(floorDigits)) {
                    return Integer.parseInt(floorDigits);
                }
            }
        } catch (RuntimeException ignored) {
            return null;
        }
        return null;
    }
}
