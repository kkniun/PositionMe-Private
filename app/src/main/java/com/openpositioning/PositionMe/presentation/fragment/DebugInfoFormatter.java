package com.openpositioning.PositionMe.presentation.fragment;

import java.util.Locale;

/**
 * 调试信息文本格式化工具。
 *
 * 该类仅负责把页面读取到的只读数据拼接为可复制文本，
 * 不参与任何采集、上报、轨迹或 protobuf 写入流程。
 */
public final class DebugInfoFormatter {

    private static final String NA = "N/A";

    private DebugInfoFormatter() {
        // 工具类不需要实例化
    }

    /**
     * 纯函数：将页面关键读数格式化为多行调试文本。
     */
    static String format(
            String timestamp,
            float[] accelerometer,
            float[] gravity,
            float[] gyro,
            float[] magnetic,
            Float light,
            Float pressure,
            Float proximity,
            float[] gnss,
            float[] pdr,
            Integer wifiApCount,
            Integer wifiStrongestRssi,
            Integer bleDeviceCount,
            Integer bleStrongestRssi
    ) {
        StringBuilder builder = new StringBuilder();
        builder.append("timestamp: ").append(safeText(timestamp)).append('\n');
        builder.append("Accelerometer: ").append(formatVector(accelerometer, "X", "Y", "Z")).append('\n');
        builder.append("Gravity: ").append(formatVector(gravity, "X", "Y", "Z")).append('\n');
        builder.append("Gyro: ").append(formatVector(gyro, "X", "Y", "Z")).append('\n');
        builder.append("Mag: ").append(formatVector(magnetic, "X", "Y", "Z")).append('\n');
        builder.append("Light: ").append(formatFloatValue(light)).append('\n');
        builder.append("Pressure: ").append(formatFloatValue(pressure)).append('\n');
        builder.append("Proximity: ").append(formatFloatValue(proximity)).append('\n');
        builder.append("GNSS(lat,long): ").append(formatVector(gnss, "Lat", "Long")).append('\n');
        builder.append("PDR(x,y): ").append(formatVector(pdr, "X", "Y")).append('\n');
        builder.append("WiFi: apCount=").append(formatIntegerValue(wifiApCount))
                .append(", strongestRssi=").append(formatRssiValue(wifiStrongestRssi)).append('\n');
        builder.append("BLE: deviceCount=").append(formatIntegerValue(bleDeviceCount))
                .append(", strongestRssi=").append(formatRssiValue(bleStrongestRssi));
        return builder.toString();
    }

    private static String formatVector(float[] values, String... labels) {
        if (values == null || labels == null || values.length < labels.length) {
            return NA;
        }
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < labels.length; i++) {
            if (i > 0) {
                builder.append(", ");
            }
            builder.append(labels[i]).append('=').append(String.format(Locale.US, "%.2f", values[i]));
        }
        return builder.toString();
    }

    private static String formatFloatValue(Float value) {
        if (value == null) {
            return NA;
        }
        return String.format(Locale.US, "%.2f", value);
    }

    private static String formatIntegerValue(Integer value) {
        return value == null ? NA : String.valueOf(value);
    }

    private static String formatRssiValue(Integer rssi) {
        return rssi == null ? NA : (rssi + " dBm");
    }

    private static String safeText(String value) {
        return (value == null || value.trim().isEmpty()) ? NA : value;
    }
}
