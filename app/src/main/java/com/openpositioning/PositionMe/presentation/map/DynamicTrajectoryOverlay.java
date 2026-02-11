package com.openpositioning.PositionMe.presentation.map;

import android.util.Log;

import androidx.annotation.NonNull;

import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Polyline;
import com.google.android.gms.maps.model.PolylineOptions;
import com.openpositioning.PositionMe.BuildConfig;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * 动态轨迹颜色叠加层：只维护自身的线段对象，不影响原有轨迹与楼层渲染。
 */
public class DynamicTrajectoryOverlay {
    private static final String TAG = "DYN_TRAJ_OVERLAY";
    private static final float SEGMENT_WIDTH = 6f;

    // 颜色常量使用 int，便于在 JVM 单测中直接校验映射结果
    static final int COLOR_GREEN = 0xFF4CAF50;
    static final int COLOR_YELLOW = 0xFFFFC107;
    static final int COLOR_RED = 0xFFF44336;

    private final List<Polyline> segments = new ArrayList<>();
    private LatLng lastPoint;
    private boolean enabled;

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public boolean isEnabled() {
        return enabled;
    }

    /**
     * 按 RSSI 生成一段叠加线。
     * 未开启或没有上一个点时，仅更新 lastPoint，不执行绘制。
     */
    public void onNewPoint(@NonNull GoogleMap map, @NonNull LatLng point, int rssi) {
        if (!enabled || lastPoint == null) {
            lastPoint = point;
            return;
        }

        int color = resolveColorForRssi(rssi);
        Polyline segment = map.addPolyline(new PolylineOptions()
                .add(lastPoint, point)
                .color(color)
                .width(SEGMENT_WIDTH));
        segments.add(segment);

        if (BuildConfig.DEBUG) {
            Log.d(TAG, String.format(Locale.US, "segment added rssi=%d color=0x%08X", rssi, color));
        }

        lastPoint = point;
    }

    /**
     * 只清理叠加层自身对象，保持地图其他层（如楼层图、原轨迹）不受影响。
     */
    public void clear() {
        for (Polyline segment : segments) {
            if (segment != null) {
                segment.remove();
            }
        }
        segments.clear();
        lastPoint = null;
    }

    /**
     * RSSI 到线段颜色的映射规则，供渲染与单测共用。
     */
    static int resolveColorForRssi(int rssi) {
        if (rssi >= -55) {
            return COLOR_GREEN;
        }
        if (rssi >= -70) {
            return COLOR_YELLOW;
        }
        return COLOR_RED;
    }
}
