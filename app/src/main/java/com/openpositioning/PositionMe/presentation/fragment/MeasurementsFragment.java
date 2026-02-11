package com.openpositioning.PositionMe.presentation.fragment;

import android.content.ClipData;
import android.content.ClipboardManager;
import android.content.Context;
import android.os.Bundle;
import android.os.Handler;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.cardview.widget.CardView;
import androidx.constraintlayout.widget.ConstraintLayout;
import androidx.core.view.MenuHost;
import androidx.core.view.MenuProvider;
import androidx.fragment.app.Fragment;
import androidx.lifecycle.Lifecycle;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import com.openpositioning.PositionMe.BuildConfig;
import com.openpositioning.PositionMe.R;
import com.openpositioning.PositionMe.sensors.SensorFusion;
import com.openpositioning.PositionMe.sensors.SensorTypes;
import com.openpositioning.PositionMe.sensors.Wifi;
import com.openpositioning.PositionMe.presentation.viewitems.WifiListAdapter;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * A simple {@link Fragment} subclass. The measurement fragment displays the set of current sensor
 * readings. The values are refreshed periodically, but slower than their internal refresh rate.
 * The refresh time is set by a static constant.
 *
 * @see HomeFragment the previous fragment in the nav graph.
 * @see SensorFusion the source of all sensor readings.
 *
 * @author Mate Stodulka
 */
public class MeasurementsFragment extends Fragment {

    // Static constant for refresh time in milliseconds
    private static final long REFRESH_TIME = 5000;
    private static final long BLE_REFRESH_TIME_MS = 1000;
    private static final int DEFAULT_BLE_DEVICE_COUNT = 0;
    private static final int DEFAULT_BLE_STRONGEST_RSSI = -100;

    // Singleton Sensor Fusion class handling all sensor data
    private SensorFusion sensorFusion;

    // UI Handler
    private Handler refreshDataHandler;
    // UI elements
    private ConstraintLayout sensorMeasurementList;
    private RecyclerView wifiListView;
    private TextView bleDeviceCountText;
    private TextView bleStrongestRssiText;
    // List of string resource IDs
    private int[] prefaces;
    private int[] gnssPrefaces;


    /**
     * Public default constructor, empty.
     */
    public MeasurementsFragment() {
        // Required empty public constructor
    }

    /**
     * {@inheritDoc}
     * Obtains the singleton Sensor Fusion instance and initialises the string prefaces for display.
     * Creates a new handler to periodically refresh data.
     *
     * @see SensorFusion handles all sensor data.
     */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        // Get sensor fusion instance
        sensorFusion = SensorFusion.getInstance();
        // Initialise string prefaces for display
        prefaces =  new int[]{R.string.x, R.string.y, R.string.z};
        gnssPrefaces =  new int[]{R.string.lati, R.string.longi};

        // Create new handler to refresh the UI.
        this.refreshDataHandler = new Handler();
    }

    /**
     * {@inheritDoc}
     * Sets title in the action bar to Sensor Measurements.
     * Posts the {@link MeasurementsFragment#refreshTableTask} using the Handler.
     */
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        // Inflate the layout for this fragment
        View rootView = inflater.inflate(R.layout.fragment_measurements, container, false);
        getActivity().setTitle("Sensor Measurements");
        this.refreshDataHandler.post(refreshTableTask);
        return rootView;
    }

    /**
     * {@inheritDoc}
     * Pauses the data refreshing when the fragment is not in focus.
     */
    @Override
    public void onPause() {
        refreshDataHandler.removeCallbacks(refreshTableTask);
        refreshDataHandler.removeCallbacks(refreshBleOverviewTask);
        super.onPause();
    }

    /**
     * {@inheritDoc}
     * Restarts the data refresh when the fragment returns to focus.
     */
    @Override
    public void onResume() {
        refreshDataHandler.postDelayed(refreshTableTask, REFRESH_TIME);
        refreshDataHandler.post(refreshBleOverviewTask);
        super.onResume();
    }

    /**
     * {@inheritDoc}
     * Obtains the constraint layout holding the sensor measurement values. Initialises the Recycler
     * View for holding WiFi data and registers its Layout Manager.
     */
    @Override
    public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);
        sensorMeasurementList = (ConstraintLayout) getView().findViewById(R.id.sensorMeasurementList);
        wifiListView = (RecyclerView) getView().findViewById(R.id.wifiList);
        bleDeviceCountText = (TextView) getView().findViewById(R.id.bleDeviceCountText);
        bleStrongestRssiText = (TextView) getView().findViewById(R.id.bleStrongestRssiText);
        wifiListView.setLayoutManager(new LinearLayoutManager(getActivity()));
        updateBleOverview(DEFAULT_BLE_DEVICE_COUNT, DEFAULT_BLE_STRONGEST_RSSI);
        registerCopyDebugMenu();
    }

    /**
     * Runnable task containing functionality to update the UI with the relevant sensor data.
     * Must be run on the UI thread via a Handler. Obtains movement sensor values and the current
     * WiFi networks from the {@link SensorFusion} instance and updates the UI with the new data
     * and the string wrappers provided.
     *
     * @see SensorFusion class handling all sensors and data processing.
     * @see Wifi class holding network data.
     */
    private final Runnable refreshTableTask = new Runnable() {
        @Override
        public void run() {
            // Get all the values from SensorFusion
            Map<SensorTypes, float[]> sensorValueMap = sensorFusion.getSensorValueMap();
            // Loop through UI elements and update the values
            for(SensorTypes st : SensorTypes.values()) {
                CardView cardView = (CardView) sensorMeasurementList.getChildAt(st.ordinal());
                ConstraintLayout currentRow = (ConstraintLayout) cardView.getChildAt(0);
                float[] values = sensorValueMap.get(st);
                for (int i = 0; i < values.length; i++) {
                    String valueString;
                    // Set string wrapper based on data type.
                    if(values.length == 1) {
                        valueString = getString(R.string.level, String.format("%.2f", values[0]));
                    }
                    else if(values.length == 2){
                        if(st == SensorTypes.GNSSLATLONG)
                            valueString = getString(gnssPrefaces[i], String.format("%.2f", values[i]));
                        else
                            valueString = getString(prefaces[i], String.format("%.2f", values[i]));
                    }
                    else{
                        valueString = getString(prefaces[i], String.format("%.2f", values[i]));
                    }
                    ((TextView) currentRow.getChildAt(i + 1)).setText(valueString);
                }
            }
            // Get all WiFi values - convert to list of strings
            List<Wifi> wifiObjects = sensorFusion.getWifiList();
            // If there are WiFi networks visible, update the recycler view with the data.
            if(wifiObjects != null) {
                wifiListView.setAdapter(new WifiListAdapter(getActivity(), wifiObjects));
            }
            // Restart the data updater task in REFRESH_TIME milliseconds.
            refreshDataHandler.postDelayed(refreshTableTask, REFRESH_TIME);
        }
    };

    private final Runnable refreshBleOverviewTask = new Runnable() {
        @Override
        public void run() {
            // 只读获取 BLE 统计并刷新 UI，不触发任何扫描流程。
            int deviceCount = sensorFusion.getLatestBleDeviceCount();
            int strongestRssi = sensorFusion.getLatestStrongestBleRssi();
            updateBleOverview(deviceCount, strongestRssi);
            refreshDataHandler.postDelayed(this, BLE_REFRESH_TIME_MS);
        }
    };

    // 统一 BLE 面板文案更新，避免分散在多个生命周期回调中。
    private void updateBleOverview(int deviceCount, int strongestRssi) {
        if (bleDeviceCountText != null) {
            bleDeviceCountText.setText(getString(R.string.ble_devices_count, deviceCount));
        }
        if (bleStrongestRssiText != null) {
            bleStrongestRssiText.setText(getString(R.string.ble_strongest_rssi, strongestRssi));
        }
    }

    // 在页面工具栏添加 Copy 按钮，仅用于只读调试信息复制。
    private void registerCopyDebugMenu() {
        MenuHost menuHost = requireActivity();
        menuHost.addMenuProvider(new MenuProvider() {
            @Override
            public void onCreateMenu(@NonNull Menu menu, @NonNull MenuInflater menuInflater) {
                menuInflater.inflate(R.menu.menu_measurements, menu);
                MenuItem copyItem = menu.findItem(R.id.action_copy_debug_info);
                if (copyItem != null) {
                    copyItem.setVisible(BuildConfig.DEBUG);
                }
            }

            @Override
            public boolean onMenuItemSelected(@NonNull MenuItem menuItem) {
                if (menuItem.getItemId() == R.id.action_copy_debug_info) {
                    copyDebugInfoToClipboard();
                    return true;
                }
                return false;
            }
        }, getViewLifecycleOwner(), Lifecycle.State.RESUMED);
    }

    // 读取当前页面可见数据并复制到剪贴板，不改变任何采集/上报流程。
    private void copyDebugInfoToClipboard() {
        String debugInfoText = buildDebugInfoText();
        ClipboardManager clipboardManager = (ClipboardManager) requireContext()
                .getSystemService(Context.CLIPBOARD_SERVICE);
        if (clipboardManager != null) {
            clipboardManager.setPrimaryClip(ClipData.newPlainText("debug_info", debugInfoText));
        }
        Toast.makeText(requireContext(), R.string.copied_debug_info, Toast.LENGTH_SHORT).show();
    }

    // 组装当前页面关键读数，缺失数据统一交给 formatter 输出 N/A。
    private String buildDebugInfoText() {
        Map<SensorTypes, float[]> sensorValueMap = sensorFusion.getSensorValueMap();
        float[] accelerometer = getSensorValues(sensorValueMap, SensorTypes.ACCELEROMETER);
        float[] gravity = getSensorValues(sensorValueMap, SensorTypes.GRAVITY);
        float[] gyro = getSensorValues(sensorValueMap, SensorTypes.GYRO);
        float[] magnetic = getSensorValues(sensorValueMap, SensorTypes.MAGNETICFIELD);
        Float light = getScalarValue(sensorValueMap, SensorTypes.LIGHT);
        Float pressure = getScalarValue(sensorValueMap, SensorTypes.PRESSURE);
        Float proximity = getScalarValue(sensorValueMap, SensorTypes.PROXIMITY);
        float[] gnssLatLong = getSensorValues(sensorValueMap, SensorTypes.GNSSLATLONG);
        float[] pdr = getSensorValues(sensorValueMap, SensorTypes.PDR);

        List<Wifi> wifiObjects = sensorFusion.getWifiList();
        Integer wifiApCount = (wifiObjects == null) ? null : wifiObjects.size();
        Integer wifiStrongestRssi = getWifiStrongestRssi(wifiObjects);

        Integer bleDeviceCount = null;
        Integer bleStrongestRssi = null;
        try {
            bleDeviceCount = sensorFusion.getLatestBleDeviceCount();
            bleStrongestRssi = sensorFusion.getLatestStrongestBleRssi();
        } catch (Exception ignored) {
            // BLE 统计不可用时降级为 N/A，保证复制功能不影响主流程。
        }

        String timestamp = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.getDefault())
                .format(new Date());
        return DebugInfoFormatter.format(
                timestamp,
                accelerometer,
                gravity,
                gyro,
                magnetic,
                light,
                pressure,
                proximity,
                gnssLatLong,
                pdr,
                wifiApCount,
                wifiStrongestRssi,
                bleDeviceCount,
                bleStrongestRssi
        );
    }

    private float[] getSensorValues(Map<SensorTypes, float[]> sensorValueMap, SensorTypes type) {
        if (sensorValueMap == null) {
            return null;
        }
        return sensorValueMap.get(type);
    }

    private Float getScalarValue(Map<SensorTypes, float[]> sensorValueMap, SensorTypes type) {
        float[] values = getSensorValues(sensorValueMap, type);
        if (values == null || values.length == 0) {
            return null;
        }
        return values[0];
    }

    // WiFi 列表取最强 RSSI（数值越大信号越强）。
    private Integer getWifiStrongestRssi(List<Wifi> wifiObjects) {
        if (wifiObjects == null || wifiObjects.isEmpty()) {
            return null;
        }
        Integer strongest = null;
        for (Wifi wifi : wifiObjects) {
            if (wifi == null) {
                continue;
            }
            if (strongest == null || wifi.getLevel() > strongest) {
                strongest = wifi.getLevel();
            }
        }
        return strongest;
    }
}
