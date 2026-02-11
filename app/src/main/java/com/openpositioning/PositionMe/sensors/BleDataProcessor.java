package com.openpositioning.PositionMe.sensors;

import android.Manifest;
import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothManager;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanResult;
import android.bluetooth.le.ScanSettings;
import android.content.Context;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.util.Log;
import android.widget.Toast;

import androidx.core.app.ActivityCompat;

import com.openpositioning.PositionMe.BuildConfig;
import android.bluetooth.le.BluetoothLeScanner;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * BLE data gathering and processing using BluetoothAdapter discovery + BroadcastReceiver.
 *
 * Debug logs tag: BLE_PIPE
 *
 * Design:
 * - Periodically start discovery (default every 5s)
 * - Use MAC as dedup key within a window
 * - On ACTION_DISCOVERY_FINISHED: flush (notifyObservers) and clear cache
 *
 * Note:
 * - Android 9 (API 28) style permissions: requires ACCESS_FINE_LOCATION for discovery results.
 * - Emulator may not support Bluetooth => bluetoothAdapter == null (expected).
 */
public class BleDataProcessor implements Observable {

    // Use a dedicated TAG so you can filter Logcat by it.
    private static final String TAG = "BLE_PIPE";
    private static final int DEFAULT_STRONGEST_RSSI = -100;

    private static final long SCAN_INTERVAL_MS = 5000;
    private static final long TOAST_DEBOUNCE_MS = 8000;
    private static final long LOG_SAMPLE_DEBOUNCE_MS = 3000;
    private String lastToastMsg = "";
    private long lastToastTimeMs = 0L;

    private final Context context;
    private final BluetoothAdapter bluetoothAdapter;
    private BluetoothLeScanner bluetoothLeScanner;

    private final ArrayList<Observer> observers = new ArrayList<>();
    private ScanCallback scanCallback;

    // Flush window every SCAN_INTERVAL_MS
    private final Handler mainHandler = new Handler(Looper.getMainLooper());

    // MAC -> latest obs in current window
    private final Map<String, BLE> windowMap = new HashMap<>();
    private long lastLogSampleMs = 0L;
    // 供 UI 只读展示的 BLE 概览统计，默认值与页面占位保持一致。
    private volatile int latestBleDeviceCount = 0;
    private volatile int latestStrongestBleRssi = DEFAULT_STRONGEST_RSSI;

    public BleDataProcessor(Context context) {
        this.context = context.getApplicationContext();

        BluetoothManager bluetoothManager =
                (BluetoothManager) this.context.getSystemService(Context.BLUETOOTH_SERVICE);

        BluetoothAdapter adapter = null;
        if (bluetoothManager != null) {
            adapter = bluetoothManager.getAdapter();
        }
        this.bluetoothAdapter = adapter;

        Log.i(TAG, "BleDataProcessor() init: bluetoothAdapter=" + (bluetoothAdapter == null ? "null" : "OK"));
    }

    // -------------------- Observable --------------------

    @Override
    public void registerObserver(Observer o) {
        observers.add(o);
        Log.i(TAG, "registerObserver(): totalObservers=" + observers.size());
    }

    @Override
    public void notifyObservers(int idx) {
        BLE[] data;
        synchronized (windowMap) {
            data = windowMap.values().toArray(new BLE[0]);
        }
        sendToObservers(data);
    }

    // -------------------- Public lifecycle --------------------

    public void startListening() {
        Log.i(TAG, "startListening() called");

        if (bluetoothAdapter == null) {
            Log.e(TAG, "BluetoothAdapter == null (Bluetooth not supported / emulator limitation)");
            showDebouncedToast("Bluetooth not supported on this device", Toast.LENGTH_SHORT);
            return;
        }

        if (!bluetoothAdapter.isEnabled()) {
            Log.e(TAG, "Bluetooth is disabled");
            showDebouncedToast("Please enable Bluetooth", Toast.LENGTH_SHORT);
            return;
        }

        if (!checkBlePermissions()) {
            Log.e(TAG, "Missing BLE permissions");
            showDebouncedToast("Missing Bluetooth permissions", Toast.LENGTH_SHORT);
            return;
        }

        bluetoothLeScanner = bluetoothAdapter.getBluetoothLeScanner();
        if (bluetoothLeScanner == null) {
            Log.e(TAG, "BluetoothLeScanner is null (adapter disabled?)");
            return;
        }

        stopScanningInternal(); // ensure clean state
        startScanningInternal();
        scheduleFlush();

        if (BuildConfig.DEBUG) {
            Log.d(TAG, "BLE scan started (mode=LOW_LATENCY, interval=" + SCAN_INTERVAL_MS + "ms)");
        }
    }

    public void stopListening() {
        Log.i(TAG, "stopListening() called");

        stopScanningInternal();
        cancelFlush();
        synchronized (windowMap) {
            windowMap.clear();
            latestBleDeviceCount = 0;
            latestStrongestBleRssi = DEFAULT_STRONGEST_RSSI;
        }

        if (BuildConfig.DEBUG) {
            Log.d(TAG, "BLE scan stopped");
        }
    }

    // -------------------- Core scanning --------------------

    // -------------------- Permissions --------------------

    private boolean checkBlePermissions() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            int scan = ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_SCAN);
            int connect = ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_CONNECT);
            return scan == PackageManager.PERMISSION_GRANTED && connect == PackageManager.PERMISSION_GRANTED;
        }
        int fineLoc = ActivityCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION);
        return fineLoc == PackageManager.PERMISSION_GRANTED;
    }

    // -------------------- Scan helpers --------------------

    @SuppressLint("MissingPermission")
    private void startScanningInternal() {
        if (bluetoothLeScanner == null) return;

        ScanSettings settings = new ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                .build();

        scanCallback = new ScanCallback() {
            @Override
            public void onScanResult(int callbackType, ScanResult result) {
                handleScanResult(result);
            }

            @Override
            public void onBatchScanResults(java.util.List<ScanResult> results) {
                if (results == null) return;
                for (ScanResult r : results) {
                    handleScanResult(r);
                }
            }

            @Override
            public void onScanFailed(int errorCode) {
                Log.e(TAG, "onScanFailed(): errorCode=" + errorCode);
            }
        };

        bluetoothLeScanner.startScan(null, settings, scanCallback);
    }

    @SuppressLint("MissingPermission")
    private void stopScanningInternal() {
        if (bluetoothLeScanner != null && scanCallback != null) {
            bluetoothLeScanner.stopScan(scanCallback);
        }
        scanCallback = null;
    }

    @SuppressLint("MissingPermission")
    private void handleScanResult(ScanResult result) {
        if (result == null) return;
        if (!checkBlePermissions()) {
            Log.w(TAG, "handleScanResult(): permissions revoked during scan, stopping");
            stopListening();
            return;
        }

        BluetoothDevice device = result.getDevice();
        String mac = (device != null) ? device.getAddress() : null;
        String uuid = extractServiceUuid(result);
        String dedupKey = buildDeviceDedupKey(mac, uuid);
        if (dedupKey == null) return; // address / uuid 都不可用时无法去重

        BLE obs = new BLE();
        obs.setMac(mac);
        obs.setName(device != null ? device.getName() : null);
        obs.setRssi(result.getRssi());
        obs.setUuid(uuid == null ? "unknown" : uuid);
        long tsMs = result.getTimestampNanos() > 0
                ? result.getTimestampNanos() / 1_000_000L
                : SystemClock.elapsedRealtime();
        obs.setTimestampMs(tsMs);

        synchronized (windowMap) {
            windowMap.put(dedupKey, obs);
            updateLatestBleStatsLocked();
        }

        long now = SystemClock.elapsedRealtime();
        if (BuildConfig.DEBUG && (windowMap.size() <= 5 || now - lastLogSampleMs >= LOG_SAMPLE_DEBOUNCE_MS)) {
            Log.d(TAG, "scan result key=" + dedupKey + " mac=" + mac + " rssi=" + result.getRssi()
                    + " unique=" + windowMap.size());
            lastLogSampleMs = now;
        }
    }

    // 根据 address/uuid 生成本次扫描窗口内的去重 key。
    static String buildDeviceDedupKey(String mac, String uuid) {
        if (mac != null && !mac.isEmpty()) {
            return "mac:" + mac;
        }
        if (uuid != null && !uuid.isEmpty()) {
            return "uuid:" + uuid;
        }
        return null;
    }

    // 提取扫描结果中的首个 service UUID，作为 address 缺失时的后备去重标识。
    private String extractServiceUuid(ScanResult result) {
        if (result.getScanRecord() == null || result.getScanRecord().getServiceUuids() == null
                || result.getScanRecord().getServiceUuids().isEmpty()) {
            return null;
        }
        return result.getScanRecord().getServiceUuids().get(0).toString();
    }

    // 在持有 windowMap 锁时更新 UI 统计，避免展示值与缓存状态不一致。
    private void updateLatestBleStatsLocked() {
        int size = windowMap.size();
        int[] rssiValues = new int[size];
        String[] dedupKeys = new String[size];
        int index = 0;
        for (Map.Entry<String, BLE> entry : windowMap.entrySet()) {
            dedupKeys[index] = entry.getKey();
            rssiValues[index] = entry.getValue().getRssi();
            index++;
        }
        latestBleDeviceCount = countUniqueBleDeviceKeys(dedupKeys);
        latestStrongestBleRssi = calculateStrongestBleRssi(rssiValues);
    }

    /**
     * 计算最强 RSSI（越大信号越强，通常是负数）。空输入返回默认值 -100。
     */
    static int calculateStrongestBleRssi(int[] rssiValues) {
        if (rssiValues == null || rssiValues.length == 0) {
            return DEFAULT_STRONGEST_RSSI;
        }
        int maxRssi = rssiValues[0];
        for (int value : rssiValues) {
            if (value > maxRssi) {
                maxRssi = value;
            }
        }
        return maxRssi;
    }

    /**
     * 按设备去重 key 统计数量，忽略 null/空字符串。
     */
    static int countUniqueBleDeviceKeys(String[] deviceKeys) {
        if (deviceKeys == null || deviceKeys.length == 0) {
            return 0;
        }
        Set<String> keySet = new HashSet<>();
        for (String key : deviceKeys) {
            if (key != null && !key.isEmpty()) {
                keySet.add(key);
            }
        }
        return keySet.size();
    }

    public int getLatestBleDeviceCount() {
        return latestBleDeviceCount;
    }

    public int getLatestStrongestBleRssi() {
        return latestStrongestBleRssi;
    }

    private void scheduleFlush() {
        cancelFlush();
        mainHandler.postDelayed(flushRunnable, SCAN_INTERVAL_MS);
    }

    private void cancelFlush() {
        mainHandler.removeCallbacks(flushRunnable);
    }

    private final Runnable flushRunnable = new Runnable() {
        @Override
        public void run() {
            flushWindow();
            mainHandler.postDelayed(this, SCAN_INTERVAL_MS);
        }
    };

    private void flushWindow() {
        BLE[] data;
        synchronized (windowMap) {
            if (windowMap.isEmpty()) return;
            data = windowMap.values().toArray(new BLE[0]);
            windowMap.clear();
        }
        if (BuildConfig.DEBUG) {
            Log.d(TAG, "flushWindow(): sending " + data.length + " entries to observers");
        }
        sendToObservers(data);
    }

    private void sendToObservers(BLE[] data) {
        Log.i(TAG, "notifyObservers(): sending count=" + data.length + " to observers=" + observers.size());
        for (Observer o : observers) {
            try {
                o.update(data);
            } catch (Exception e) {
                Log.e(TAG, "notifyObservers(): observer.update() crashed", e);
            }
        }
    }

    // -------------------- Toast helper --------------------

    private void showDebouncedToast(String message, int duration) {
        long now = SystemClock.elapsedRealtime();
        if (message.equals(lastToastMsg) && now - lastToastTimeMs <= TOAST_DEBOUNCE_MS) return;
        lastToastMsg = message;
        lastToastTimeMs = now;
        Toast.makeText(context, message, duration).show();
    }
}
