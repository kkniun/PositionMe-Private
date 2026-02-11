# Workflow: Dynamic Trajectory Color Overlay (Wi-Fi RSSI)

## Scope
- 在不改动原楼层渲染与原轨迹 polyline 行为的前提下，新增可开关的动态轨迹颜色叠加层。
- 叠加层必须自维护对象列表并独立清理，不影响 floor overlay 与原轨迹对象。

## Implementation Constraints
- 不替换原 polyline；原 `updateUserLocation` 主逻辑保持不变。
- 不修改 `clearMapAndReset` 的既有语义，只允许增量追加 overlay 自清理。
- 叠加层默认关闭；开启后新增彩色分段；关闭后不再新增分段。
- 禁止 `map.clear()`。

## Data Path
1. `WifiDataProcessor`
- 增加 `volatile latestStrongestRssi` 缓存。
- 每次扫描后更新 strongest RSSI。
- 暴露只读 getter。

2. `SensorFusion`
- 只读转发 getter，禁止修改录制/protobuf 写入逻辑。

## Render Path
1. 新增 `DynamicTrajectoryOverlay`
- 维护：`segments`、`lastPoint`、`enabled`。
- 接口：`setEnabled`、`onNewPoint`、`clear`。
- 颜色映射：
- `rssi >= -55` -> green
- `-70 <= rssi < -55` -> yellow
- `rssi < -70` -> red

2. `TrajectoryMapFragment`
- 在原点更新逻辑完成后，追加调用 overlay。
- 初始化时读取 `SharedPreferences` key `dyn_traj_enabled`（默认 false）。
- Debug 构建下提供长按入口切换开关并持久化 key。
- reset/clear 流程中追加 `overlay.clear()`。

## Test Strategy
- `WifiDataProcessorTest`：覆盖 strongest RSSI 计算纯函数。
- `DynamicTrajectoryOverlayTest`：覆盖 RSSI->color 纯函数。
- 不引入新测试框架。

## Validation Commands
- `./gradlew lint`
- `./gradlew testDebugUnitTest`
- `./gradlew assembleDebug`
- `./gradlew installDebug`（若无设备，应记录失败原因）

## Manual Verification (ADB)
- `adb shell am force-stop com.openpositioning.PositionMe`
- `adb shell am start -n com.openpositioning.PositionMe/.presentation.activity.MainActivity`
- `adb logcat | rg DYN_TRAJ_OVERLAY`

## Acceptance Checklist
- 默认关闭时：楼层图、楼层切换、原轨迹表现不变。
- 开启后：出现额外彩色分段线。
- 关闭后：不再新增分段，既有分段保留，楼层图不受影响。
