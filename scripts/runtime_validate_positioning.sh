#!/usr/bin/env bash
set -euo pipefail

APP_ID="com.openpositioning.PositionMe"
MAIN_ACTIVITY="com.openpositioning.PositionMe.presentation.activity.MainActivity"
OUTPUT_DIR="${1:-artifacts/runtime_validation_$(date +%Y%m%d_%H%M%S)}"

mkdir -p "${OUTPUT_DIR}"

if ! command -v adb >/dev/null 2>&1; then
  echo "adb is not available on PATH." >&2
  exit 1
fi

DEVICE_COUNT="$(adb devices | awk 'NR > 1 && $2 == "device" {count++} END {print count+0}')"
if [[ "${DEVICE_COUNT}" -lt 1 ]]; then
  echo "No adb device is connected." >&2
  exit 1
fi

cat > "${OUTPUT_DIR}/README.txt" <<'EOF'
Runtime validation checklist
1. Floor change:
   - Look for SensorFloorDiag / FloorDiag events such as:
     accepted_absolute_floor_prior
     wrong_floor_recovery_accepted
     DISPLAY_RESET_TOKEN_ISSUED
     DISPLAY_RESET_TOKEN_CONSUMED
2. Absolute fix recovery:
   - Look for MOTION_DIAG / SensorFusion events such as:
     absolute_fix decision
     absolute_fix pf_result
     rollback_particle_change_kept_previous_pose
     suppressed_during_elevator
3. Display sync:
   - Look for FloorDiag / TrajectoryMapFragment events such as:
     map_display_floor_selected
     cross_floor_not_authorized
     accepted_cross_floor_commit
4. Suggested walkthrough:
   - Start on one floor and record a stable marker.
   - Trigger a stairs/lift transition and check how quickly floor text and marker switch.
   - Force a wrong-floor or wrong-corridor state, then wait for a high-quality WiFi/GNSS fix.
   - Confirm that the marker snaps closer to the fused pose without a long visual tail.
EOF

echo "Installing debug build..."
./gradlew installDebug

echo "Clearing previous logcat buffer..."
adb logcat -c

echo "Launching app..."
adb shell am start -n "${APP_ID}/${MAIN_ACTIVITY}" >/dev/null

echo "Streaming focused logcat. Output: ${OUTPUT_DIR}/logcat.txt"
adb logcat -v time \
  SensorFloorDiag:D \
  MOTION_DIAG:D \
  FloorDiag:D \
  TrajectoryMapFragment:I \
  SensorFusion:I \
  '*:S' | tee "${OUTPUT_DIR}/logcat.txt"
