#!/usr/bin/env bash
set -euo pipefail

PKG="com.openpositioning.PositionMe"
DEFAULT_ADB="${HOME}/Library/Android/sdk/platform-tools/adb"

if [[ -n "${ADB_BIN:-}" ]]; then
  ADB="${ADB_BIN}"
elif command -v adb >/dev/null 2>&1; then
  ADB="$(command -v adb)"
else
  ADB="${DEFAULT_ADB}"
fi

if [[ ! -x "${ADB}" ]]; then
  echo "adb not found. Set ADB_BIN or install Android platform-tools." >&2
  exit 1
fi

DEVICE_SERIAL="${DEVICE_SERIAL:-}"
if [[ -z "${DEVICE_SERIAL}" ]]; then
  DEVICE_SERIAL="$("${ADB}" devices | awk 'NR>1 && $2=="device"{print $1; exit}')"
fi

if [[ -z "${DEVICE_SERIAL}" ]]; then
  echo "No connected Android device in 'device' state." >&2
  "${ADB}" devices
  exit 1
fi

OUT_DIR="${OUT_DIR:-$(pwd)}"
mkdir -p "${OUT_DIR}"

TAP_RECORD_X="${TAP_RECORD_X:-301}"
TAP_RECORD_Y="${TAP_RECORD_Y:-1408}"
TAP_SET_X="${TAP_SET_X:-540}"
TAP_SET_Y="${TAP_SET_Y:-2200}"
TAP_MAP_X="${TAP_MAP_X:-540}"
TAP_MAP_Y="${TAP_MAP_Y:-1200}"

tap() {
  local x="$1"
  local y="$2"
  "${ADB}" -s "${DEVICE_SERIAL}" shell input tap "${x}" "${y}"
}

capture() {
  local file="$1"
  "${ADB}" -s "${DEVICE_SERIAL}" exec-out screencap -p > "${OUT_DIR}/${file}"
  echo "saved ${OUT_DIR}/${file}"
}

echo "[1/6] adb devices"
"${ADB}" devices

echo "[2/6] start app"
"${ADB}" -s "${DEVICE_SERIAL}" shell am force-stop "${PKG}" || true
if ! "${ADB}" -s "${DEVICE_SERIAL}" shell am start -n "${PKG}/.MainActivity" >/tmp/open_floorplan_start.log 2>&1; then
  "${ADB}" -s "${DEVICE_SERIAL}" shell am start -n "${PKG}/.presentation.activity.MainActivity" >/tmp/open_floorplan_start.log 2>&1
fi
sleep 3
capture "step1_home.png"

echo "[3/6] tap Record (${TAP_RECORD_X},${TAP_RECORD_Y})"
tap "${TAP_RECORD_X}" "${TAP_RECORD_Y}"
sleep 3
capture "step2_after_record.png"

echo "[4/6] tap Set (${TAP_SET_X},${TAP_SET_Y})"
tap "${TAP_SET_X}" "${TAP_SET_Y}"
sleep 6
capture "step3_recording_map.png"

echo "[5/6] tap map/polygon (${TAP_MAP_X},${TAP_MAP_Y}) to select venue and show floorplan"
tap "${TAP_MAP_X}" "${TAP_MAP_Y}"
sleep 4
capture "step4_floorplan_selected.png"

echo "[6/6] done"
echo "device=${DEVICE_SERIAL}"
echo "record=(${TAP_RECORD_X},${TAP_RECORD_Y}) set=(${TAP_SET_X},${TAP_SET_Y}) map=(${TAP_MAP_X},${TAP_MAP_Y})"
echo "screenshots saved in: ${OUT_DIR}"
