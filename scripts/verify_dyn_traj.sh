#!/usr/bin/env bash
set -euo pipefail

APP_ID="com.openpositioning.PositionMe"
MAIN_ACTIVITY=".presentation.activity.MainActivity"

# --- ADB path (Mac default). If adb already in PATH, we will use that.
if command -v adb >/dev/null 2>&1; then
  ADB="adb"
else
  ADB="$HOME/Library/Android/sdk/platform-tools/adb"
fi

if [[ ! -x "$(command -v "$ADB" 2>/dev/null || true)" && ! -x "$ADB" ]]; then
  echo "ERROR: adb not found. Please install Android platform-tools or fix ANDROID_HOME."
  exit 1
fi

# Pick the first connected device (USB). You can override: SERIAL=xxxx ./scripts/verify_dyn_traj.sh
SERIAL="${SERIAL:-$($ADB devices | awk 'NR>1 && $2=="device"{print $1; exit}')}"
if [[ -z "${SERIAL}" ]]; then
  echo "ERROR: no adb device found. Run: $ADB devices"
  exit 1
fi

echo "==> Using device: $SERIAL"
echo "==> Build & checks..."
./gradlew lint testDebugUnitTest

echo "==> Install debug APK..."
./gradlew installDebug

echo "==> Restart app..."
$ADB -s "$SERIAL" logcat -c
$ADB -s "$SERIAL" shell am force-stop "$APP_ID"

# Enable overlay via Intent extra (requires the Debug-only MainActivity change)
# --ez is boolean extra.
$ADB -s "$SERIAL" shell am start -n "$APP_ID/$APP_ID$MAIN_ACTIVITY" --ez dyn_traj_enabled true

echo "==> Logcat (Ctrl+C to stop). Watching WIFI_RSSI / WIFI_RSSI_GET / DYN_TRAJ_OVERLAY ..."
$ADB -s "$SERIAL" logcat -s WIFI_RSSI:D WIFI_RSSI_GET:D DYN_TRAJ_OVERLAY:D *:S
