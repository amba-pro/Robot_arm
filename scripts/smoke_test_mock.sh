#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS="${ROOT_DIR}/ros_ws"
LOG_DIR="${ROOT_DIR}/.smoke_logs"
mkdir -p "${LOG_DIR}"

ANGLE_LOG="${LOG_DIR}/angle_reader.log"
SERVER_LOG="${LOG_DIR}/robot_tcp_server.log"
LAUNCH_LOG="${LOG_DIR}/ros_launch.log"

ANGLE_PID=""
SERVER_PID=""
LAUNCH_PID=""

cleanup() {
  for pid in "${LAUNCH_PID}" "${SERVER_PID}" "${ANGLE_PID}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "[FAIL] Missing required command: $1"
    if [[ "$1" == "ros2" ]]; then
      echo
      echo "Hint:"
      echo "  1) Source your existing ROS distro:"
      echo "     source /opt/ros/\$ROS_DISTRO/setup.bash"
      echo "  2) Or source directly, for example Jazzy:"
      echo "     source /opt/ros/jazzy/setup.bash"
      echo "  3) Then rerun this script."
    fi
    exit 1
  fi
}

echo "[1/7] Checking base commands..."
require_cmd python3
require_cmd colcon
require_cmd ros2

echo "[2/7] Building ROS workspace..."
cd "${ROS_WS}"
colcon build >/dev/null

echo "[3/7] Starting mock angle reader..."
cd "${ROOT_DIR}"
ARM4_MOCK=1 python3 angle_reader.py >"${ANGLE_LOG}" 2>&1 &
ANGLE_PID=$!
sleep 3
if ! kill -0 "${ANGLE_PID}" 2>/dev/null; then
  echo "[FAIL] angle_reader.py did not stay running"
  exit 1
fi

echo "[4/7] Validating angles cache update..."
python3 - <<'PY'
import json
import os
import time

cache = "angles_cache.json"
deadline = time.time() + 8
ok = False
while time.time() < deadline:
    if os.path.exists(cache):
        try:
            with open(cache, "r", encoding="utf-8") as f:
                data = json.load(f)
            angles = data.get("angles", {})
            if all(k in angles for k in ["A0", "A1", "A2", "A3", "A4"]):
                ok = True
                break
        except Exception:
            pass
    time.sleep(0.3)
if not ok:
    raise SystemExit(1)
PY

echo "[5/7] Starting mock TCP server..."
ARM4_MOCK=1 python3 robot_tcp_server.py >"${SERVER_LOG}" 2>&1 &
SERVER_PID=$!
sleep 3
if ! kill -0 "${SERVER_PID}" 2>/dev/null; then
  echo "[FAIL] robot_tcp_server.py did not stay running"
  exit 1
fi

echo "[6/7] Starting ROS launch (headless)..."
cd "${ROS_WS}"
set +u
source install/setup.bash
set -u
ros2 launch arm4_bringup arm4_bringup.launch.py rviz:=false rqt:=false >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!
sleep 6
if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  echo "[FAIL] ROS launch exited early"
  exit 1
fi

echo "[7/7] Checking ROS topics..."
timeout 6 ros2 topic echo /arm4/angles --once >/dev/null
timeout 6 ros2 topic echo /joint_states --once >/dev/null

echo
echo "[PASS] Mock smoke test completed successfully."
echo "Logs:"
echo "  - ${ANGLE_LOG}"
echo "  - ${SERVER_LOG}"
echo "  - ${LAUNCH_LOG}"
