#!/usr/bin/env bash
# run_continuity_test.sh - 一键 camera→fusion 连续性测试
# 流程: 清 SHM → mp4_replay(先建 SHM) → radar_camera(red) → radar_fusion(camera on, 主题 remap) → location_recorder → 分析
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
RADAR_WS="${RADAR_WS:-/workspace}"
HIK_SHM="/hikcamera_shm"
DURATION="${DURATION:-60}"
UNIT_FACTOR="${UNIT_FACTOR:-0.01}"
GAP_THRESHOLD="${GAP_THRESHOLD:-10}"
ENEMY_COLOR="${ENEMY_COLOR:-red}"
JUMP_THRESHOLD="${JUMP_THRESHOLD:-3.0}"
VIDEO=""
MAX_FRAMES=0

while [[ $# -gt 0 ]]; do
    case $1 in
        --video) VIDEO="$2"; shift 2 ;;
        --duration) DURATION="$2"; shift 2 ;;
        --max-frames) MAX_FRAMES="$2"; shift 2 ;;
        --unit-factor) UNIT_FACTOR="$2"; shift 2 ;;
        --color) ENEMY_COLOR="$2"; shift 2 ;;
        *) echo "unknown: $1" >&2; exit 2 ;;
    esac
done

if [[ -z "$VIDEO" ]]; then
    echo "usage: run_continuity_test.sh --video <mp4> [--duration 60] [--max-frames N] [--unit-factor x]" >&2
    exit 2
fi

IN_CONTAINER=0
[ -f /.dockerenv ] && IN_CONTAINER=1

exec_in() {
    if [ "$IN_CONTAINER" = "1" ]; then
        bash -c "$1"
    else
        docker exec devcontainer-radar-develop-1 bash -c "$1"
    fi
}

host_path() {
    echo "${1/#\/workspace/${ROOT_DIR}}"
}

RECORDER="${RADAR_WS}/tools/video_zmq/location_recorder.py"
ANALYZE="${ROOT_DIR}/tools/video_zmq/analyze_continuity.py"
OUT_DIR="${RADAR_WS}/tools/video_zmq/out"
mkdir -p "$(host_path "$OUT_DIR")"
CSV_OUT="${OUT_DIR}/continuity_$(date +%Y%m%d_%H%M%S).csv"

stop_all() {
    exec_in "pkill -f '[r]adar_camera' 2>/dev/null || true; pkill -f '[r]adar_fusion' 2>/dev/null || true; pkill -f '[l]ocation_recorder' 2>/dev/null || true; sleep 1; pkill -f '[m]p4_replay' 2>/dev/null || true; sleep 1; rm -f /dev/shm/hikcamera_shm 2>/dev/null || true"
}

trap stop_all EXIT
stop_all

ENV_SETUP="export ENEMY_COLOR=${ENEMY_COLOR}; source /opt/ros/jazzy/setup.bash && source ${RADAR_WS}/ros_ws/install/setup.bash && cd ${RADAR_WS}/ros_ws"

echo "[1/5] mp4_replay first (creates SHM, max-frames=${MAX_FRAMES})"
REPLAY_ARGS="--video '${VIDEO}' --shm ${HIK_SHM} --speed 1.0"
[ "$MAX_FRAMES" -gt 0 ] && REPLAY_ARGS="${REPLAY_ARGS} --max-frames ${MAX_FRAMES}"
exec_in "cd ${RADAR_WS}/tools/video_zmq/build && LD_LIBRARY_PATH=${RADAR_WS}/ros_ws/install/hikcamera/lib:${RADAR_WS}/ros_ws/third-party/hikcamera_sdk/src/sdk/lib:/opt/MVS/lib/64 setsid bash -c 'exec ./mp4_replay ${REPLAY_ARGS} > /tmp/cont_replay.log 2>&1' < /dev/null &"
echo "waiting for SHM ${HIK_SHM} ..."
SHM_OK=0
for i in $(seq 1 60); do
    if exec_in "[ -e /dev/shm${HIK_SHM} ]"; then
        SHM_OK=1
        break
    fi
    sleep 0.5
done
if [ "$SHM_OK" != "1" ]; then
    echo "error: SHM ${HIK_SHM} not created by mp4_replay (see /tmp/cont_replay.log)" >&2
    exit 1
fi
sleep 3

echo "[2/5] start radar_camera (enemy=${ENEMY_COLOR})"
exec_in "${ENV_SETUP} && setsid bash -c 'exec ros2 run radar_camera radar_camera_node --ros-args --params-file ${RADAR_WS}/ros_ws/install/radar_bringup/share/radar_bringup/config/camera/radar_camera.yaml -p enemy_color:=${ENEMY_COLOR} -p pub_topic_name:=/radar_camera/robot_pose > /tmp/cont_camera.log 2>&1' < /dev/null &"
sleep 3

echo "[3/5] start radar_fusion (camera fusion on, topic /radar_camera/robot_pose)"
exec_in "${ENV_SETUP} && setsid bash -c 'exec ros2 run radar_fusion radar_fusion_node --ros-args -p enable_camera_fusion:=true -p enemy_color:=${ENEMY_COLOR} > /tmp/cont_fusion.log 2>&1' < /dev/null &"
sleep 2

echo "[4/5] record ${DURATION}s -> ${CSV_OUT}"
exec_in "source /opt/ros/jazzy/setup.bash && source ${RADAR_WS}/ros_ws/install/setup.bash && timeout ${DURATION} python3 ${RECORDER} --out ${CSV_OUT} --duration ${DURATION} 2>&1 | tail -3"
# 等待 replay 结束（若 max-frames 先于 duration 结束）
sleep 2

echo "[5/5] analyze"
python3 "${ANALYZE}" "$(host_path "${CSV_OUT}")" --gap-threshold "${GAP_THRESHOLD}" --jump-threshold "${JUMP_THRESHOLD}" --unit-factor "${UNIT_FACTOR}" --plot "$(host_path "${OUT_DIR}")/traj.png"
echo "CSV: $(host_path "${CSV_OUT}")"
echo "PLOT: $(host_path "${OUT_DIR}")/traj.png"
echo "logs: /tmp/cont_camera.log /tmp/cont_fusion.log /tmp/cont_replay.log"
