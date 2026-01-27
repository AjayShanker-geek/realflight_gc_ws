#!/usr/bin/env bash
set -euo pipefail

# TMUX session name
SESSION=ground_control_station

# -----------------------------
# Kill existing tmux session
# -----------------------------
# tmux has-session -t $SESSION 2>/dev/null
# if [ $? -eq 0 ]; then
# 	echo "Existing tmux session '$SESSION' found. Killing it..."
# 	tmux kill-session -t $SESSION
# 	sleep 1
# fi

# ============================================================================
# ROS2 Environment Setup Command
# ============================================================================
# This command will be executed at the start of each screen session to
# properly configure the ROS2 environment
WORKSPACE_DIR="$HOME/realflight_gc_ws"
ROS2_SETUP_CMD="source /opt/ros/humble/setup.bash && source $WORKSPACE_DIR/install/setup.bash"

# ROS2 topics
# TODO: Cleanup
STATE_CMD_TOPIC_DRONE_0="/state/command_drone_0"
STATE_STATE_TOPIC_DRONE_0="/state/state_drone_0"
TRAJ_TOPIC_DRONE_0="/fmu/in/trajectory_setpoint"
LOCAL_POS_TOPIC_DRONE_0="/fmu/out/vehicle_local_position"

STATE_CMD_TOPIC_DRONE_1="/state/command_drone_1"
STATE_STATE_TOPIC_DRONE_1="/state/state_drone_1"
TRAJ_TOPIC_DRONE_1="/px4_1/fmu/in/trajectory_setpoint"
LOCAL_POS_TOPIC_DRONE_1="/px4_1/fmu/out/vehicle_local_position"

STATE_CMD_TOPIC_DRONE_2="/state/command_drone_2"
STATE_STATE_TOPIC_DRONE_2="/state/state_drone_2"
TRAJ_TOPIC_DRONE_2="/px4_2/fmu/in/trajectory_setpoint"
LOCAL_POS_TOPIC_DRONE_2="/px4_2/fmu/out/vehicle_local_position"

BAG_TOPICS=(
    "${STATE_CMD_TOPIC_DRONE_0}"
    "${STATE_STATE_TOPIC_DRONE_0}"
    "${TRAJ_TOPIC_DRONE_0}"
    "${LOCAL_POS_TOPIC_DRONE_0}"
    "${STATE_CMD_TOPIC_DRONE_1}"
    "${STATE_STATE_TOPIC_DRONE_1}"
    "${TRAJ_TOPIC_DRONE_1}"
    "${LOCAL_POS_TOPIC_DRONE_1}"
    "${STATE_CMD_TOPIC_DRONE_2}"
    "${STATE_STATE_TOPIC_DRONE_2}"
    "${TRAJ_TOPIC_DRONE_2}"
    "${LOCAL_POS_TOPIC_DRONE_2}"
    "/vrpn_mocap/multilift_payload/pose"
    "/vrpn_mocap/multilift_payload/twist"
    "/vrpn_mocap/multilift_payload/accel"
)
BAG_TOPICS_STR="${BAG_TOPICS[*]}"

# ============================================================================
# Environment Variable Validation
# ============================================================================

echo "======================================"
echo "Sync with remote time server"
echo "======================================"

export TIMESYNC_IP="192.168.1.3"
if command -v ntpdate >/dev/null 2>&1; then
    sudo ntpdate -u "$TIMESYNC_IP" && echo "time already sync with $TIMESYNC_IP."
fi

echo "======================================"
echo "Checking Environment Variables..."
echo "======================================"

if [ -z "$VICON_IP" ]; then
    echo "ERROR: VICON_IP environment variable is not set"
    echo "Please set it with: export VICON_IP=<vicon_server_ip>"
    exit 1
fi
echo "VICON_IP: $VICON_IP"
echo "======================================"
echo ""


# TODO: Migration to Ground Computer
# ============================================================================
# Module 4: Vicon to PX4 Bridge
# ============================================================================
# Converts Vicon pose data to PX4-compatible format and publishes
# - Reads: Vicon pose from vrpn_mocap
# - Publishes: Vision-based position estimate to PX4
# - Handles: Frame transformations (ENU/NED/FLU conversions)
# echo ""
# echo "======================================"
# echo "Starting Vicon-PX4 Bridge..."
# echo "======================================"
# VICON_BRIDGE_CMD="ros2 launch vicon_px4_bridge vicon_px4_bridge.launch.py

# RECORD: rosbag
BAG_DIR="$WORKSPACE_DIR/rosbags"
mkdir -p "$BAG_DIR"
BAG_PATH="${BAG_DIR}/gc_$(date +%Y%m%d_%H%M%S)"

# TMUX Setup
tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s $SESSION
tmux split-window -h -t $SESSION
tmux split-window -v -t ${SESSION}:1

# Pane 0
# ============================================================================
# VRPN/Vicon Motion Capture Client
# ============================================================================
# Connects to Vicon motion capture system and publishes pose data to ROS2
# - Server: VICON_IP (Vicon Tracker server address)
# - Port: 3883 (VRPN default port)
# - Publishes: Transform data for tracked objects
VICON_CLIENT_CMD="ros2 launch vrpn_mocap client.launch.yaml server:=$VICON_IP port:=3883"
tmux send-keys -t 0 "cd $WORKSPACE_DIR" C-m
tmux send-keys -t 0 "$ROS2_SETUP_CMD" C-m
tmux send-keys -t 0 "$VICON_CLIENT_CMD" C-m


# ROSBAG
tmux send-keys -t 1 "cd $WORKSPACE_DIR" C-m
tmux send-keys -t 1 "$ROS2_SETUP_CMD" C-m
tmux send-keys -t 1 "echo \"Recording ROS 2 bag from GC\"" C-m
tmux send-keys -t 1 "echo \"Topics:\"; for t in ${BAG_TOPICS_STR}; do echo \"  - \$t\"; done" C-m
tmux send-keys -t 1 "echo \"Output: ${BAG_PATH}\"" C-m
tmux send-keys -t 1 "ros2 bag record -o ${BAG_PATH} ${BAG_TOPICS_STR}" C-m

tmux send-keys -t 2 "cd $WORKSPACE_DIR" C-m
tmux send-keys -t 2 "$ROS2_SETUP_CMD" C-m
tmux send-keys -t 2 "ros2 launch vicon_px4_bridge vicon_px4_bridge_gc.launch.py " C-m

tmux attach-session -t "$SESSION"
