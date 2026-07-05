#!/bin/bash

source /opt/ros/noetic/setup.bash

if [ -f ~/catkin_ws/devel/setup.bash ]; then
    source ~/catkin_ws/devel/setup.bash
fi

ROBOT_IP=192.168.0.121

export ROS_MASTER_URI=http://$ROBOT_IP:11311
export ROS_IP=$(ip route get $ROBOT_IP | awk '{for(i=1;i<=NF;i++) if ($i=="src") {print $(i+1); exit}}')

echo "Using ROS_MASTER_URI=$ROS_MASTER_URI"
echo "Using ROS_IP=$ROS_IP"

cd ~/lab_data_recorder || exit 1

echo "Starting recorder script..."
python3 ulrecobuv3 &
RECORDER_PID=$!

echo "Starting teleoperation script..."
python3 ps4_base_arm_fk_ik_toggle.py &
TELEOP_PID=$!

echo "Recorder PID: $RECORDER_PID"
echo "Teleop PID: $TELEOP_PID"
echo "Both scripts are running. Press Ctrl+C to stop both."

cleanup() {
    echo ""
    echo "Stopping recorder and teleoperation..."

    kill -INT $RECORDER_PID 2>/dev/null
    kill -INT $TELEOP_PID 2>/dev/null

    wait $RECORDER_PID 2>/dev/null
    wait $TELEOP_PID 2>/dev/null

    echo "Stopped."
    exit 0
}

trap cleanup SIGINT SIGTERM

wait
