#!/bin/bash
if [ -z "$ROBOT_ID" ]; then
    echo "Error: ROBOT_ID is not set."
    exit 1
fi

USER_NAME=$USER
WORK_DIR="/home/$USER_NAME/ros2_ws"
SCRIPT_PATH="$WORK_DIR/pioneer_ws/scripts/start_ros2.sh"

SERVICE_FILE="/etc/systemd/system/ros2_start.service"

cat <<EOF | sudo tee $SERVICE_FILE > /dev/null
[Unit]
Description=ROS 2 Auto Start
After=network-online.target
Wants=network-online.target

[Service]
Environment="ROBOT_ID=$ROBOT_ID"
User=$USER_NAME
WorkingDirectory=$WORK_DIR
ExecStart=/bin/bash $SCRIPT_PATH
Restart=always
Environment="DISPLAY=:0"
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

echo "Service file created at: $SERVICE_FILE"

cd $WORK_DIR
rm -rf pioneer_base sim
colcon build

sudo systemctl enable ros2_start.service
sudo systemctl start ros2_start.service
