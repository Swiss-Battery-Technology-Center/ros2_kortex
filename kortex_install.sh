#!/bin/bash
echo "Updating package list..."
sudo apt update
echo "Installing Kortex Bringup package..."
sudo apt install -y ros-$ROS_DISTRO-kortex-bringup
echo "Installing Kinova Gen3 MoveIt config..."
sudo apt install -y ros-$ROS_DISTRO-kinova-gen3-7dof-robotiq-2f-85-moveit-config
echo "Installing dependencies with rosdep..."
rosdep install --ignore-src --from-paths src -y -r
echo "Building the workspace with colcon..."
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
BASHRC_PATH="$HOME/.bashrc"
WORKSPACE_SETUP="source ~/workspace/ros2_kortex_ws/install/setup.bash"
if ! grep -Fxq "$WORKSPACE_SETUP" $BASHRC_PATH; then
    echo "Adding workspace setup to bashrc..."
    echo "$WORKSPACE_SETUP" >> $BASHRC_PATH
else
    echo "Workspace setup already exists in bashrc."
fi
echo "Setup complete. Please restart your terminal or run 'source ~/.bashrc' to apply changes."