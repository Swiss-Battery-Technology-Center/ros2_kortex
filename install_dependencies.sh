sudo apt install python3-colcon-common-extensions python3-vcstool

# Ros2Control, Moveit2, etc is already installed, so we need only this
vcs import . --skip-existing --input ros2_kortex-not-released.$ROS_DISTRO.repos 

# Make sure to install the dependencies
rosdep install --ignore-src --from-paths . -y -r

