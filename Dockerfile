# Pinned by digest for reproducibility (ROS 2 Jazzy desktop, Ubuntu 24.04 Noble).
FROM osrf/ros@sha256:ffe0b5ea5736823fce29fea0b11f8d1637f1ceff611a3324bd147f669d93fc4f

# Gazebo Harmonic, ROS-GZ bridge, and cv_bridge.
# NOTE: the ROS apt repo only serves the latest snapshot (no historical
# version archive), so these cannot be pinned by version string here the
# way requirements.txt pins the Python layer below. The base image digest
# above is the reproducibility anchor for this layer.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-cv-bridge \
    python3-colcon-common-extensions \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Pinned Python dependencies (see requirements.txt) instead of the distro's
# python3-opencv package.
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed -r /tmp/requirements.txt

# Automatically source ROS 2 and Gazebo
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
