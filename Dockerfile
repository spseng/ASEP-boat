FROM ros:jazzy
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions build-essential gdb \
    ros-jazzy-foxglove-bridge socat \
 && rm -rf /var/lib/apt/lists/*
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
WORKDIR /ws