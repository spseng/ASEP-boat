FROM ros:jazzy
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions build-essential gdb \
    ros-jazzy-foxglove-bridge socat \
    clangd jq \
    libeigen3-dev \
    libgeographiclib-dev \
 && rm -rf /var/lib/apt/lists/*
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN ln -s /ws/src/ASEP-boat/tools/cb /usr/local/bin/cb
RUN echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc
WORKDIR /ws