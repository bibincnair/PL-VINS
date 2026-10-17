FROM ros:jazzy-ros-base-noble
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git ca-certificates pkg-config python3-dev python3-numpy \
    python3-venv python3-pip python3-colcon-common-extensions python3-rosdep \
    libblas-dev liblapack-dev libsuitesparse-dev libtbb-dev libmetis-dev \
    libjpeg-dev libpng-dev libtiff-dev libopenexr-dev zlib1g-dev libgtk-3-dev \
    ros-jazzy-ament-cmake-gtest ros-jazzy-message-filters ros-jazzy-tf2-ros \
    ros-jazzy-rosbag2-storage-mcap ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace/PL-VINS
COPY . .
RUN bash tools/build.sh
RUN python3 -m venv --system-site-packages .venv && \
    .venv/bin/pip install --require-hashes -r requirements.lock
CMD ["bash"]
