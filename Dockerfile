# hadolint global ignore=DL3006,DL3008,DL3013
FROM ros:jazzy
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update \
  && DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-recommends \
  ros-dev-tools \
  && rm -rf /var/lib/apt/lists/*

COPY src /get-started-ros2/src
WORKDIR /get-started-ros2

RUN apt-get update \
  && rosdep update \
  && rosdep install --ignore-src --from-paths src \
  && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/jazzy/setup.bash \
  && colcon build \
  --mixin release compile-commands
