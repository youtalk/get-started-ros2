# hadolint global ignore=DL3006,DL3008,DL3013
FROM ros:jazzy
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-recommends \
  ros-dev-tools \
  && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* "$HOME"/.cache

COPY src /get-started-ros2/src
WORKDIR /get-started-ros2
RUN ls /get-started-ros2/src

RUN rosdep update && rosdep install --ignore-src --from-paths src

RUN source /opt/ros/jazzy/setup.bash \
  && colcon build \
  --mixin release compile-commands
