# hadolint global ignore=DL3006,DL3008,DL3013
ARG BASE_IMAGE

FROM $BASE_IMAGE
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO

COPY src /get-started-ros2/src
WORKDIR /get-started-ros2

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-recommends \
  ros-dev-tools \
  python3-rosdep \
  && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* "$HOME"/.cache

RUN rosdep update && rosdep install --ignore-src --from-paths src

RUN source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && colcon build --cmake-args \
  --mixin release compile-commands
