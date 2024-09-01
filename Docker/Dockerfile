ARG ROS_DISTRO=humble

# Use the official ROS image as the base image
FROM osrf/ros:${ROS_DISTRO}-desktop as base
ENV ROS_DISTRO=${ROS_DISTRO}

# Set shell for running commands
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install --no-install-recommends -y \
  build-essential \
  git \
  python3-colcon-common-extensions \
  python3-colcon-mixin \
  python3-rosdep \
  python3-vcstool  \
  ros-${ROS_DISTRO}-foxglove-bridge \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  ros-${ROS_DISTRO}-rosbag2-storage-mcap \
  && rm -rf /var/lib/apt/lists/*

# Copy the cyclonedds.xml configuration file
RUN echo '<?xml version="1.0" encoding="UTF-8" ?>\
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">\
  <Domain id="any">\
      <General>\
          <NetworkInterfaceAddress>lo</NetworkInterfaceAddress>\
          <AllowMulticast>false</AllowMulticast>\
      </General>\
      <Discovery>\
          <ParticipantIndex>auto</ParticipantIndex>\
          <Peers>\
              <Peer Address="localhost"/>\
          </Peers>\
          <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>\
      </Discovery>\
  </Domain>\
</CycloneDDS>' > /root/.ros/cyclonedds.xml

# Source the ROS environment setup script and set default entrypoint
RUN echo "source ros_entrypoint.sh" >> ~/.bashrc
CMD ["/bin/bash"]

COPY docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

#############################
# Overlay Image for slamvdb #
#############################
FROM base AS overlay

# Create an overlay Colcon workspace
RUN mkdir -p /colcon_ws/src
WORKDIR /colcon_ws
COPY slam_vdb_ros/ ./src/slam_vdb_ros/
# NOTE: uncomment this until we resolve build issues
# RUN rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
#     && colcon build --symlink-install

# Set up the entrypoint
COPY docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

#####################
# Development Image #
#####################
FROM overlay as dev

# Dev container arguments
ARG USERNAME=devuser
ARG UID=1000
ARG GID=${UID}

# Create new user and home directory
RUN groupadd --gid $GID $USERNAME \
 && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
 && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && mkdir -p /home/${USERNAME} \
 && chown -R ${UID}:${GID} /home/${USERNAME}

# Set the ownership of the overlay workspace to the new user
RUN chown -R ${UID}:${GID} /colcon_ws/

# Set the user and source entrypoint in the user's .bashrc file
USER ${USERNAME}
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc