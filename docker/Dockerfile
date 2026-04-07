FROM osrf/ros:humble-desktop

LABEL maintainer="Swarmap Team"
LABEL description="Swarmap multi-robot mapping stack — ROS2 Humble + Gazebo + rosbridge"

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=0

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-rosbridge-suite \
    ros-humble-nav2-msgs \
    ros-humble-tf2-tools \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    curl \
    wget \
    git \
    && rm -rf /var/lib/apt/lists/*

# Node.js 20 for dashboard
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y nodejs \
    && rm -rf /var/lib/apt/lists/*

# Copy workspace
WORKDIR /swarmap_ws
COPY . .

# Install ROS dependencies
RUN . /opt/ros/humble/setup.sh \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y

# Install dashboard npm packages
RUN if [ -f src/swarmap_dashboard/package.json ]; then \
        npm --prefix src/swarmap_dashboard install; \
    fi

# Build ROS workspace
RUN . /opt/ros/humble/setup.sh \
    && colcon build --symlink-install

# Source workspace on every shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /swarmap_ws/install/setup.bash" >> /root/.bashrc

EXPOSE 9090

CMD ["bash", "-c", \
    "source /opt/ros/humble/setup.bash && \
     source /swarmap_ws/install/setup.bash && \
     ros2 launch swarmap_bringup simulation.launch.py"]
