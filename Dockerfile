ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

# Set shell and working directory
WORKDIR /root/ros2_ws
SHELL ["/bin/bash", "-c"]

# Create the directory structure for packages
WORKDIR /root/ros2_ws/src
RUN mkdir -p yolo_bringup yolo_msgs yolo_ros

# Copy files required for dependencies first to leverage Docker cache
COPY requirements.txt .
COPY yolo_bringup/package.xml yolo_bringup/
COPY yolo_msgs/package.xml yolo_msgs/
COPY yolo_ros/package.xml yolo_ros/

# Go back to the workspace root
WORKDIR /root/ros2_ws

# --- CORRECTED DEPENDENCY INSTALLATION BLOCK ---
# Combine all apt operations into a single RUN layer
RUN apt-get update && apt-get -y --quiet --no-install-recommends install \
    python3 \
    python3-pip \
    # Now run rosdep while the apt lists are still available
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    # Clean up apt lists at the very end
    && rm -rf /var/lib/apt/lists/*

# The conditional pip install logic is preserved
RUN if [ "$(lsb_release -rs)" = "24.04" ] || \
   [ "$(lsb_release -rs)" = "24.10" ]; then \
    pip3 install -r src/requirements.txt --break-system-packages --ignore-installed; \
    else \
    pip3 install -r src/requirements.txt; \
    fi

# Now, copy the rest of your source code.
# Only this layer and the build layer below will be invalidated on code changes.
COPY . /root/ros2_ws/src

# Build the workspace with colcon
ARG CMAKE_BUILD_TYPE=Release
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build

# Source the ROS 2 setup file and set the default command
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc
CMD ["bash"]