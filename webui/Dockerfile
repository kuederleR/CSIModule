FROM ros:humble

WORKDIR /app

# Install Python and dependencies
# ROS Humble already has rclpy, we just need to ensure it's available
RUN apt-get update && \
    apt-get install -y \
        python3-pip \
        ros-humble-rclpy \
        git && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install wifi_msgs ROS2 package
RUN mkdir -p /root/ros2_ws/src && \
    cd /root/ros2_ws/src && \
    git clone https://github.com/kuederleR/ros2-csi-msgs.git && \
    cd /root/ros2_ws && \
    . /opt/ros/humble/setup.sh && \
    colcon build --packages-select wifi_msgs && \
    rm -rf build log

# Copy requirements (without rclpy) and install
COPY requirements.txt .
RUN grep -v "^rclpy" requirements.txt > requirements_filtered.txt && \
    pip3 install --no-cache-dir -r requirements_filtered.txt

# Copy application
COPY app.py .
COPY templates/ templates/

# Create logs directory
RUN mkdir -p /app/logs

# Expose port
# EXPOSE 5000

# Source ROS2, wifi_msgs workspace, and run the application
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && python3 app.py"]
