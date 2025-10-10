# ESP32 CSI Flash Tool - Docker Container
FROM espressif/idf:v5.5.1

# Set working directory
WORKDIR /workspace

# Update package list and install pip
RUN apt-get update && \
    apt-get install -y python3-pip && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install additional Python packages for our CLI tool
RUN pip3 install click pyserial pyyaml rich --break-system-packages

# Copy the ESP32 projects
COPY esp/ /workspace/esp/

# Copy our Python CLI tool and configuration templates
COPY docker/ /workspace/docker/

# Set up environment
ENV PYTHONPATH="/workspace/docker:$PYTHONPATH"
ENV IDF_PATH="/opt/esp/idf"

# Make entrypoint executable
RUN chmod +x /workspace/docker/flash_entrypoint.sh

# Set the entrypoint to simple bash script for direct flashing
ENTRYPOINT ["/workspace/docker/flash_entrypoint.sh"]