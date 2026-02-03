FROM ros:jazzy-ros-base

# Set working directory
WORKDIR /app

# Install Python dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Install Python requirements
COPY backend/backend/requirements.txt .
RUN pip3 install --no-cache-dir --break-system-packages -r requirements.txt

# Copy application code
COPY backend/backend/. .

# Expose port
EXPOSE 8000

ENV ROS_DISTRO=jazzy

# Start command that sources ROS and starts FastAPI
CMD ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && cd /app && python3 setup.py && uvicorn main:app --host 0.0.0.0 --port 8000"]