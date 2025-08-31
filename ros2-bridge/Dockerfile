FROM ros:humble-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    ros-humble-rclpy \
    ros-humble-std-msgs \
  && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY requirements.txt ./
RUN pip3 install --no-cache-dir -r requirements.txt
COPY app.py ./
ENV PYTHONUNBUFFERED=1 \
    ROS_DOMAIN_ID=42
EXPOSE 8081
CMD ["python3", "app.py"]
