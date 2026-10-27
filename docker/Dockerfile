FROM ros:humble-ros-base-jammy@sha256:566b87e6dfc48392189a9c32e594e84060fa9948de93b04e536f39475a1f6963

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    LANG=C.UTF-8

RUN printf 'Acquire::Retries "12";\nAcquire::http::Timeout "60";\nAcquire::https::Timeout "60";\n' > /etc/apt/apt.conf.d/80-retries

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        ffmpeg \
        python3-colcon-common-extensions \
        python3-pip \
        python3-numpy \
        python3-opencv \
        python3-rosdep \
        ros-humble-xacro \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-cv-bridge \
        ros-humble-vision-msgs \
        ros-humble-image-transport \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-tf2-ros \
        ros-humble-tf2-geometry-msgs \
        libgl1-mesa-dri \
        libgl1 \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-ros2-control \
        ros-humble-gazebo-dev \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-controller-manager \
        libgazebo-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-moveit-ros-move-group \
        ros-humble-moveit-planners-ompl \
        ros-humble-moveit-kinematics \
        ros-humble-moveit-simple-controller-manager \
        ros-humble-moveit-ros-planning \
        ros-humble-moveit-ros-perception \
        ros-humble-moveit-servo \
        ros-humble-navigation2 \
        ros-humble-nav2-smac-planner \
        ros-humble-nav2-mppi-controller \
        ros-humble-rviz2 \
        ros-humble-rqt-graph \
        graphviz xvfb xauth \
    && rm -rf /var/lib/apt/lists/*

# CPU wheels keep the image small. yolov8n is fast enough for one camera.
RUN pip3 install --no-cache-dir --index-url https://download.pytorch.org/whl/cpu \
        torch torchvision \
    && pip3 install --no-cache-dir "packaging>=24" \
    && pip3 install --no-cache-dir --no-deps ultralytics \
    && pip3 install --no-cache-dir "numpy>=1.23,<2" \
        matplotlib pandas pyyaml requests scipy tqdm pillow psutil ultralytics-thop

COPY docker/requirements-mujoco.txt /tmp/requirements-mujoco.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements-mujoco.txt

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
