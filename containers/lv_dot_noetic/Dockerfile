FROM ros:noetic-ros-base-focal

ARG LV_DOT_REPOSITORY=https://github.com/Zhefan-Xu/LV-DOT.git
ARG LV_DOT_COMMIT=449bf2c960a26b067b235d82f6e0aac65fc05a6b

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get -o Acquire::Retries=5 update \
    && apt-get -o Acquire::Retries=5 install -y --no-install-recommends \
    build-essential \
    git \
    libpcl-dev \
    python3-catkin-tools \
    python3-opencv \
    python3-pip \
    ros-noetic-cv-bridge \
    ros-noetic-gazebo-msgs \
    ros-noetic-image-transport \
    ros-noetic-message-filters \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /opt/lv_dot_ws/src \
    && git clone "${LV_DOT_REPOSITORY}" /opt/lv_dot_ws/src/LV-DOT \
    && cd /opt/lv_dot_ws/src/LV-DOT \
    && git checkout "${LV_DOT_COMMIT}"

RUN /bin/bash -lc \
    'source /opt/ros/noetic/setup.bash && cd /opt/lv_dot_ws && catkin build'

COPY detector_param.yaml /opt/uav_usv/lv_dot/detector_param.yaml
COPY run_lv_dot.launch /opt/uav_usv/lv_dot/run_lv_dot.launch
COPY ros2_ingress.py /opt/uav_usv/lv_dot/ros2_ingress.py
COPY ros2_egress.py /opt/uav_usv/lv_dot/ros2_egress.py
COPY entrypoint.sh /opt/uav_usv/lv_dot/entrypoint.sh
RUN chmod +x \
    /opt/uav_usv/lv_dot/entrypoint.sh \
    /opt/uav_usv/lv_dot/ros2_ingress.py \
    /opt/uav_usv/lv_dot/ros2_egress.py

ENTRYPOINT ["/opt/uav_usv/lv_dot/entrypoint.sh"]
CMD ["roslaunch", "/opt/uav_usv/lv_dot/run_lv_dot.launch"]
