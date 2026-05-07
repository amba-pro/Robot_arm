FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    python3 \
    python3-pip \
    python3-serial \
    python3-tk \
    openssh-server \
    git \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && add-apt-repository universe -y \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
      > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y \
      ros-humble-ros-base \
      ros-humble-rviz2 \
      ros-humble-rqt \
      ros-humble-rqt-common-plugins \
    && python3 -m pip install --upgrade pip pyserial customtkinter \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /var/run/sshd
RUN sed -i 's/^#\?PermitRootLogin .*/PermitRootLogin no/' /etc/ssh/sshd_config && \
    sed -i 's/^#\?PasswordAuthentication .*/PasswordAuthentication yes/' /etc/ssh/sshd_config

WORKDIR /workspace
COPY . /workspace

RUN chmod +x /workspace/scripts/install_dependencies.sh /workspace/scripts/setup_ssh.sh || true

RUN bash -lc "source /opt/ros/humble/setup.bash && cd /workspace/ros_ws && colcon build"

EXPOSE 22 50000/udp 50001/tcp

CMD ["/bin/bash", "-lc", "service ssh start && source /opt/ros/humble/setup.bash && echo 'Container ready. Use ROS launch from /workspace/ros_ws' && tail -f /dev/null"]
