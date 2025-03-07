FROM osrf/ros:humble-simulation-jammy

RUN apt update && apt install -y --no-install-recommends \
    openssh-server \
    xauth \
    snapd \
    python3-colcon-common-extensions \
    python3-pip \
    wget

RUN echo root:robotics | chpasswd

RUN echo "PermitRootLogin yes" >> /etc/ssh/sshd_config && \
    echo "X11UseLocalhost no" >> /etc/ssh/sshd_config && \
    echo "PermitUserEnvironment yes" >> /etc/ssh/sshd_config && \
    mkdir -p /root/.ssh && \
    touch /root/.ssh/environment

#RUN sudo mkdir -p /etc/apt/keyrings && \
#    sudo curl -s https://cyberbotics.com/Cyberbotics.asc > /etc/apt/keyrings/Cyberbotics.asc && \
#    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list && \
#    sudo apt-get update && \
#    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y webots

RUN sudo python3 -m pip install tornado simplejpeg

RUN sudo apt install -y libpcl-dev

RUN echo "Port 2201" >> /etc/ssh/sshd_config

RUN mkdir -p /home/root

RUN git config --global --add safe.directory /home/root

COPY ssh_entry.sh /root/.bash_profile
COPY ros_bashrc.sh /root/ros_bashrc.sh
RUN echo "source /root/ros_bashrc.sh" >> /root/.bashrc

ENTRYPOINT service ssh restart && bash

