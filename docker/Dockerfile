ARG image=osrf/ros:noetic-desktop-full
FROM ${image}

ENV WS_DIR="/root/git"
WORKDIR ${WS_DIR}

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get upgrade -y  && apt-get -y install sudo
RUN apt-get install -y --no-install-recommends libncurses5-dev sudo && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*


RUN echo "cloning and building libfranka"

RUN apt-get update && apt-get install -y \
    build-essential cmake git libpoco-dev libeigen3-dev

RUN cd /root/git  \   
    && git clone --recursive https://github.com/frankaemika/libfranka \
    && cd libfranka \
    && mkdir build \ 
    && cd build \ 
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j4 \
    && make install 

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "export FRANKA_IP=192.168.1.107" >> ~/.bashrc
RUN echo "export FOLLOWER_IP=192.168.0.1" >> ~/.bashrc
RUN echo "export LEADER_IP=192.168.0.3" >> ~/.bashrc

ARG DEBIAN_FRONTEND=dialog


