FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# set the locale
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

# ros
RUN apt-get update && apt-get install software-properties-common -y
RUN add-apt-repository universe
RUN apt-get update && apt-get install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get upgrade
RUN apt-get install ros-humble-desktop -y

# gedit
RUN apt-get update && apt-get install -y gedit

# user
RUN apt-get update && apt-get install -y sudo
RUN useradd -m -s /bin/bash csc496ta
RUN echo 'csc496ta:csc496ta' | chpasswd
RUN usermod -aG sudo csc496ta

# 3D slicer deps
RUN apt update && apt install -y git build-essential cmake cmake-curses-gui cmake-qt-gui libqt5x11extras5-dev qtmultimedia5-dev libqt5svg5-dev qtwebengine5-dev libqt5xmlpatterns5-dev qttools5-dev qtbase5-private-dev qtbase5-dev qt5-qmake
USER csc496ta

# 3D slicer
RUN mkdir /home/csc496ta/slicer
WORKDIR /home/csc496ta/slicer
RUN git clone https://github.com/Slicer/Slicer.git
RUN cd Slicer && git checkout v5.6.2
RUN mkdir Slicer-SuperBuild-Debug
WORKDIR /home/csc496ta/slicer/Slicer-SuperBuild-Debug
RUN cmake -DSlicer_USE_SYSTEM_OpenSSL=ON -DSlicer_BUILD_TESTING=OFF -DSlicer_DOWNLOAD_TEST_DATA=OFF -DSlicer_USE_TESTING_DATA=OFF -DBUILD_TESTING=OFF ../Slicer && make -j14

# use bash
SHELL ["/bin/bash", "-c"]

# slicer ros2
USER root
RUN apt-get update && apt-get install -y python3-colcon-common-extensions ros-humble-ament-cmake ros-humble-joint-state-publisher ros-humble-xacro
USER csc496ta
WORKDIR /home/csc496ta
RUN mkdir -p /home/csc496ta/ros2_ws/src
RUN cd /home/csc496ta/ros2_ws/src && git clone https://github.com/rosmed/slicer_ros2_module
WORKDIR /home/csc496ta/ros2_ws
RUN source /opt/ros/humble/setup.bash && colcon build --cmake-args -DSlicer_DIR:PATH=/home/csc496ta/slicer/Slicer-SuperBuild-Debug/Slicer-build -DCMAKE_BUILD_TYPE=Release

# franka_bringup && franka_description
WORKDIR /home/csc496ta/ros2_ws/src
RUN git clone https://mcsgitlab.utm.utoronto.ca/iseoluwa/franka_bringup.git
RUN git clone https://mcsgitlab.utm.utoronto.ca/iseoluwa/franka_description.git
WORKDIR /home/csc496ta/ros2_ws
RUN source /opt/ros/humble/setup.bash && colcon build

# aliases
RUN echo 'alias ll="ls -la"' >> ~/.bashrc
RUN echo 'alias s_ros="source /opt/ros/humble/setup.bash && source /home/csc496ta/ros2_ws/install/setup.bash"' >> ~/.bashrc
RUN echo 'alias slicer="s_ros && /home/csc496ta/slicer/Slicer-SuperBuild-Debug/Slicer-build/Slicer"' >> ~/.bashrc

# terminal
USER root
RUN apt-get update && apt-get install -y tilix

USER csc496ta
WORKDIR /home/csc496ta

# docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY <image_name>
ENV DISPLAY=:0
ENTRYPOINT ["/bin/bash"]
