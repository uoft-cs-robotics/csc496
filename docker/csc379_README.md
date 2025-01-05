# Docker

# Install 

Use docker-compose standalone, not docker desktop

# Build (TA's only)

These should already be built for you. check with `docker images`

```bash
# build slicer and ros2
docker build -f Dockerfile_csc379 . -t slicerros_csc379 --no-cache
# Later this should seperate ros and slicer

# build the rest of packages: libfranka, franka_descriptions, robotics_toolbox
# image name:docker-slicerros_csc379_franka
./build_docker_container.sh
```

# Run

```bash
# starts the docker container docker-slicerros_csc379_franka on the system
./start_docker_container.sh 

# Get into the terminal
./open_docker_container.sh

./stop_docker_container.sh
```

# Deploy (TA's only)

To save the docker image to a file:
```bash
$ docker save docker-slicerros_csc379_franka | gzip > docker-slicerros_csc379_franka.tar.gz
```

To load the docker image on the computer:
```bash
$ docker load < docker-slicerros_csc379_franka.tar.gz
```

# for myself, radian
```bash
docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/radian/csc496:/home/csc496:rw -e DISPLAY=$DISPLAY osrf/ros:humble-desktop 
```
