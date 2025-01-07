# Docker

# Install 

Use docker-compose standalone, not docker desktop

# Build (TA's only)

These should already be built for you. check with `docker images`

```bash
# build slicer and ros2
docker build -f Dockerfile_csc379 . -t slicerros_csc379 --no-cache

# build the rest of packages: libfranka, franka_descriptions, robotics_toolbox, slicer_ros2_modules
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
docker save -o docker-slicerros_csc379_franka.tar docker-slicerros_csc379_franka

```

To load the docker image on the computer:
```bash
docker load -i docker-slicerros_csc379_franka.tar
```

To save the docker image to a file:
```bash
docker save -o slicerros_csc379.tar slicerros_csc379
```

To load the docker image on the computer:
```bash
docker load -i slicerros_csc379.tar
```
