
# Build

```bash
# build slicer and ros2
docker build -f Dockerfile_csc379 . -t slicerros_csc379 --no-cache
# Later this should seperate ros and slicer

# build franka libs
docker build -f Dockerfile_csc379_franka . -t slicerros_csc379_franka --no-cache
```

# Run

```bash
docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY slicerros_csc379_franka
```

# Deploy

To save the docker image to a file:
```bash
$ docker save slicerros_csc379_franka | gzip > slicerros_csc379_franka.tar.gz
```

To load the docker image on the computer:
```bash
$ docker load < slicerros_csc379_franka.tar.gz
```

# for myself
```bash
docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/teachinglab_student/csc496:/home/csc496:rw -e DISPLAY=$DISPLAY osrf/ros:humble-desktop 
```
