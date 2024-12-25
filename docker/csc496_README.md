
# Build

```bash
# build slicer and ros2
docker build -f Dockerfile_csc496 . -t slicerros_csc496 --no-cache
# Later this should seperate ros and slicer

# build franka libs
docker build -f Dockerfile_csc496.franka . -t slicerros_csc496_franka --no-cache
```

# Run

```bash
docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY slicerros_csc496_franka
```

# Deploy

To save the docker image to a file:
```bash
$ docker save slicerros_csc496_franka | gzip > slicerros_csc496_franka.tar.gz
```

To load the docker image on the computer:
```bash
$ docker load < slicerros_csc496_franka.tar.gz
```
