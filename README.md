# csc379 Meta Repo for Assignments and Practicals


# Setup (TA's only)

Cloning the Repo:
```bash
git clone https://github.com/uoft-cs-robotics/csc496.git
cd csc496
git checkout feature/winter2025
git submodule update --init --recursive
```


# To install dependancies and build workspace

Inside the docker terminal 
```bash
apt install python3-rosdep #installs rosdep 
rosdep init 
rosdep update 
apt update 

cd /home/csc379/scratchpad/franka_ws 
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build

source /home/csc379/scratchpad/franka_ws/install/local_setup.bash

```

Note: git submodule update requires our gitlab http userid (utorid), and access token as password. Create an access token by clicking the user icon -> Preferences. Then click Access tokens. 

# Docker build

Follow instructions to build docker files: 
[docker/csc379_README.md](docker/csc379_README.md).
