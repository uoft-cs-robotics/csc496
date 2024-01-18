# Build

```bash
cd ~/<path_to_this_repo>
mkdir build
cd build
cmake ..
make
```

# Turning on Franka Robot

1. Pull "red" colored power switch.
2. Wait for status LED to turn solid yellow.
3. Open chrome browser and type 192.168.1.107 to access Franka Desk.
4. Unlock the joints by clicking unlock icon.
5. Go to the hamburger menu and click Active FCI. Let it remain active.

# Turning off Franka Robot

1. Open Franka Desk.
2. Deactivate FCI.
3. Go to the hamburger menu and select shutdown robot.
4. Once Franka Desk says it is okay to power off, then press the red power switch.

# Starting Docker 

Every micro PC computer in the lab has a desktop folder called **CSC496**. In that folder there are four scripts.

``` bash
# Builds docker containers (already done for you)
./build_docker_container.sh 

# Starts the docker containers (run once to get started)
./start_docker_container.sh 

# Run in every terminal in host machine where you want to access container's terminal
./open_docker_terminal.sh

# Run to close the container once you are done with all your work
./stop_docker_containers.sh
```

# Running Libfranka Examples

Once you have started the docker container and are in the docker terminal, you can access the libfranka source code and compiled files in the following folders:

``` bash
# Source code can be found in the examples folder.
/root/git/scratchpad/libfranka/examples

# Build directory contains examples folder where all the compiled examples are ready to run
/root/git/scratchpad/libfranka/build/examples
```

If you make any changes in the libfranka cpp files, you will then need to build the files again.

Note that in order to run any of the examples, you need to include the ip address of the Franka:

``` bash
# Example
./communication_test 192.168.1.107
```

# Run Teleop

Make sure two franka controller pcs are connected via ethernet cable directly, and set the static ip
in the same subnet (x's): xxx.xxx.x.yyy. On the teaching lab computer, there should be in settings->network
an ip address config: test_teleop or similar name, which should set the IP of the computer correctly. If not
ask the TA for correct IP address.

On one computer with ip: \<hal\>, start the follower:
```bash
cd build
./run_follower_franka <port> <robot_ip> <follower_options>
# example
./run_follower_franka 2233 192.168.1.107 1
```

follower_options:
1. no motion scaling
2. scaled motion

Note on the teaching lab computer, static ip should be \<hal\> = 192.168.0.1

On another computer, start the leader:
```bash
cd build
./run_follower_franka <port> <hal> <robot_ip> <leader_options>
# example
./run_leader_franka 2233 <hal> 192.168.1.107 1
```

leader_options:
1. no feedback from follower
2. position feedback from follower
3. ext torque feedback from follower

# Run other examples

TODO
