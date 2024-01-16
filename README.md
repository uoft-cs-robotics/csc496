# Build

```bash
cd ~/<path_to_this_repo>
mkdir build
cd build
cmake ..
make
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
