# Digital Twin for Unitree Go1 Quadruped Robot using ROS and Isaac Sim

## Introduction

This repository contains the development of a Digital Twin for the Unitree Go1 quadruped robot. The project integrates ROS (Robot Operating System) and NVIDIA's Isaac Sim to achieve real-time synchronization between the physical robot and its virtual counterpart.

## Author

- **Adam Zahir Rodriguez**
- Email: adamzr2000@gmail.com

## Project Structure

- `go1-interface`: Contains ROS APIs in combination with the Unitree Legged SDK to communicate with the physical robot.
- `isaac-sim`: Contains setup instructions and scripts necessary for configuring Isaac Sim, along with a pre-configured virtual environment.
- `ros-master`: Features a lightweight Docker image designed to run the ROS MASTER, facilitating decentralized operation across multiple machines.
- `utils`: Includes utility scripts such as `iptables-wifi6.sh` for network configuration on the Raspberry Pi, enabling data transfer between the robot's internal network and the external WLAN.


## Getting Started

### Prerequisites

Ensure you have the following prerequisites installed and configured on your system:

- **Docker**: For installation instructions, refer to the [Docker Installation Guide](https://docs.docker.com/engine/install/ubuntu/).
- **Isaac Sim Host System Requirements**: To check if your system meets the requirements for running Isaac Sim, visit [Isaac Sim Requirements](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html).


### Setup

Follow these steps to set up the project on your local system:

1. **Clone the Repository**

```bash
git clone https://github.com/<your-github-username>/go1-docker-ros-digital-twin.git
cd go1-docker-ros-digital-twin
```

2. **Build Docker Images**

Use the following command to build the Docker images for all components (except `isaac-sim`):

```bash
./build.sh
```

**Note**: If your network setup differs, before building the `go1-interface`, you must replace the placeholder IP address in the `line 29` of `go1-interface/catkin_ws/src/unitree_ros_to_real/unitree_legged_real/scripts/twist_sub_wifi6.cpp` with your Raspberry Pi IP address (wireless interface):

```bash
Before: 
high_udp(8090, "10.5.98.70", 8082, sizeof(HighCmd), sizeof(HighState))

After: 
high_udp(8090, "<your-ip-address>", 8082, sizeof(HighCmd), sizeof(HighState))
```

For setting up `isaac-sim`, refer to the instructions provided in `isaac-sim/README.md`


3. **Running the Components**

To start a component, execute the `run_example.sh` script:

```bash
./run_example.sh
```