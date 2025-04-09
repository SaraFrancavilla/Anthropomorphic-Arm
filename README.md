## Features
- **Simulation visualization**: Tools to interact with the UR5 simulation in a graphical environment.
- **Example ROS 2 nodes**: Includes a sample node for trajectory publication to get you started.
- **Launch files**: Ready-to-use launch files for interacting with the simulated or real UR5 robot.
- **Bash scripts**: Simplifies running the UR5 simulation and ROS 2 environments with pre-defined commands.

---

## Repository Structure
```plaintext
.
├── launch/
│   ├── sim.launch.py                     # Launch file to simulate the UR5 robot in gazebo
│   ├── interface.launch.py               # Launch file to interact with simulated and real UR5 robot
├── src/
│   ├── gripper_service.cpp               # Node that provides a service to open/close the gripper
│   ├── publish_trajectory_node.cpp       # Example node for trajectory publication
├── config/
│   ├── ur_controllers.yaml               # Configuration file for the UR5 controllers
├── params/
│   ├── ur5_bridge.yaml                   # Parameters file for the Gazebo bridge
├── gripper/
│   ├── open.script                       # Script to open the soft robotics gripper
│   ├── close.script                      # Script to close the soft robotics gripper
│   ├── neutral_from_open.script          # Script to move the gripper to a neutral position from the open position
│   ├── neutral_from_closed.script        # Script to move the gripper to a neutral position from the closed position
├── models/
│   ├── desk.urdf.xacro                   # URDF file that defines the desk where the UR5 is mounted
│   ├── desk.config                       # Config file auxiliary to the SDF file
│   ├── ur_gz.ros2_control.xacro          # URDF file that defines the UR5 robot with ROS 2 control
│   ├── ur_gz.urdf.xacro                  # URDF file that defines the UR5 robot
│   ├── soft_robotics_gripper.urdf.xacro  # URDF file that defines the soft robotics gripper
│   ├── camera.sdf                        # SDF file that defines the RGBD camera
│   ├── block.urdf.xacro                  # URDF file that defines a block
│   ├── block.config                      # Config file auxiliary to the SDF file
│   ├── meshes
│   |   ├── desk.stl                      # STL file with the 3D mesh of the desk
│   |   ├── base.stl                      # STL file with the 3D mesh of the base of the soft robotics gripper
│   |   ├── link1.stl                     # STL file with the 3D mesh of the first link of the soft robotics gripper
│   |   ├── link2.stl                     # STL file with the 3D mesh of the second link of the soft robotics gripper
│   |   ├── X*-Y*-Z*.stl                  # STL files with the 3D mesh of the blocks
├── docker/
│   ├── Dockerfile                        # Dockerfile for the ROS 2 UR5 interface docker image
│   ├── entrypoint.sh                     # Entrypoint script used in the docker
├── scripts/
│   ├── ur5.sh                            # Starts the URSim simulator container
│   ├── ros2.sh                           # Starts the pla10/ros2_ur5_interface container
├── worlds/
│   ├── empty.world                       # Gazebo world file with plugins for the UR5 simulation
├── rviz/
│   ├── ur5.rviz                          # RViz configuration file
├── README.md                             # Project overview and instructions
```

---

## Prerequisites
- **Docker**: Ensure Docker is installed and running on your system.
- **Docker Network Setup**: Create a Docker network named `ursim_net` before running the containers:
  ```bash
  docker network create --subnet=192.168.56.0/24 ursim_net
  ```

## Simulation
You can choose between two simulation environments:
- **URSim**: A graphical simulation environment for the UR5 robot that uses a dedicated Docker container. This solution is preferred since it is lightweight and simulates the real connection with the robot. But with this solution you can simulate ONLY the robot, not the environment (no cameras, external sensors, etc).
- **Gazebo**: A more complex simulation environment that uses the ROS 2 Gazebo bridge and is included in the [pla10/ros2_ur5_interface](https://hub.docker.com/r/pla10/ros2_ur5_interface) Docker image.

## How to Use (URSim)
### 1. Start the UR5 Simulator
Run the URSim container using the provided bash script:
```bash
bash scripts/ur5.sh
```
This starts the [pla10/ursim_e-series](https://hub.docker.com/r/pla10/ursim_e-series) Docker container for UR5 simulation. Access the simulator via your browser at [http://localhost:6080](http://localhost:6080).

<img style="display: block; margin-left: auto; margin-right: auto;" alt="UR5 Simulation" width="800" src="https://gyazo.com/7e2514442ef1753eb8b20e2b674056fc/raw">

To prepare the robot to work with ROS2 you need to go to the program tab and add the URCaps > External Control.

---

### 2. Start the ROS 2 Interface
Run the ROS 2 container using the provided bash script:
```bash
bash scripts/ros2.sh
```
This starts the [pla10/ros2_ur5_interface](https://hub.docker.com/r/pla10/ros2_ur5_interface) container. Access the environment via noVNC at [http://localhost:6081](http://localhost:6081).

<img style="display: block; margin-left: auto; margin-right: auto;" alt="UR5 Simulation" width="800" src="https://gyazo.com/ca4a65bce9e2ac0e5217edfc423d5fa9/raw">

The user password is "ubuntu".

---

### 3. Run ROS 2 Nodes
- Open a terminal inside the ROS 2 container (accessible via noVNC).
- Navigate to the ROS 2 workspace:
  ```bash
  cd /home/ubuntu/ros2_ws
  ```
- Source the ROS 2 setup:
  ```bash
  source install/setup.bash
  ```
- Launch the control nodes using the provided launch files:
  ```bash
  ros2 launch ros2_ur5_interface interface.launch.py
  ```

## How to Use (Gazebo)
### 1. Start the UR5 Gazebo Simulation
Run the ROS 2 container using the provided bash script:
```bash
bash scripts/ros2.sh
```
This starts the [pla10/ros2_ur5_interface](https://hub.docker.com/r/pla10/ros2_ur5_interface) container. Access the environment via noVNC at [http://localhost:6081](http://localhost:6081).

- Open a terminal inside the ROS 2 container (accessible via noVNC).
- Navigate to the ROS 2 workspace:
  ```bash
  cd /home/ubuntu/ros2_ws
  ```
- Source the ROS 2 setup:
  ```bash
  source install/setup.bash
  ```
- Launch the simulation nodes using the provided launch files:
  ```bash
  ros2 launch ros2_ur5_interface sim.launch.py
  ```

### 2. Run ROS 2 Example Node
Run the trajectory publication node to visualize the UR5 robot moving:
```bash
ros2 run ros2_ur5_interface publish_trajectory_node
```

<img style="display: block; margin-left: auto; margin-right: auto;" alt="UR5 Trajectory Simulation" width="800" src="https://gyazo.com/e25c7f6d13dd8dd77d8a149f36bf9e17/raw">

<br><br>

## Connect to the real UR5 robot
Check the robot's IP address by following these steps:
1. Press the burger menu button on the top right of the robot's teach pendant.
2. Select "Settings".
3. Navigate to "System" > "Network".
4. Check the IP address.

<img style="display: block; margin-left: auto; margin-right: auto;" width="600" src="https://i.gyazo.com/thumb/3024/243c42358c019f3e0c9f0345ed28e31b-heic.jpg">

<br><br>
   
Check under the installation tab if in the URCaps > External Control your IP address is the same as the one you are using to connect to the robot.

<img style="display: block; margin-left: auto; margin-right: auto;" width="700" src="https://i.gyazo.com/954c9c5d830a27f247d87145b050b057.png">

<br><br>

To connect to the real UR5 robot you need to specify the IP address of the robot when launching the interface.launch.py file. You can do this by adding the following argument to the launch command:
```bash
ros2 launch ros2_ur5_interface interface.launch.py robot_ip:=<robot_ip>
```

<br><br>

<p style="display: block; margin-left: auto; margin-right: auto;">
<img src="images/ur5_demo1.gif" width="45%" style="display: inline-block; margin-right: 5px;" /> 
<img src="images/ur5_demo2.gif" width="45%" style="display: inline-block; margin-left: 5px;" /> 
</p>
---

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
