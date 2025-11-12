# `Important: This repo was deliberately not forked, as the container contains private packages and internal code we don't want to be exposed or publicly associated with an upstream repo.`

The container contains a map of the track used at The IEEE INTELLIGENT VEHICLES SYMPOSIUM (IV 2025), called ieee2. 

# F1TENTH gym environment ROS2 communication bridge
This is a containerized ROS communication bridge for the F1TENTH gym environment that turns it into a simulation in ROS2.

The installed ros2 in the container is a foxy version but as soon as car1 and car2 will be able to use humble, I will update the container as well.
# Installation

**Supported System:**

- Windows 10, macOS, and Ubuntu


## Without an NVIDIA gpu:

**Install the following dependencies:**

If your system does not support nvidia-docker2, noVNC will have to be used to forward the display.
- Again you'll need **Docker**. Follow the instruction from above.
- Additionally you'll need **docker-compose**. Follow the instruction [here](https://docs.docker.com/compose/install/) to install docker-compose.

**Installing the simulation:**

1. Clone this repo 
2. Bringup the novnc container and the sim container with docker-compose:
```bash
docker-compose up
#or 
docker compose up
``` 
Ok if you didn't get any error messages and you see a ubuntus computer interface in novnc then congratulations, your container is running!

Connecting to a Docker Container via Visual Studio Code (Simpler Method)
Instead of running terminal commands manually like:

docker exec -it f1tenth_gym_ros-sim-1 /bin/bash
You can use Visual Studio Code (VS Code) to connect to the container and open a terminal inside it much more easily:

Steps:
Install Required Extension
Open VS Code.
Go to the Extensions panel (Ctrl+Shift+X).
Search for and install the Dev Containers extension
(previously called Remote - Containers).
Attach to the Running Container
Press F1 or Ctrl+Shift+P to open the command palette.
Type:
"Dev Containers: Attach to Running Container..."
Select your container from the list (e.g., f1tenth_gym_ros-sim-1).
Then press the oren folders menu and start typing "/sim_ws/" and press enter.
Open Terminal Inside the Container
Once attached, VS Code will switch into the container environment.
Open a terminal via Ctrl+ or from the menu:
Terminal > New Terminal.
The terminal will now be running inside the container – no need for docker exec.
Access the VNC GUI
To access the GUI from the browser:

Open your web browser.
Go to:
http://localhost:8080/vnc.html
You should see the noVNC logo.
Click the Connect button to open the graphical session.

# Launching the Simulation

```bash
$ cd sim_ws
$ colcon build
$ source /opt/ros/foxy/setup.bash
$ source install/local_setup.bash
$ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
A rviz window should pop up showing the simulation either on your host system or in the browser window depending on the display forwarding you chose.


