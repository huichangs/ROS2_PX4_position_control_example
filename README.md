
# ROS2_PX4_Offboard_Example

## Overview
Multi-uav simulation with camera using PX4 and ROS2

This repo is a derivative of Jaeyoung Lim and ARK-Electronics's Offboard example

https://github.com/Jaeyoung-Lim/px4-offboard

https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example

I've taken these example and added some functionality. 

### Prerequisites
* ROS2 Humble
* PX4 Autopilot
* Micro XRCE-DDS Agent
* px4_msgs
* Ubuntu 22.04
* Python 3.10
* gazebo garden


## Setup Steps

### Install PX4 Autopilot
To [Install PX4](https://docs.px4.io/main/en/ros2/user_guide.html) run this code 
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

Run this script in a bash shell to install everything

```
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

You will now need to restart your computer before continuing.


### Install ROS2 Humble
To install ROS2 Humble follow the steps [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Install Dependencies

Install Python dependencies as mentioned in the [PX4 Docs](https://docs.px4.io/main/en/ros/ros2_comm.html#install-ros-2) with this code

```
pip3 install --user -U empy pyros-genmsg setuptools
```

Additionally, if you do not install these packages, you will have problems loading Gazebo.

```
pip3 install kconfiglib
pip install --user jsonschema
pip install --user jinja2
```

### Build Micro DDS
As mentioned in the [PX4 Docs](https://docs.px4.io/main/en/ros/ros2_comm.html#setup-micro-xrce-dds-agent-client) run this code in order to build MicroDDS on your machine

```
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### Setup Workspace
This git repo is intended to be a ROS2 package that is cloned into a ROS2 workspace.

We're going to create a workspace in our home directory, and then clone in this repo and also the px4_msgs repo. 

For more information on creating workspaces, see [here](https://docs.ros.org/en/humble/Tutorials/Workspace/Creating-A-Workspace.html)

Run this code to create a workspace in your home directory

```
mkdir -p ~/position_control_example/src
cd ~/position_control_example/src
```

### Clone in Packages
We first will need the px4_msgs package. Our ROS2 nodes will rely on the message definitions in this package in order to communicate with PX4. Read [here](https://docs.px4.io/main/en/ros/ros2_comm.html#overview:~:text=ROS%202%20applications,different%20PX4%20releases) for more information.

Be sure you're in the src directory of your workspace and then run this code to clone in the px4_msgs repo

```
git clone https://github.com/PX4/px4_msgs.git
```

Once again be sure you are still in the src directory of your workspace. Run this code to clone in our example package

```
git clone https://github.com/huichangs/ROS2_PX4_position_control_example.git
```

Run this code to clone the repo



### Building the Workspace
The two packages in this workspace are px4_msgs and position_control.

Before we build these two packages, we need to source our ROS2 installation. Run this code to do that

```
source /opt/ros/humble/setup.bash
```

This will need to be run in every terminal that wants to run ROS2 commands. An easy way to get around this, is to add this command to your .bashrc file. This will run this command every time you open a new terminal window.

To build these two packages, you must be in workspace directory not in src, run this code to change directory from src to one step back i.e. root of your workspace and build the packages

```
colcon build
```
As mentioned in Jaeyoung Lim's [example](https://github.com/Jaeyoung-Lim/px4-offboard/blob/master/doc/ROS2_PX4_Offboard_Tutorial.md) you will get some warnings about setup.py but as long as there are no errors, you should be good to go.


After this runs, we should never need to build px4_msgs again. However, we will need to build position_control every time we make changes to the code. To do this, and save time, we can run
```
colcon build --packages-select position_control
```

If you tried to run our code now, it would not work. This is because we need to source our current workspace. This is always done after a build. To do this, be sure you are in the src directory, and then run this code

```
source install/setup.bash
```

We will run this every time we build. It will also need to be run in every terminal that we want to run ROS2 commands in.


### Running the Code
This example has been designed to run from one launch file that will start all the necessary nodes. The launch file will run a python script that uses gnome terminal to open a new terminal window for MicroDDS and Gazebo.

Run this code to start the example

```
ros2 launch position_control offboard_position_control.launch.py
```

This will run numerous things. In no particular order, it will run:

* processes.py in a new window
   * MicroDDS in a new terminal window
   * Gazebo will open in a second tab in the same terminal window
      * Gazebo GUI will open in it's own window
* control.py in a new window
   * Sends ROS2 Teleop commands
* RVIZ will open in a new window
* position_control.py runs as it's own node, and is the main node of this example


## Closing Simulation *IMPORTANT*
When closing the simulation, it is very tempting to just close the terminal windows. However, this will leave Gazebo running in the background, potentially causing issues when you run Gazebo in the future. To correctly end the Gazebo simulation, go to it's terminal window and click *Ctrl+C*. This will close Gazebo and all of it's child processes. Then, you can close the other terminal windows.
 


