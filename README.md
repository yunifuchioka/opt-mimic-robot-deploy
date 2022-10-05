# robot_script
## Setup Instructions
### Enviroment setup
See my google docs for setting up ROS2 and an RT_PREEMPT patched machine
### Install dependencies and set up project directory
1. `mkdir` and `cd` into a directory to contain all project files. In my case `~/Documents/robot_script_ws`
2. Clone the [Machines in Motion Lab treep package](https://github.com/machines-in-motion/treep_machines_in_motion). In my case
```
git clone git@github.com:machines-in-motion/treep_machines_in_motion.git
```
3. Use treep to clone the [real_time_tools](https://github.com/machines-in-motion/real_time_tools) repo and all of its dependencies:
```
treep --clone REAL_TIME_TOOLS
```
Do the same for the [odri_control_interface](https://github.com/open-dynamic-robot-initiative/odri_control_interface) repo:
```
treep --clone ODRI_CONTROL_INTERFACE
```
Afterwards, my directory looks like
```
robot_script_ws/
	treep_machines_in_motion/
	workspace/
		src/
			googletest/
			master-board/
			mpi_cmake_modules/
			odri_control_interface/
			pybind11/
			real_time_tools/
			yaml_utils
```
4. Create a `config` folder. This folder will store configuration files for the robot.
```
mkdir config
```
then my directory becomes
```
robot_script_ws/
	config/
	treep_machines_in_motion/
	workspace/
		src/
			googletest/
			master-board/
			mpi_cmake_modules/
			odri_control_interface/
			pybind11/
			real_time_tools/
			yaml_utils
```
5. Clone this repo
```
cd workspace/src/
git clone https://github.com/yunifuchioka/robot_script.git
```
finally, my directory becomes
```
robot_script_ws/
	config/
	treep_machines_in_motion/
	workspace/
		src/
			googletest/
			master-board/
			mpi_cmake_modules/
			odri_control_interface/
			pybind11/
			real_time_tools/
			robot_script/
			yaml_utils
```

### Run main script
1. Switch to root, which is necessary for the network communcation
```
sudo -s
```
2. Source ROS2 if this isn't done automatically in the `.bashrc`. In my case
```
source /opt/ros/dashing/setup.bash
source /opt/openrobots/setup.bash
```
3. `cd` to the workspace directory (`cd ~/Documents/robot_script_ws/workspace` in my case), then build with colcon
```
colcon build
```
4. Source the setup file
```
source install/setup.bash
```
5. Calibrate the robot. This needs to be done after every power up of the robot, and it saves a file to the `config` folder created earlier. `MY_INTERFACE` is the name of the network interface, obtained from running `ifconfig`.
```
ros2 run robot_script calibrate MY_INTERFACE
```

6. Run the main script. `MY_INTERFACE` is the name of the network interface, obtained from running `ifconfig`.
```
ros2 run robot_script main MY_INTERFACE
```
