# enpm808x_final_inspection_robot

[![Build Status](https://app.travis-ci.com/rnvandemark/enpm808x_final_inspection_robot.svg?branch=master)](https://app.travis-ci.com/rnvandemark/enpm808x_final_inspection_robot)
[![Coverage Status](https://coveralls.io/repos/github/rnvandemark/enpm808x_final_inspection_robot/badge.svg?branch=master)](https://coveralls.io/github/rnvandemark/enpm808x_final_inspection_robot?branch=master)

## Project Overview/Description

**TODO**

## Personnel

### Aditya Jadhav

**TODO**

#### Abhishek Nalawade

**TODO**

#### R. Nick Vandemark

**TODO**

## License

**TODO**

## Links to Agile Iterative Process (AIP) Products

To Project Backlog (Product Backlog, Iteration Backlogs, and Work Log):

[https://docs.google.com/spreadsheets/d/1DmnGjTfYCdlwXq4yxJ25zSwCLq8LcW4DftqtF5_p5Tk/edit?usp=sharing](https://docs.google.com/spreadsheets/d/1DmnGjTfYCdlwXq4yxJ25zSwCLq8LcW4DftqtF5_p5Tk/edit?usp=sharing)

To Sprint Planning Notes/Review:

[https://docs.google.com/document/d/1JHqd9Alk2kZUKKPmb6cLGOPEvR-gZKouV5svNGUF8SE/edit?usp=sharing](https://docs.google.com/document/d/1JHqd9Alk2kZUKKPmb6cLGOPEvR-gZKouV5svNGUF8SE/edit?usp=sharing)

Furthermore, see the "UML" directory of this package for UML files, and Proposal
deliverables of the initial proposal in the "Proposal" directory.

## Known Issues/Bugs

**TODO**

## Dependencies

The following should install all of the required packages (assuming the proper
sources have been declared, see [ROS Melodic installation](http://wiki.ros.org/melodic/Installation/Ubuntu) otherwise):
```
sudo apt-get update
sudo apt-get install git python-rosinstall ros-melodic-desktop-full python-catkin-tools ros-melodic-joint-state-controller ros-melodic-twist-mux ros-melodic-ompl ros-melodic-controller-manager ros-melodic-moveit-core ros-melodic-moveit-ros-perception ros-melodic-moveit-ros-move-group ros-melodic-moveit-kinematics ros-melodic-moveit-ros-planning-interface ros-melodic-moveit-simple-controller-manager ros-melodic-moveit-planners-ompl ros-melodic-joy ros-melodic-joy-teleop ros-melodic-teleop-tools ros-melodic-control-toolbox ros-melodic-sound-play ros-melodic-navigation ros-melodic-depthimage-to-laserscan ros-melodic-moveit-commander
```

## How to

### Setting Up Your Workspace

- Create a catkin workspace and its src directory at e.g. /path/to/tiago_ws/src
- Clone this package into the src directory:
```
cd /path/to/tiago_ws/src
git clone https://github.com/rnvandemark/enpm808x_final_inspection_robot.git
```
- Navigate to the catkin workspace:
```
cd /path/to/tiago_ws/
```
- Use rosinstall to download additional packages into your workspace, using the
  additional dependencies declare in this repository's rosinstall file:
```
# If needed, replace 'enpm808x_final_inspection_robot' with the local name you
# gave this repository
rosinstall src /opt/ros/melodic src/enpm808x_final_inspection_robot/dependencies.rosinstall
```
- Set up rosdep and ensure any missing dependencies required by this workspace
  are met/installed:
```
sudo rosdep init
rosdep update
cd /path/to/tiago_ws/
rosdep install --from-paths src --ignore-src -y --rosdistro melodic --skip-keys="opencv2 opencv2-nonfree pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev python-graphitesend-pip python-statsd pal_filters pal_vo_server pal_usb_utils pal_pcl pal_pcl_points_throttle_and_filter pal_karto pal_local_joint_control camera_calibration_files pal_startup_msgs pal-orbbec-openni2 dummy_actuators_manager pal_local_planner gravity_compensation_controller current_limit_controller dynamic_footprint dynamixel_cpp tf_lookup opencv3 joint_impedance_trajectory_controller cartesian_impedance_controller omni_base_description omni_drive_controller"
```
- Optionally, add this to ~/.bashrc (and resource as necessary) to restrict
  message generation for extra languages:
```
export ROS_LANG_DISABLE=genlisp:gennodejs:geneus
```

### Building the Program and Tests

#### Building the Package

- Prerequisite: you have set up your workspace as described in the previous
  section, assuming the catkin workspace at e.g. /path/to/tiago_ws/
- Navigate to the catkin workspace and build with the build script:
```
cd /path/to/tiago_ws/
./src/enpm808x_final_inspection_robot/bin/build-ws.sh
```

#### Building the Tests

The tests for this package are automatically built with the aforementioned
build script, build-ws.sh. See the 'Building the Package' section.

### Running a Sample of the Program

**TODO**

### Running the Tests

The tests for this package can be ran by navigating to the project workspace
directory and running the helper script:
```
cd /path/to/tiago_ws/
./src/enpm808x_final_inspection_robot/bin/run-tests.sh
```

### Generating Doxygen Docs

**TODO**
