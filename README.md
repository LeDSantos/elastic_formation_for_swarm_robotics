# elastic_formation_for_swarm_robotics
Code of Elastic Formations as Dynamic Boundaries for Potential Fields in Swarm Robotics

## Install the dependences and packages

For the turtlebot3:

```sh
$ sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
```

Install TurtleBot3 Packages

```sh
$ sudo apt install ros-noetic-dynamixel-sdk
$ sudo apt install ros-noetic-turtlebot3-msgs
$ sudo apt install ros-noetic-turtlebot3
```

For the simulation:
```sh
$ cd ~/catkin_ws/src/
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

For other packages:
```sh
$ sudo apt-get install ros-noetic-actionlib ros-noetic-costmap-2d \
  ros-noetic-move-base-msgs ros-noetic-nav-msgs \
  ros-noetic-navfn ros-noetic-roscpp \
  ros-noetic-tf2 ros-noetic-tf2-ros \
  ros-noetic-rospy
```

For teb:
```sh
$ sudo apt-get install ros-noetic-teb-local-planner
```

For the paper pakages:
```sh
$ cd ~/catkin_ws/src/
$ git clone https://github.com/LeDSantos/elastic_formation_for_swarm_robotics.git
```

## Execute the simulation

Choose the simulation scenario (House or Columns) and the formation configurations at central_control.cpp and move_robot.cpp


### Compile:

```sh
$ cd ~/catkin_ws
$ catkin_make
```

### Prepare the environment:

Run in each terminal that will be used:
```sh
source devel/setup.bash
export TURTLEBOT3_MODEL=burger
```

Unzip models.zip, place the bigRoomTurtle folder in the Gazebo's building_editor_models folder and cylinders in the Gazebo's model_editor_models folder. These models are used at the Columns scenario.

<!-- Descompacte modelos.zip, coloque a pasta bigRoomTurtle na pasta building_editor_models do Gazebo e cylinders na pasta model_editor_models do Gazebo. Esses modelos são utilizados no cenário colunas. -->

### Compile:

```sh
catkin_make ## in project folder
```

### Execute:

To start gazebo and rviz:
```sh
roslaunch multiple_turtlebots_nav simulation_(house or columns).launch ## there are some rviz configs at src/multiple_turtlebots_nav/navigation
```

To start the metrics:
```sh
roslaunch metrics metrics.launch environment:="(house or columns)" ## THIS IS OPTIONAL
```

To start central control and 3 robots:
```sh
roslaunch central_control run_central_and3robots.launch
```

## Useful commands:

To create new map:
```sh
roslaunch multiple_turtlebots_nav map_new_room.launch
```
To save the map after mapping:
```sh
rosrun map_server map_saver
```
# For real tests

Launch each turtlebot.

<!-- PUT LAUNCH FILE -->

Then config simulation_lab233.launch: put the init position taht was used in generated_robots_lab2332robots.launch at central_control.cpp (CONFIG_REAL_TEST) and in run_central_and2REALrobots.launch

To start rviz:
```sh
roslaunch multiple_turtlebots_nav simulation_lab233.launch ## there are some rviz configs at src/multiple_turtlebots_nav/navigation
```

To start the metrics:
```sh
roslaunch metrics metrics.launch environment:="(.....)" ## THIS IS OPTIONAL
```

To start central control and 2 robots:
```sh
roslaunch central_control run_central_and2REALrobots.launch
```