# Oppdragsplanlegging

This is a repo containing code for the trutlebot.

## Installation

Before cloning and installation, make sure ROS 1 is installed and that the `catkin_ws` folder is filled with the github repos.
Then clone the repo, including the submodules like shown below. Do not clone the repo inside `catkin_ws`.

```bash
git clone --recursive git@github.com:Kristianstub/Oppdragsplanlegging.git
```

Then run the installation script to build the AI planner within this directory.
This will also set up the symlinks for the rest of the system.

Installation script:
```bash
./install.sh
```

If you for some reason need to create the symlinks again, do so by running.
```bash
./setupSymlink.sh
```

## Show the simulation

Run these commands in the background.

Start the core, this will forward messages where they should go.
```bash
roscore
```

To visualize the simulation, use the folliwing command
```bash
./6-Gazebo/testgazebo.sh
```

## Some useful commands for the different tasks

### Task 5 - Compute a plan

Compute a plan using the pddl files:

```bash
cd catkin_ws/src
source temporal-planning-main/bin/activate
cd temporal-planning-main/temporal-planning/

python2.7 bin/plan.py stp-2 ~/catkin_ws/src/temporal-planning-main/temporal-planning/domains/ttk4192/domain/PDDL_domain_1.pddl ~/catkin_ws/src/temporal-planning-main/temporal-planning/domains/ttk4192/problem/PDDL_problem_1.pddl
```

You can then see the plan here:
temporal-planning-main/temporal-planning/tmp_sas_plan.1"

### Task 6 - Gazebo

Open the world file in Gazebo by running. (This requires a GUI)

```bash
gazebo ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_ttk4192.world
```

To show the map with the robot, use these commands.

```bash
./6-Gazebo/testgazebo.sh
rqt
```

### Task 7 and 8 - GNC

Make sure UV is installed, see `install.sh`.

Set up the venv to get all the packages needed. First download the dependencides

```bash
cd mission-planner
uv sync
```

Then activate the venv.
```bash
source .venv/bin/activate
```

To test if the mission planner works run these commands:
```bash
cd ~/catkin_ws
source devel/setup.bash
rosrun assigment4_ttk4192 mission_planner_ttk4192.py
```

0.000: ( move turtlebot0 waypoint0 waypoint1 d01 ) [15.4444]
15.444: ( manipulate_valve turtlebot0 waypoint1 valve0 ) [10.0000]
25.446: ( move turtlebot0 waypoint1 waypoint2 d12 ) [52.5000]
77.946: ( manipulate_valve turtlebot0 waypoint2 valve1 ) [10.0000]
87.948: ( move turtlebot0 waypoint2 waypoint5 d25 ) [26.1111]
114.059: ( take_picture turtlebot0 waypoint5 pump0 ) [5.0000]
119.061: ( move turtlebot0 waypoint5 waypoint6 d56 ) [32.2222]
151.284: ( take_picture turtlebot0 waypoint6 pump1 ) [5.0000]
156.284: ( move turtlebot0 waypoint6 waypoint4 d64 ) [23.7222]
