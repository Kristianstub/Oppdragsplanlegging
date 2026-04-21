# Oppdragsplanlegging

This is a repo containing code for the trutlebot.

## Installation

Before cloning and installation, make sure ROS 1 is installed and that the `catkin_ws` folder is filled with the github repos.
Then clone the repo, including the submodules like this:

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

For the later tasks, where python3 is needed, UV is a super smooth package and project manager.
Install UV using the comamnd below.

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

## Some useful command for the different tasks

### Task 5 - Compute a plan

Compute a plan using the pddl files:

```bash
cd catkin_ws/src
source temporal-planning-main/bin/activate
cd temporal-planning-main/temporal-planning/

python2.7 bin/plan.py stp-2 ~/catkin_ws/src/temporal-planning-main/temporal-planning/domains/ttk4192/domain/PDDL_domain_1.pddl ~/catkin_ws/src/temporal-planning-main/temporal-planning/domains/ttk4192/problem/PDDL_problem_1.pddl
```

You can then see the plan here:
temporal-planning-main/temporal-planning/,tmp sas plan.1"

