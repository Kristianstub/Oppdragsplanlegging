# Oppdragsplanlegging

This is a repo containing code for the trutlebot.

## Installation

Before cloning and installation, make sure ROS 1 is installed and that the `catkin_ws` folder is filled with the github repos.
Then clone the repo, including the submodules like this:

```bash
git clone -b theodor/task1 --recursive git@github.com:Kristianstub/Oppdragsplanlegging.git
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
