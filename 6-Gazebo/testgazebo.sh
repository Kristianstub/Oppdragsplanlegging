cd ~/catkin_ws && catkin_make
export TURTLEBOT3_MODEL=waffle_pi
source devel/setup.bash
roslaunch turtlebot3_gazebo turtlebot3_ttk4192.launch