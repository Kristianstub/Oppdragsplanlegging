
rm -rf ~/catkin/src/temporal-planning-main
ln -sf "$(pwd)/temporal-planning-main" ~/catkin_ws/src/
rm -rf ~/catkin_ws/src/temporal-planning-main/temporal-planning/domains/ttk4192
ln -sf "$(pwd)/3-PDDL" ~/catkin_ws/src/temporal-planning-main/temporal-planning/domains/ttk4192

ln -sf "$(pwd)/6-Gazebo/turtlebot3_ttk4192.world" ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/
ln -sf "$(pwd)/6-Gazebo/turtlebot3_ttk4192.launch" ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/

rm -rf ~/catkin_ws/src/assignment4_ttk419
ln -sf "$(pwd)/mission-planner" ~/catkin_ws/src/assignment4_ttk419

