# ros2-examples-pubsub-cmake
Simple ROS2 example: one package, two nodes (one publisher, one subscriber). Tested with ROS2 Galactic.

One node is missing. Add it!




Build and run:


1. Source your ROS2 environment.

2. Build

cd pubsub 
colcon build --packages-select pubsub_cpp

3. Source environment extras for finding the new package
. install/local_setup.bash 

4. Run
ros2 launch pubsub_cpp pubsub.launch.py
