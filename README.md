# ros2-examples-crane-cmake
Simple ROS2 example: one package, two nodes (one publisher, one subscriber). MAS418 2022. Tested with ROS2 Galactic.

One node is missing. Add it!


<br />

Build and run:


1. Source your ROS2 environment ("underlay"). If built as described here: https://docs.ros.org/en/galactic/Installation/Ubuntu-Development-Setup.html
```console
. ~/ros2_galactic/install/local_setup.bash
```

2. Build packages

```console
colcon build
```

3. Source overlay for finding the new packages
```console
. install/local_setup.bash 
```

4. Run
```console
ros2 launch crane_cpp crane.launch.py
```
