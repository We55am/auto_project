# Milestone 5

## Steps for running the gazebo model

```
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch auto_ms4 ms4_launch.launch
# for running the pathplanning algorithm, in another terminal
# change the directory first then run any of the three algorithms.
$ source ~/catkin_ws/devel/setup.bash
$ roscd auto_ms4/scripts/
$ rosrun auto_ms4 BFS.py
```

