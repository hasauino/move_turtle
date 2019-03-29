# move_turtle
H-BRS foundation course ROS project. A simple action server for controlling the trutle in the turtlesim_node to go from it's current pose to a goal pose.

# Running example

- To run all the nodes (turtlesim node, turtle_teleop_key node, and the move_turtle_node), run this launch file:

```
roslaunch move_turtle all.launch
```

- To run only the move_turtle_node, run this launch file:
```
roslaunch move_turtle move_turtle_node.launch
```

# Smach Example
After running all the nodes (```all.launch```), run the ```smach_example``` node:
```
rosrun move_turtle smach_example 
```

# Video

[Youtube video here](https://www.youtube.com/watch?v=vQmRY5bl4VI&feature=youtu.be)
