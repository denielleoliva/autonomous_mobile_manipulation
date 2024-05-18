
1. Start simulation & moveit from original instructions

```
roslaunch robowork_gazebo bvr_SIM_playpen.launch
```

2. Start compliance controller

3. Start pointcloud to laserscan

```
rosrun pointcloud_to_laserscan pointcloud_to_laserscan_node cloud_in:=/bvr_SIM/bvr_SIM/velodyne_points
```

4. Start force packages


```
#in its own terminal

rosrun effector_control slider_service.py

#start in new sourced terminal

rosrun effector_control follow.py
```

Apply forces through gui. Robot behavior should follow force and avoid obstacles. (with respect to the map frame) Testing for obstacle avoidance was done in the provided world file through navigation around big obstacles like the wall and beams included.
