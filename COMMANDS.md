## Robot Setup

1. Turn on the robot with the button beside e-stop (movo1)
2. Take off black back cover, click button to turn on smaller pc (movo2) on middle right
3. ping movo1 and ping movo2 to make sure they are on and connected to wifi (autoconnected to robohub network)
4. Do setup detailed in README for displaying rostopics


## Passwords and Sign In's
Surface: Surface!

## 

run on the movo1
roslaunch movo_demos map_nav.launch sim:=false local:=true map_file:=movo_map


roslaunch movo_viz view_robot.launch function:=map_nav

roslaunch movo_demos robot_map_nav.launch map_file:=movo_map

rosrun map_server map_saver -f ~/movo_map

rosrun gmapping slam_gmapping scan:=/movo/scan_multi _base_frame:=base_link _odom_frame:=odom _map_update_interval:=1.0[ INFO] [1762810286.701162467]: Done initializing likelihood field model.

 rosrun joy joy_node


rosrun map_server map_server /home/robohub/catkin_ws/src/kinova-movo/movo_demos/maps/floor_plan.yaml _frame_id:=map _topic:=/floor_plan


rosrun movo_nav goto_points.py _waypoints_file:=/home/krish/catkin_ws/src/kinova-movo/movo_nav/movo_waypoints.yaml _target:=storage_closet


rostopic echo /clicked_point

python3 realsense_person_counter.py 
rostopic echo /people/count


