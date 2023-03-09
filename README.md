# ROS-Chess-package

This is a robotics foundations coursework. The files can be found [here](https://moodle.gla.ac.uk/course/view.php?id=34588)

## Launch

### terminal 1

```bash
roslaunch baxter_gazebo baxter_world.launch
```

### terminal 2

```bash
rosrun baxter_tools enable_robot.py -e
rosrun baxter_interface joint_trajectory_action_server.py
```

### Terminal 3:

```sh
roslaunch baxter_moveit_config baxter_grippers.launch
```

### Terminal 4:

```sh
rosrun chess_baxter spawn_chessboard.py
rosrun chess_baxter pick_and_place_moveit.py
```

### terminal 5

```bash
rosrun chess_baxter delete_chess_game.py
```

### Steps
#### tf_listener.py
##### Make sure the node is set up
Add the following code above rate.sleep() in line 33:

```python
q = transformation.transform.rotation
quat = (q.x, q.y, q.z, q.w) 
rot_mat = quaternions.quat2mat(quat)
rospy.loginfo("Rotation matrix \n" + str(rot_mat))

H = numpy.eye(4)
trans = transformation.transform.translation
H[:3,:3] = rot_mat
H[:3,3] = [trans.x, trans.y, trans.z]
rospy.loginfo("Homogenous matrix: \n" + str(H))

new_quat = quaternions.mat2quat(rot_mat)
rospy.loginfo("New quaternion: " + str(new_quat))
```

##### Run the node

```sh
rosrun lab2_pkg tf_listener.py
```

#### pick_and_place_moveit.py
Terminal 1:

```sh
roslaunch baxter_gazebo baxter_world.launch
```

Terminal 2:

```sh
rosrun baxter_tools enable_robot.py -e
rosrun baxter_interface joint_trajectory_action_server.py
```

Terminal 3:

```sh
roslaunch baxter_moveit_config baxter_grippers.launch
```

Terminal 4:

```sh
rosrun lab4_pkg pick_and_place_moveit.py
```
## Create package

```bash
cd ~/rf_ws/src
catkin_create_pkg chess_baxter rospy geometry_msgs sensor_msgs control_msgs trajectory_msgs baxter_core_msgs baxter_interface
```

### Copy nodes from courswork directory to package

```bash
cp ~/Desktop/coursework/spawn_chessboard.py ~/rf_ws/src/chess_baxter/src/spawn_chessboard.py
cp ~/Desktop/coursework/delete_chessgame.py ~/rf_ws/src/chess_baxter/src/delete_chessgame.py
cp -R ~/Desktop/coursework/models ~/rf_ws/src/chess_baxter
```

### Make exectutable

```bash
chmod +x ~/rf_ws/src/chess_baxter/src/spawn_chessboard.py
chmod +x ~/rf_ws/src/chess_baxter/src/delete_chessgame.py
```

### Make it

```bash
cd ~/rf_ws
catkin_make
```
