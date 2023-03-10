# ROS-Chess-package

This is a robotics foundations coursework. The files can be found [here](https://moodle.gla.ac.uk/course/view.php?id=34588)

## Git pull and code
```
cd ~/rf_ws/src/chess_baxter/
code .
```

```
cd ~/rf_ws/src/chess_baxter/
git pull
```

## Launch

terminal 1

```bash
roslaunch baxter_gazebo baxter_world.launch
```

terminal 2

```bash
rosrun baxter_tools enable_robot.py -e
rosrun baxter_interface joint_trajectory_action_server.py
```

terminal 3:

```sh
roslaunch baxter_moveit_config baxter_grippers.launch
```

terminal 4:

```sh
rosrun chess_baxter spawn_chessboard.py
```

### Listener

This should output the positions of the head with refference to origin (\base rostopic)
```sh
rosrun chess_baxter tf_listener.py
```

To  find more possible [rostopics](http://wiki.ros.org/rostopic) type

```
rostopic info tf
```

### Broadcaster
Taken from [here](http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29), this should output the position of the block.

```
rosrun chess_baxter gazebo2tfframe.py
```

### Pick and place

```sh
rosrun chess_baxter pick_and_place_moveit.py
```

### Delete Board

```bash
rosrun chess_baxter delete_chess_game.py
```

### Flow chart

![image](https://user-images.githubusercontent.com/82882938/224342808-c0500f92-1435-4d2e-b013-f0fc948d41c2.png)

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
