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

terminal 1: Startup

```bash
roslaunch baxter_gazebo baxter_world.launch
```

terminal 2: Enable and publish joint trajectories

```bash
rosrun baxter_tools enable_robot.py -e
rosrun baxter_interface joint_trajectory_action_server.py
```

terminal 3: Enable grippers

```sh
roslaunch baxter_moveit_config baxter_grippers.launch
```

terminal 4: Spawn chesboard

```sh
rosrun chess_baxter spawn_chessboard.py
```

Terminal 5: Broadcaster

To  find more possible [rostopics](http://wiki.ros.org/rostopic) type
```
rosrun ches_baxter gazebo2tfframe.py
```

This should output the positions of the head with refference to origin (\base rostopic)

Terminal 6: Place chess pieces

```sh
rosrun chess_baxter pick_and_place_moveit.py
```

Terminal 7: Place chess pieces

```sh
rosrun chess_baxter play_chess
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
