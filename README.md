# ROS-Chess-package

This is a robotics foundations coursework. The files can be found [here](https://moodle.gla.ac.uk/course/view.php?id=34588)

- [ ] Add a assesment package called "chess"

## Create package

```bash
cd ~/rf_ws/src
catkin_create_pkg chess_pkg rospy geometry_msgs sensor_msgs control_msgs trajectory_msgs baxter_core_msgs baxter_interface
```
## Copy nodes from courswork directory to package

```bash
cp ~/Desktop/coursework/spawn_chessboard.py ~/rf_ws/src/chess_pkg/src/spawn_chessboard.py
```

```bash
cp ~Desktop/coursework/delete_chessgame.py ~/rf_ws/src/chess_pkg/src/delete_chessgame.py
```

## Make exectutable

```bash
chmod +x ~/rf_ws/src/chess_pkg/src/spawn_chessboard.py
```

```bash
chmod +x ~/rf_ws/src/chess_pkg/src/delete_chessgame.py
```

## Make it

```bash
cd ~/rf_ws
catkin_make
```

## Add objects to gazebo

In `baxter.urdf.xacro` add:

```xml
 <xacro:include filename="$(find baxter_sim_examples)/models/cafe_table/cafe_table.xacro" />
```



## Launch

### terminal 1

```bash
roslaunch baxter_gazebo baxter_world.launch
```

### terminal 2

```bash
rosrun baxter_tools enable_robot.py -e
rosrun rviz rviz
```

### terminal 4

```bash
rosrun chess_pkg spawn_chessboard.py
```

### terminal 5

```bash
rosrun chess_pkg delete_chess_game.py
```
