# ROS-Chess-package

This is a robotics foundations coursework. The files can be found [here](https://moodle.gla.ac.uk/course/view.php?id=34588)

## Create package

```bash
cd ~/rf_ws/src
catkin_create_pkg chess_baxter rospy geometry_msgs sensor_msgs control_msgs trajectory_msgs baxter_core_msgs baxter_interface
```

## Copy nodes from courswork directory to package

```bash
cp ~/Desktop/coursework/spawn_chessboard.py ~/rf_ws/src/chess_baxter/src/spawn_chessboard.py
```

```bash
cp ~Desktop/coursework/delete_chessgame.py ~/rf_ws/src/chess_baxter/src/delete_chessgame.py
```

## Make exectutable

```bash
chmod +x ~/rf_ws/src/chess_baxter/src/spawn_chessboard.py
```

```bash
chmod +x ~/rf_ws/src/chess_baxter/src/delete_chessgame.py
```

## Make it

```bash
cd ~/rf_ws
catkin_make
```

## Add objects to gazebo (lecture 4 and 5)

inspect pick_and_place_moveit.py

look at load_gazebo_models and delete_gazebo_model

In `baxter.urdf.xacro` add(but for pieces and such):

```xml
 <xacro:include filename="$(find baxter_sim_examples)/models/cafe_table/cafe_table.xacro" />
```

convert chesboard.sdf to urdf 

```bash
roscd ~/rf_ws/src/chess_baxter/src/models/chessboard
export MESH_WORKSPACE_PATH=~/ros_ws/src
rosrun pysdf sdf2urdf.py model.sdf model.urdf
```

then to xacro

```bash
cp model.urdf ~/rf_ws/src/chess_baxter/src/chessboard.xacro
```

open the following to edit

1. Replace 
```xml
<robot name="chessboard"> 
```
```xml
<robot name="chessboard" xmlns:xacro="http://www.ros.org/wiki/xacro">
```

2. Add a joint of the chessboard such that Rviz and TF can understand where the coffee table is located in space and its place within the robot's and environment's transformation hierarchy. In here, we attach the chessboard to the world, so copy/paste the code below just before the robot closing tag in chessboard.xacro. Save the file after you finish adding the code below.

```
<joint name="chessboard_fixed" type="fixed">
  <origin rpy="0 0 0" xyz="1.0 0 -0.93"/>
  <axis xyz="0 0 1"/>
  <parent link="world"/>
  <child link="chessboard__link"/>
</joint>
```

open `baxter.urdf.xacro`:

```bash
code ~/ros_ws/src/baxter/baxter/baxter_common/baxter_description/urdf/baxter.urdf.xacro
```

 and add the following just before the robot closing tag:
 
 ```xml
<!-- Chess Board -->
<xacro:include filename="$(find chess_baxter)/src/chessboard.xacro" />
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
rosrun chess_baxter spawn_chessboard.py
```

### terminal 5

```bash
rosrun chess_baxter delete_chess_game.py
```
