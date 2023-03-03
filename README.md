# ROS-Chess-package

This is a robotics foundations coursework. The files can be found [here](https://moodle.gla.ac.uk/course/view.php?id=34588)

- [ ] Add a assesment package called "chess"

```bash
cd ~/rf_ws/src
catkin_create_pkg chess_pkg rospy geometry_msgs sensor_msgs control_msgs trajectory_msgs baxter_core_msgs baxter_interface
```

- [ ] Add the following nodes to the package src

```bash
cp ~/Desktop/coursework/spawn_chessboard.py ~/rf_ws/src/chess_pkg/src/spawn_chessboard.py
```

```bash
cp ~Desktop/coursework/delete_chessgame.py ~/rf_ws/src/chess_pkg/src/delete_chessgame.py
```