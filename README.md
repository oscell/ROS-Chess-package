# ROS Chess Baxter: Autonomous Chess Playing with Baxter Robot

This project develops a ROS solution that enables Baxter to autonomously play chess. Baxter packages and tutorials were provided, which include Baxter setting up the chessboard with at least 5 pieces and moving each piece in a sequence of valid chess moves.

The full solution utilizes ROS, Gazebo, RViz, and MoveIt. A demonstration video and the source code are included in the repository.

<div align="center">

:movie_camera: [Click here](https://www.youtube.com/watch?v=VA5hQQhipFk) to watch Baxter in action! :movie_camera:

<img src="your-image-link" width="500">

</div>
<video src="assets/Video/Chess%20Baxter%20-%20Made%20with%20Clipchamp.mp4" controls title="Title"></video>
## Overview

Our approach to the problem was threefold:

1. Implementing the broadcaster and listener.
2. Placing the chess pieces autonomously.
3. Executing a sequence of valid chess moves.

For a detailed breakdown of our approach, the challenges faced, and our proposed improvements, refer to the **Reflective Analysis** section below.

## Repository Contents
- **[Courswork Description]()**
- **[Video Demonstration](link-to-your-video)**: A visual showcase of Baxter completing the tasks.
- **[ROS Package](link-to-zip-file)**: Contains the ROS package with all the source code.
- **[Reflective Analysis](#reflective-analysis)**: Insights into our development process, challenges, and proposed enhancements.

## Setup & Installation

### Prerequisites
* ROS
* Gazebo
* RViz
* MoveIt

### Clone and Build

```bash
cd ~/rf_ws
git clone your-repo-link
catkin_make
```

## Usage

Before launching, **rename the package to chess_baxter**.

1. **Startup**:

   ```bash
   roslaunch baxter_gazebo baxter_world.launch
   ```

2. **Enable and publish joint trajectories**:

   ```bash
   rosrun baxter_tools enable_robot.py -e
   rosrun baxter_interface joint_trajectory_action_server.py
   ```

3. **Enable grippers**:

   ```sh
   roslaunch baxter_moveit_config baxter_grippers.launch
   ```

4. **Spawn chessboard**:

   ```sh
   rosrun chess_baxter spawn_chessboard.py
   ```

5. **Broadcast positions**:

   ```bash
   rosrun chess_baxter gazebo2tfframe.py
   ```

6. **Place chess pieces**:

   ```sh
   rosrun chess_baxter pick_and_place_moveit.py
   ```

7. **Play chess**:

   ```sh
   rosrun chess_baxter play_chess
   ```

8. **Delete Board**:

   ```bash
   rosrun chess_baxter delete_chess_game.py
   ```

## Reflective Analysis

### Approach & Assumptions

Our system utilizes a broadcaster node to publish the position and orientation of each block. It's then parsed by a listener within the pick and place node, providing the pose of each block relative to the base. This enables Baxter to determine how to pick up each block.

Assumptions made:

1. Chess pieces are given ground truth positions, eliminating the need for environmental scanning.
2. No obstacles or barriers obstruct the robot's movement.
3. All chess pieces are represented as cubes, simplifying the gripping process.

### Challenges & Solutions

One major challenge was the inconsistency in simulation results. Different outcomes would sometimes result from the same commands due to environmental and software limitations.

Solutions:

1. Initialize conditions, ensuring the robot always starts from the same state.
2. Run multiple simulations and average over the results for more consistent outcomes.
3. Add checks that ensure a robot reaches a specific pose before the next command.

For a deeper dive into the challenges faced and our strategies for overcoming them, refer to the **full analysis**.

### Vision Module with Deep Learning

To replace the Canny edge detector and image moments from Lab 5, a deep learning model could be implemented using an image database of chess pieces. The robot vision system must be able to classify the type of piece, its location, and orientation.

Steps include:

1. Annotate images for orientation and position.
2. Preprocess data, resize images, normalize pixel values, and split datasets.
3. Design and train a convolutional neural network (CNN) for classification.

## Acknowledgements

Special thanks to:

- Dr. Gerardo Aragon-Camarasa
- Lab Demonstrators: Florent Audonnet and Anith Ravindran