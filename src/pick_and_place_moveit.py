#!/usr/bin/env python
import sys
import copy

import rospy
import rospkg

from std_msgs.msg import (
    Empty,
)

from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

import baxter_interface
import moveit_commander


class PickAndPlaceMoveIt(object):
    def __init__(self, limb, hover_distance=0.2, verbose=True):
        self._limb_name = limb  # string
        self._hover_distance = hover_distance  # in meters
        self._verbose = verbose  # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")

        self._robot = moveit_commander.RobotCommander()
        # This is an interface to one group of joints.  In our case, we want to use the "right_arm".
        # We will use this to plan and execute motions
        self._group = moveit_commander.MoveGroupCommander(limb+"_arm")

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))

        self.gripper_open()
        self._group.set_pose_target(start_angles)
        plan = self._group.plan()
        self._group.execute(plan)
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance

        self._group.set_pose_target(approach)
        plan = self._group.plan()
        self._group.execute(plan)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w

        # servo up from current pose
        self._group.set_pose_target(ik_pose)
        plan = self._group.plan()
        self._group.execute(plan)

    def _servo_to_pose(self, pose):
        # servo down to release
        self._group.set_pose_target(pose)
        plan = self._group.plan()
        self._group.execute(plan)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def wait(self):
        # go to the position
        self._servo_to_pose(Pose(
        position=Point(x=0.5, y=0.5, z=0.3),
        orientation=Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)))


    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

    


def load_gazebo_models(table_pose=Pose(position=Point(x=1.0, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.6, y=0.6, z=0.7825)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    # Load Table SDF
    table_xml = ''
    with open(model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml = table_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_sdf("cafe_table", table_xml, "/", table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))


def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model("cafe_table")
        delete_model("block")
        delete_model("chessboard")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))    

def load_new(piece, name, block_pose=Pose(position=Point(x=0.6, y=0.6, z=0.7825))):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('chess_baxter')+"/models/"
    # Load block SDF
    block_xml = ''
    with open(model_path + piece, "r") as block_file:
        block_xml = block_file.read().replace('\n', '')
        # Spawn block SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_sdf(name, block_xml, "/", block_pose, "world")
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ik_pick_and_place_moveit")
    
    limb = 'left'
    hover_distance = 0.15  # meters
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)
    # NOTE: Gazebo and Rviz has different origins, even though they are connected. For this
    # we need to compensate for this offset which is 0.93 from the ground in gazebo to
    # the actual 0, 0, 0 in Rviz.
    starting_pose = Pose(
        position=Point(x=0.6, y=0.6, z=0.35),
        orientation=overhead_orientation)
    pnp = PickAndPlaceMoveIt(limb, hover_distance)

    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    load_gazebo_models()

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    

    
    

    block_poses = list()
    # The Pose of the block in its initial location.
    # You may wish to replace these poses with estimates
    # from a perception node.

    # NOTE: Remember that there's an offset in Rviz wrt Gazebo. We need
    # to command MoveIt! to go below because the table is 74 cm height.
    # Since the offset is 0.93, we just simply need to substract
    # 0.74 - 0.93 = -0.15 in Z
    #load_new

    position = rospy.get_param("piece_target_position_map")

    start = Point(x=0.6, y=0.6, z=-0.14)

    #setting z to 0 just to it pauses above the block
    block_poses.append(Pose(
        position=start,
        orientation=overhead_orientation))

    block_poses.append(Pose(
        position=Point(position['05'][0], position['05'][1], position['05'][2]),
        orientation=overhead_orientation))

    block_poses.append(Pose(
        position=start,
        orientation=overhead_orientation))

    block_poses.append(Pose(
        position=Point(position['02'][0], position['02'][1], position['02'][2]),
        orientation=overhead_orientation)) 

    block_poses.append(Pose(
        position=start,
        orientation=overhead_orientation))

    block_poses.append(Pose(
        position=Point(position['00'][0], position['00'][1], position['00'][2]),
        orientation=overhead_orientation))
        ##r0 in corect poistion

    block_poses.append(Pose(
        position=start,
        orientation=overhead_orientation))

    block_poses.append(Pose(
        position=Point(position['07'][0], position['07'][1], position['07'][2]),
        orientation=overhead_orientation)) 

    ########White pieces are now set up

    ####

    ########Now set up black pieces

    block_poses.append(Pose(
    position=start,
    orientation=overhead_orientation))

    block_poses.append(Pose(
        position=Point(position['75'][0], position['75'][1], position['75'][2]),
        orientation=overhead_orientation))

    block_poses.append(Pose(
        position=start,
        orientation=overhead_orientation))

    block_poses.append(Pose(
        position=Point(position['72'][0], position['72'][1], position['72'][2]),
        orientation=overhead_orientation)) 

    block_poses.append(Pose(
        position=start,
        orientation=overhead_orientation))

    block_poses.append(Pose(
        position=Point(position['70'][0], position['70'][1], position['70'][2]),
        orientation=overhead_orientation))
        ##r0 in corect poistion

    block_poses.append(Pose(
        position=start,
        orientation=overhead_orientation))

    block_poses.append(Pose(
        position=Point(position['77'][0], position['77'][1], position['77'][2]),
        orientation=overhead_orientation)) 
    #
    #
    #
    #
    #####BOARD IS NOW SET
    #
    #
    #
    #

    #WHITE ROOK TAKES BLACK ROOK
    block_poses.append(Pose(
    position=Point(position['00'][0], position['00'][1], position['00'][2]),
    orientation=overhead_orientation))

    block_poses.append(Pose(
        position=Point(position['70'][0], position['70'][1], position['70'][2]),
        orientation=overhead_orientation))

    # Move to the desired starting angles
    pnp.move_to_start(starting_pose)
    idx = 0
    

        #####White Pieces
        
    #white bishop
    print("\nSpawning white bishop...")
    load_new("b.sdf", "b2")
    print("\nPicking...")
    pnp.pick(block_poses[idx])
    print("\nTo Wait")
    pnp.wait()
    idx = (idx+1)
    print("\nPlacing...")
    pnp.place(block_poses[idx])
    pnp.wait()

    #white bishop
    idx = (idx+1)
    print("\nSpawning white bishop...")
    load_new("b.sdf", "b5")
    print("\nPicking...")
    pnp.pick(block_poses[idx])
    print("\nTo Wait")
    pnp.wait()
    print("\nPlacing...")
    idx = (idx+1)
    pnp.place(block_poses[idx])
    pnp.wait()

    #white rook
    idx = (idx+1)
    print("\nSpawning white rook...")
    load_new("r.sdf", "r0")
    print("\nPicking...")
    pnp.pick(block_poses[idx])
    print("\nTo Wait")
    pnp.wait()
    print("\nPlacing...")
    idx = (idx+1)
    pnp.place(block_poses[idx])
    pnp.wait()

    #white rook
    idx = (idx+1)
    print("\nSpawning white rook...")
    load_new("r.sdf", "r7")
    print("\nPicking...")
    pnp.pick(block_poses[idx])
    print("\nTo Wait")
    pnp.wait()
    print("\nPlacing...")
    idx = (idx+1)
    pnp.place(block_poses[idx])
    pnp.wait()

    #####Black pieces

    #black bishop
    idx = (idx+1)
    print("\nSpawning black bishop...")
    load_new("B.sdf", "B2")
    print("\nPicking...")
    pnp.pick(block_poses[idx])
    print("\nTo Wait")
    pnp.wait()
    idx = (idx+1)
    print("\nPlacing...")
    pnp.place(block_poses[idx])
    pnp.wait()

    #black bishop
    idx = (idx+1)
    print("\nSpawning black bishop...")
    load_new("B.sdf", "B5")
    print("\nPicking...")
    pnp.pick(block_poses[idx])
    print("\nTo Wait")
    pnp.wait()
    print("\nPlacing...")
    idx = (idx+1)
    pnp.place(block_poses[idx])
    pnp.wait()

    #black rook
    idx = (idx+1)
    print("\nSpawning black rook...")
    load_new("R.sdf", "R0")
    print("\nPicking...")
    pnp.pick(block_poses[idx])
    print("\nTo Wait")
    pnp.wait()
    print("\nPlacing...")
    idx = (idx+1)
    pnp.place(block_poses[idx])
    pnp.wait()

    #black rook
    idx = (idx+1)
    print("\nSpawning Black rook...")
    load_new("R.sdf", "R7", )
    print("\nPicking...")
    pnp.pick(block_poses[idx])
    print("\nTo Wait")
    pnp.wait()
    print("\nPlacing...")
    idx = (idx+1)
    pnp.place(block_poses[idx])
    pnp.wait()

    #wHITE ROOK TO BLACK ROOK
    idx = (idx+1)
    print("\nSpawning Black rook...")
    load_new("R.sdf", "R7")
    print("\nPicking...")
    pnp.pick(block_poses[idx])
    print("\nTo Wait")
    pnp.wait()
    print("\nPlacing...")
    idx = (idx+1)
    pnp.place(block_poses[idx])
    pnp.wait()

if __name__ == '__main__':
    sys.exit(main())
