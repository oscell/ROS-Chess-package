#!/usr/bin/env python  
import rospy
import math

# Importing TF to facilitate the task of receiving transformations
import tf2_ros

# Import quaternions and numpy
from transforms3d import quaternions
import numpy

if __name__ == '__main__':
    rospy.init_node('baxter_tf_listener')

    # This creates a transform listener object; once created, it starts receiving
    # transformations using the /tf topic and buffers them up for up to 10 seconds.
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # This is where the magic happens, a query is passed to the listener for the
    # /base to /head transform by means of the lookupTransform fn. The arguments are
    # from "this frame" to "this frame" at "this specific time"
    # (if you pass "rospy.Time(0), the fn will give you the latest available transform 
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            transformation = tfBuffer.lookup_transform('base', 'head', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        rospy.loginfo("Translation: \n" + str(transformation.transform.translation))
        rospy.loginfo("Quaternion: \n" + str(transformation.transform.rotation))

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
        rate.sleep()