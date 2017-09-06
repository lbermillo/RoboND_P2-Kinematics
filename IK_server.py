#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        ### Your FK code here
        # Create symbols
        r, p, y = symbols('r p y')

        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # Create Modified DH parameters
        s = {alpha0:       0,   a0:      0,     d1:  0.75,
             alpha1: -pi / 2,   a1:   0.35,     d2:     0,  q2: q2 - pi / 2,
             alpha2:       0,   a2:   1.25,     d3:     0,
             alpha3: -pi / 2,   a3: -0.054,     d4:   1.5,
             alpha4:  pi / 2,   a4:      0,     d5:     0,
             alpha5: -pi / 2,   a5:      0,     d6:     0,
             alpha6:       0,   a6:      0,     d7: 0.303,  q7:            0}

        # Define Modified DH Transformation matrix
        def TF_Matrix(alpha, a, d, q):
            return Matrix([[cos(q), -sin(q), 0, a],
                           [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                           [sin(q) * sin(alpha), cos(q) * sin(alpha),  cos(alpha),  cos(alpha) * d],
                           [0, 0, 0, 1]])

        # Create individual transformation matrices
        T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(s)
        T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(s)
        T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(s)

        # Create generic rotation matrices
        R_x = Matrix([[1,      0,       0],
                      [0, cos(r), -sin(r)],
                      [0, sin(r),  cos(r)]])

        R_y = Matrix([[ cos(p), 0, sin(p)],
                      [      0, 1,      0],
                      [-sin(p), 0, cos(p)]])

        R_z = Matrix([[cos(y), -sin(y), 0],
                      [sin(y),  cos(y), 0],
                      [     0,       0, 1]])
        # Perform an extrinsic rotation to solve the generic EE rotations
        R_EE = R_z * R_y * R_x
        # Apply correction for difference in design
        R_Z_corr, R_Y_corr = R_z.subs(y, pi), R_y.subs(p, -pi/2)

        R_corr = R_Z_corr * R_Y_corr
        R_EE = R_EE * R_corr
        # Create a rotation from base to joint 3 to aid finding theta 4, 5, 6
        R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
            # roll, pitch, yaw = end-effector orientation
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
            # Sub in given values in our generic EE rotation matrix
            R_EE = R_EE.subs({'r': roll, 'p': pitch, 'y': yaw})
            # Initialize EE position
            EE = Matrix([[px],
                         [py],
                         [pz]])
            # Solve for WC position
            WC = EE - s[d7] * R_EE[:, 2]

            # Calculate joint angles using Geometric IK method
            #Calculate theta 1, 2, and 3
            theta1 = atan2(WC[1], WC[0])

            xc, yc = sqrt(WC[0] ** 2 + WC[1] ** 2) - s[a1], WC[2] - s[d1]

            # Get sides A, B, C; see diagram in writeup.md for reference
            side_A = sqrt(s[a3] ** 2 + s[d4] ** 2)      # distance from joint 3 to WC
            side_B = sqrt(xc ** 2 + yc ** 2)            # distance from joint 2 to WC
            side_C = s[a2]                              # distance from joint 2 to joint3

            # Solve for theta2; see diagram in writeup.md for reference
            # beta is the angle between side_C and the X-axis
            beta = atan2(yc, xc)
            # angle_a is retrieved using the Law of Cosine, this angle is between sides B and C
            angle_a = acos((side_B ** 2 + side_C ** 2 - side_A ** 2) / (2 * side_B * side_C))
            # Subtract 90 degrees from angle_a and beta and we get theta2
            theta2 = pi / 2 - angle_a - beta

            # Solve for theta3; see diagram in writeup.md for reference
            # gamma gives the angle between side_A and the link from 4 to WC
            gamma = atan2(s[a3], s[d4])
            # angle_b is retrieved using the Law of Cosine, this angle is between sides A and C
            angle_b = acos((side_A ** 2 + side_C ** 2 - side_B ** 2) / (2 * side_A * side_C))
            # By subtracting angle_b and gamma from 90 degrees we get theta3
            theta3 = pi / 2 - angle_b - gamma

            # Evaluate the generic rotation function from base to joint 3 with the calculated thetas
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            # Solve for rotations from joint 3 to joint 6
            R3_6 = R0_3.inv("LU") * R_EE
            # Calculate theta 4, 5, and 6 angles
            theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
            theta5 = atan2(sqrt(R3_6[2, 2]**2 + R3_6[0, 2]**2), R3_6[1, 2])
            theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
