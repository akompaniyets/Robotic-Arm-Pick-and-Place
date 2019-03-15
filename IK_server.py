  GNU nano 2.5.3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             File: IK_server.py                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                

#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import math
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #Used to represent thetas
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') #Used to represent the twist angles
        r, p, y = symbols('r p y') #Used to represent roll, pitch, and yaw
        #
        # Create Modified DH parameters
        # The DH parameters are filled in with the values explained in the write-up;
        # the distances 'a' and 'd' are filled in using the URDF file, the alphas
        # are calculated using the right-hand rule, and a modification to theta2 is
        # made with the starting -90 degree deficit.
        s = {alpha0:     0,   a0:       0,  d1: 0.75,   q1: q1,
             alpha1: -pi/2,   a1:    0.35,  d2:    0,   q2: q2 - pi/2,
             alpha2:     0,   a2:    1.25,  d3:    0,   q3: q3,
             alpha3: -pi/2,   a3:  -0.054,  d4: 1.50,   q4: q4,
             alpha4:  pi/2,   a4:       0,  d5:    0,   q5: q5,
             alpha5: -pi/2,   a5:       0,  d6:    0,   q6: q6,
             alpha6:     0,   a6:       0,  d7: 0.303,   q7: 0}
        #
        # Define Modified DH Transformation matrix
        # Each transformation matrix is defined as explained in the lessons provided, 
        # substituting the variables with each DH parameter above.
        T0_1 = Matrix([[             cos(q1),            -sin(q1),             0,               a0],
               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0),  -sin(alpha0),  -sin(alpha0)*d1],
               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),   cos(alpha0),   cos(alpha0)*d1],
               [                   0,                   0,             0,                1]])
        T0_1 = T0_1.subs(s)

        T1_2 = Matrix([[             cos(q2),            -sin(q2),             0,               a1],
               [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1),  -sin(alpha1),  -sin(alpha1)*d2],
               [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),   cos(alpha1),   cos(alpha1)*d2],
               [                   0,                   0,             0,                1]])
        T1_2 = T1_2.subs(s)

        T2_3 = Matrix([[             cos(q3),            -sin(q3),             0,               a2],
               [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2),  -sin(alpha2),  -sin(alpha2)*d3],
               [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),   cos(alpha2),   cos(alpha2)*d3],
               [                   0,                   0,             0,                1]])
        T2_3 = T2_3.subs(s)

        T3_4 = Matrix([[             cos(q4),            -sin(q4),             0,               a3],
               [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3),  -sin(alpha3),  -sin(alpha3)*d4],
               [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),   cos(alpha3),   cos(alpha3)*d4],
               [                   0,                   0,             0,                1]])
        T3_4 = T3_4.subs(s)

        T4_5 = Matrix([[             cos(q5),            -sin(q5),             0,               a4],
               [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4),  -sin(alpha4),  -sin(alpha4)*d5],
               [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),   cos(alpha4),   cos(alpha4)*d5],
               [                   0,                   0,             0,                1]])
        T4_5 = T4_5.subs(s)

        T5_6 = Matrix([[             cos(q6),            -sin(q6),             0,               a5],
               [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5),  -sin(alpha5),  -sin(alpha5)*d6],
               [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),   cos(alpha5),   cos(alpha5)*d6],
               [                   0,                   0,             0,                1]])
        T5_6 = T5_6.subs(s)

        T6_G = Matrix([[             cos(q7),            -sin(q7),             0,               a6],
               [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6),  -sin(alpha6),  -sin(alpha6)*d7],
               [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),   cos(alpha6),   cos(alpha6)*d7],
               [                   0,                   0,             0,                1]])
        T6_G = T6_G.subs(s)
        #
        # Create individual transformation matrices
        # In order to attain T0_G, each of the matrices above is post-multiplied.
        T0_2 = simplify(T0_1 * T1_2)
        T0_3 = simplify(T0_2 * T2_3)
        T0_4 = simplify(T0_3 * T3_4)
        T0_5 = simplify(T0_4 * T4_5)
        T0_6 = simplify(T0_5 * T5_6)
        T0_G = simplify(T0_6 * T6_G)

        #Next, a correctional matrix is attained (4X4 in size to match the transformation
        # matrices above. This correcional matrix is the 180 degree Z-axis rotation and 
        # -90 degree Y-axis rotation, also post-multiplied.
        R_z2 = Matrix([[   cos(np.pi),  -sin(np.pi),            0, 0],
              [   sin(np.pi),   cos(np.pi),            0, 0],
              [            0,            0,            1, 0],
               [0,0,0,1]    ])

        R_y2 = Matrix([[ cos(-np.pi/2),           0, sin(-np.pi/2), 0],
              [             0,           1,             0, 0],
              [-sin(-np.pi/2),           0, cos(-np.pi/2), 0],
              [0,0,0,1]     ])

        R_corr2 = simplify(R_z2 * R_y2)

        T_total = simplify(T0_G * R_corr2)

        # Extract rotation matrices from the transformation matrices;
        # This is simply the top left 3X3 matrix from each transformation matrix above.
        R0_1 = Matrix([[             cos(q1),            -sin(q1),             0],
               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0),  -sin(alpha0)],
               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),   cos(alpha0)]])
        R0_1 = R0_1.subs(s)

        R1_2 = Matrix([[             cos(q2),            -sin(q2),             0],
               [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1),  -sin(alpha1)],
               [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),   cos(alpha1)]])
        R1_2 = R1_2.subs(s)

        R2_3 = Matrix([[             cos(q3),            -sin(q3),             0],
               [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2),  -sin(alpha2)],
               [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),   cos(alpha2)]])
        R2_3 = R2_3.subs(s)

        R3_4 = Matrix([[             cos(q4),            -sin(q4),             0],
               [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3),  -sin(alpha3)],
               [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),   cos(alpha3)]])
        #R3_4 = R3_4.subs(s)

        R4_5 = Matrix([[             cos(q5),            -sin(q5),             0],
               [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4),  -sin(alpha4)],
               [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),   cos(alpha4)]])
        #R4_5 = R4_5.subs(s)

        R5_6 = Matrix([[             cos(q6),            -sin(q6),             0],
               [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5),  -sin(alpha5)],
               [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),   cos(alpha5)]])
        #R5_6 = R5_6.subs(s)

        R6_G = Matrix([[             cos(q7),            -sin(q7),             0],
               [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6),  -sin(alpha6)],
               [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),   cos(alpha6)]])
        #R6_G = R6_G.subs(s)
        #

        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            p_x = req.poses[x].position.x
            p_y = req.poses[x].position.y
            p_z = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            # The correctional rotation matrix here is identical in its rotations
            # as that used for forward kinematics, except it is 3X3 this time to 
            # match the rotational matrices used in the inverse kinematics problem.
            R_z = Matrix([[   cos(np.pi),  -sin(np.pi),            0],
              [   sin(np.pi),   cos(np.pi),            0],
              [            0,            0,            1]])

            R_y = Matrix([[ cos(-np.pi/2),           0, sin(-np.pi/2)],
              [             0,           1,             0],
              [-sin(-np.pi/2),           0, cos(-np.pi/2)]])

            R_corr = simplify(R_z * R_y)
            #
            # Calculate joint angles using Geometric IK method
            #Determining wrist center (WC) position
            d = 0.303 #The z-axis distance from the end-effector to the wrist center.

            Rot_Z = Matrix([[ cos(y), -sin(y),     0],
                [ sin(y),  cos(y),     0],
                [        0,         0,     1]])

            Rot_Y = Matrix([[ cos(p),       0, sin(p)],
                [          0,       1,          0],
                [-sin(p),       0, cos(p)]])

            Rot_X = Matrix([[          1,         0,          0],
                [          0, cos(r), -sin(r)],
                [          0, sin(r),  cos(r)]])

            #The complete rotational matrix from the floor to the end effector
            #can be discovered by pre-multiplying (extrinsic rotation) the roll (x),
            # pitch(y), and yaw(z) values attained above. The correctional matrix is
            # then post-multiplied to correct for the discrepancy between DH and URDF
            # parameters.
            R_rpy = Rot_Z * Rot_Y * Rot_X * R_corr

            R_rpy = R_rpy.subs({r: roll, p: pitch, y: yaw})

            # The third column of the total rotation matrix represents the coordinates
            # of the vector which points to the new z-axis of the end-effector.
            n_x = R_rpy[0,2]
            n_y = R_rpy[1,2]
            n_z = R_rpy[2,2]

            # Now, the x,y,z components of 'd' can be subtracted from the end-effector
            # position attained above to derive the wrist center position.
            w_x = p_x - ((d)*n_x)
            w_y = p_y - ((d)*n_y)
            w_z = p_z - ((d)*n_z)

            #First theta, derived as described in the write-up diagrams:
            theta1 = math.atan2(w_y, w_x)

            #Second and third thetas:
            #First, side 'B' of the derived triangle (as in the write-up diagrams) is calculated:
            B = sqrt(((sqrt((w_x*w_x)+(w_y*w_y))-0.35)*(sqrt((w_x*w_x)+(w_y*w_y))-0.35)) + ((w_z-0.75)*(w_z-0.75)))

            #Then, angles 'o_w' and 'a' are calculated, also following the diagram in the write-up:
            o_w = atan2((w_z-0.75),((sqrt((w_x*w_x)+(w_y*w_y))-0.35)))

            a = acos(((1.25*1.25)+(B * B)-(1.5*1.5))/(2*1.25*B))

            #This allows theta2 to be calculated following the write-up:
            theta2 = (np.pi/2) - o_w - a

            #'o_v' and 'c' were calculated, but proved to not be necessary for calculating the
            # necessary variables.
            #o_v = atan2((sqrt((w_x*w_x)+(w_y*w_y))-0.35),(w_z-0.75))
            #c = acos(((1.5*1.5)+(B*B)-(1.25*1.25))/(2*1.5*B))

            #Next, angle 'b' is calculated, following the write-up diagram:
            b = acos(((1.25*1.25)+(1.5*1.5)-(B*B))/(2*1.25*1.5))

            #After this, theta3 can be calculated (using the additional variable 'v', calculated in
            # the write-up. It is simply written as 0.03599 here, since this variable value remains
            # constant.
            theta3 = (np.pi/2) - (b + 0.03599)

            #Figuring out R3_6
            #As described in the write-up, R0_3 is figured out first, which is the post-multiplicaton of
            # R0_1, R1_2, and R2_3. Afterwards, the newly calculated theta values are inserted into the 
            # R0_3 matrix.
            R0_3 = R0_1 * R1_2 * R2_3
            R0_3 = R0_3.evalf(subs = {q1: theta1, q2: theta2, q3: theta3})

            #As described in the lessons provided, R3_6 is the inverse of R0_3 multiplied by R_rpy:
            R3_6 = R0_3.inv("LU") * R_rpy


            #Following the rotation matrix derived in the write-up, each theta is calculated appropriately:
            theta4 = math.atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt((R3_6[0,2]*R3_6[0,2])+(R3_6[2,2] * R3_6[2,2])), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])


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






