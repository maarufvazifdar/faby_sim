#!/usr/bin/env python

# Author: Mohammed Maaruf Vazifdar

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
import Tkinter
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys
import numpy as np
import math

goals1 = [[3.4, 3.4, -2], [5, 1.2, -2.5], [5.6, 4, -1], [5, 4.8, 1.8]]
goals2 = [[3.4, 7, -2], [5, 5.4, -2.5], [5.6, 7, -1]]
goals3 = [[3.4, 9, -2], [5, 9.5, 1.8], [5.6, 9, 0]]
goals4 = [[6.4, 3.4, -2], [7.8, 1.2, -2.5], [8.6, 4, -1], [7.8, 4.8, 1.8]]
goals5 = [[6.4, 7, -2], [7.8, 5.4, -2.5], [8.6, 7, -1]]
goals6 = [[6.4, 9, -2], [7.8, 9.5, 1.8], [8.6, 9, 0]]


def calc_distance(goals, x_current, y_current):
    global index
    index = 0
    distance = []
    for i in range(len(goals)):
        distance.append(math.sqrt(
            (goals[i][0] - x_current) ** 2 +
            (goals[i][1] - y_current) ** 2))
    index = distance.index(min(distance))
    return index


def pose_callback(msg):
    global robot_position
    global robot_quat
    global robot_euler_angle

    robot_position = [
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z]
    robot_quat = [
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w]
    robot_euler_angle = euler_from_quaternion(robot_quat)


def Button():
    top = Tkinter.Tk()
    B0 = Tkinter.Button(top, text="Origin", command=Origin)
    B0.pack()
    B1 = Tkinter.Button(top, text="Table1", command=Table1)
    B1.pack()
    B2 = Tkinter.Button(top, text="Table2", command=Table2)
    B2.pack()
    B3 = Tkinter.Button(top, text="Table3", command=Table3)
    B3.pack()
    B4 = Tkinter.Button(top, text="Table4", command=Table4)
    B4.pack()
    B5 = Tkinter.Button(top, text="Table5", command=Table5)
    B5.pack()
    B6 = Tkinter.Button(top, text="Table6", command=Table6)
    B6.pack()
    B7 = Tkinter.Button(top, text="Tool Room", command=Tool_Room)
    B7.pack()
    B8 = Tkinter.Button(top, text="Inventory Room", command=Inventory_Room)
    B8.pack()
    B9 = Tkinter.Button(top, text="Welding Room", command=Welding_Room)
    B9.pack()
    top.mainloop()


def Origin():
    nav_goal_py(0, 0, 0)
    print("Reached Origin")


def Table1():
    calc_distance(goals1, robot_position[0], robot_position[1])
    nav_goal_py(goals1[index][0], goals1[index][1], goals1[index][2])
    print("Reached Table1")


def Table2():
    calc_distance(goals2, robot_position[0], robot_position[1])
    nav_goal_py(goals2[index][0], goals2[index][1], goals2[index][2])
    print("Reached Table2")


def Table3():
    calc_distance(goals3, robot_position[0], robot_position[1])
    nav_goal_py(goals3[index][0], goals3[index][1], goals3[index][2])
    print("Reached Table3")


def Table4():
    calc_distance(goals4, robot_position[0], robot_position[1])
    nav_goal_py(goals4[index][0], goals4[index][1], goals4[index][2])
    print("Reached Table4")


def Table5():
    calc_distance(goals5, robot_position[0], robot_position[1])
    nav_goal_py(goals5[index][0], goals5[index][1], goals5[index][2])
    print("Reached Table5")


def Table6():
    calc_distance(goals6, robot_position[0], robot_position[1])
    nav_goal_py(goals6[index][0], goals6[index][1], goals6[index][2])
    print("Reached Table6")


def Inventory_Room():
    nav_goal_py(7.5, 0.2, 2.2)
    print("Reached Inventory Room")


def Tool_Room():
    nav_goal_py(11.8, 0.2, 2.2)
    print("Reached Tool Room")


def Welding_Room():
    nav_goal_py(10.5, 14, -1.5)
    print("Reached Welding Room")


def nav_goal_py(x, y, yaw):
    # Create an action client called "move_base" with action definition file
    # "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # Waits until the action server has started up and started listening for
    # goals.
    client.wait_for_server()
    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.header.stamp = rospy.Time.now()
    # add goal pose in terms of x,y (in metrers) and yaw (in raidans)
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    # to send orientation with a yaw we need quaternion transform
    X, Y, Z, W = quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = X
    goal.target_pose.pose.orientation.y = Y
    goal.target_pose.pose.orientation.z = Z
    goal.target_pose.pose.orientation.w = W

    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
    result = client.get_result
    if result:
        print("Goal Reached")
        print(robot_position)


if __name__ == '__main__':
    try:
        # Initializes a rospy node to let the SimpleActionClient publish and
        # subscribe
        rospy.init_node('nav_goal_py')
        rospy.Subscriber('/odom', Odometry, pose_callback)
        Button()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
