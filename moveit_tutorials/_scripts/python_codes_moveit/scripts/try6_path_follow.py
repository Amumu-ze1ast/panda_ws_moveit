#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import copy

group = moveit_commander.MoveGroupCommander("panda_arm")  # Replace with your arm's planning group name

# Initialize ROS node
rospy.init_node('moveit_arm_line_demo', anonymous=True)
robot = moveit_commander.RobotCommander()
wpose = group.get_current_pose().pose

def move_arm_along_line(line_end, num_points):

    # Define waypoints for the straight line
    waypoints = []
    for i in range(num_points + 1):
        fraction = float(i) / num_points
        point = [a + (b - a) * fraction for a, b in zip([x_old, y_old, z_old], line_end)]
        waypoints.append(point)

    # waypoints = []
    # waypoints.append(copy.deepcopy(wpose))

    # Set the arm's starting position
    group.set_start_state_to_current_state()

    # Create a trajectory using the defined waypoints
    (plan, fraction) = group.compute_cartesian_path(
        waypoints=[geometry_msgs.msg.Pose(geometry_msgs.msg.Point(*waypoint), geometry_msgs.msg.Quaternion()) for waypoint in waypoints],
        eef_step=0.01,        # End effector step
        jump_threshold=0.0    # Jump threshold
    )


    # Execute the planned trajectory
    if fraction == 1.0:
        print("Path computed successfully!")
        group.execute(plan, wait=True)
    else:
        print("Path planning failed!")

if __name__ == '__main__':
    # Define the start and end points of the straight line

    x_old = wpose.position.x
    y_old = wpose.position.y
    z_old = wpose.position.z

    x_new = x_old 
    y_new = y_old + 0.1
    z_new = z_old

    start_point = [x_old,y_old,z_old]    # (x, y, z) coordinates of the starting point
    end_point = [x_new, y_new, z_new]      # (x, y, z) coordinates of the ending point
    num_points = 50

    # Initialize MoveIt and move the arm along the straight line
    move_arm_along_line(end_point, num_points)
