#!/usr/bin/env python3
import sys
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped

# Parameters
files_path = "/home/ali"
# Such as orb3, vsgraphs, uco, suco (semantic UcoSLAM)
slam_method = "vsgraphs"
dataset_seq = "seq01"  # The dataset sequence running
gt_pose_topic = "/gazebo/model_states"
slam_pose_topic = "/orb_slam3/camera_pose"

# Creating a txt file that will contain poses
print("Creating txt files for adding SLAM and Ground-Truth poses ...")
gt_pose_file = open(
    f"{files_path}/gt_pose_{slam_method}_{dataset_seq}.txt", "w+")
gt_pose_file.write("#timestamp tx ty tz qx qy qz qw\n")
slam_pose_file = open(
    f"{files_path}/slam_pose_{slam_method}_{dataset_seq}.txt", "w+")
slam_pose_file.write("#timestamp tx ty tz qx qy qz qw\n")

# Check what to set for the robot-ID
robot_id = -1
for i in range(1, len(sys.argv)):
    if i == 1:
        robot_id = sys.argv[i]
    elif i == 2:
        gt_pose_topic = sys.argv[i]


def groundtruthPoseCallback(groundtruth_pose_msg):
    # Calculate different variables
    time = rospy.get_rostime().to_sec()
    tx = groundtruth_pose_msg.pose[int(robot_id)].position.x
    ty = groundtruth_pose_msg.pose[int(robot_id)].position.y
    tz = groundtruth_pose_msg.pose[int(robot_id)].position.z
    rx = groundtruth_pose_msg.pose[int(robot_id)].orientation.x
    ry = groundtruth_pose_msg.pose[int(robot_id)].orientation.y
    rz = groundtruth_pose_msg.pose[int(robot_id)].orientation.z
    rw = groundtruth_pose_msg.pose[int(robot_id)].orientation.w
    # Write to the Ground-Truth file
    gt_pose_file.write(
        str(time) + " " + str(tx) + " " + str(ty) + " "
        + str(tz) + " " + str(rx) + " " + str(ry) + " "
        + str(rz) + " " + str(rw) + "\n"
    )


def slamPoseCallback(slam_pose_msg):
    # Calculate different variables
    time = slam_pose_msg.header.stamp.to_sec()
    odom_x = slam_pose_msg.pose.position.x
    odom_y = slam_pose_msg.pose.position.y
    odom_z = slam_pose_msg.pose.position.z
    odom_rx = slam_pose_msg.pose.orientation.x
    odom_ry = slam_pose_msg.pose.orientation.y
    odom_rz = slam_pose_msg.pose.orientation.z
    odom_rw = slam_pose_msg.pose.orientation.w
    # Write to the SLAM file
    slam_pose_file.write(
        str(time) + " " + str(odom_x) + " " + str(odom_y) + " "
        + str(odom_z) + " " + str(odom_rx) + " " + str(odom_ry) + " "
        + str(odom_rz) + " " + str(odom_rw) + "\n"
    )


def subscribers():
    rospy.init_node("text_file_generator", anonymous=True)
    # Subscriber to the ground-truth topic
    rospy.Subscriber(gt_pose_topic,
                     ModelStates, groundtruthPoseCallback)
    # Subscriber to the SLAM topic
    rospy.Subscriber(slam_pose_topic, PoseStamped, slamPoseCallback)
    rospy.spin()


if __name__ == "__main__":
    subscribers()
