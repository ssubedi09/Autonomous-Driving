#!/usr/bin/env python3

import rospy
import yaml
from geometry_msgs.msg import PoseStamped

def read_waypoints_from_yaml(file_path):
    with open(file_path, 'r') as yaml_file:
        waypoints_data = yaml.safe_load(yaml_file)
        return waypoints_data['waypoints']

def publish_waypoints(waypoints):
    rospy.init_node('waypoint_publisher', anonymous=True)
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz
    count = 0
    for waypoint in waypoints:
        pose = PoseStamped()
        pose.header.seq = count
        count+=1
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = waypoint['pose']['position']['x']
        pose.pose.position.y = waypoint['pose']['position']['y']
        pose.pose.position.z = waypoint['pose']['position']['z']
        pose.pose.orientation.x = waypoint['pose']['orientation']['x']
        pose.pose.orientation.y = waypoint['pose']['orientation']['y']
        pose.pose.orientation.z = waypoint['pose']['orientation']['z']
        pose.pose.orientation.w = waypoint['pose']['orientation']['w']

        pub.publish(pose)
        rospy.loginfo("Waypoint published: {}".format(waypoint))
        rate.sleep()

if __name__ == '__main__':
    try:
        waypoints = read_waypoints_from_yaml('../../config/final_proj_waypoints.yaml')
        publish_waypoints(waypoints)
    except rospy.ROSInterruptException:
        pass
