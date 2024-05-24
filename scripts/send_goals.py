#!/usr/bin/env python3

import rospy
import random
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

class GoalPublisher:
    def __init__(self):
        rospy.init_node('send_goals', anonymous=True)
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.goal_callback)
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.marker_pub = rospy.Publisher('/goal_marker', Marker, queue_size=1) 
        self.rate = rospy.Rate(10)  # 10hz
        
        self.map_data = None
        self.map_resolution = None
        self.map_width = None
        self.map_height = None
        self.map_origin = None
        self.goal_status = None

        rospy.loginfo("Waiting for map data...")
        while self.map_data is None:
            rospy.sleep(1)

    def map_callback(self, data):
        self.map_data = data.data
        self.map_resolution = data.info.resolution
        self.map_width = data.info.width
        self.map_height = data.info.height
        self.map_origin = data.info.origin

    def is_free_space(self, x, y):
        """Check if the position (x, y) is free space."""
        mx = int((x - self.map_origin.position.x) / self.map_resolution)
        my = int((y - self.map_origin.position.y) / self.map_resolution)

        if 0 <= mx < self.map_width and 0 <= my < self.map_height:
            index = my * self.map_width + mx
            return self.map_data[index] == 0
        return False

    def get_random_pose(self):
        while True:
            x = random.uniform(self.map_origin.position.x, self.map_origin.position.x + self.map_width * self.map_resolution)
            y = random.uniform(self.map_origin.position.y, self.map_origin.position.y + self.map_height * self.map_resolution)

            if self.is_free_space(x, y):
                orientation = random.uniform(-3.14, 3.14)

                goal_msg = PoseStamped()
                goal_msg.header.frame_id = "map"
                goal_msg.pose.position.x = x
                goal_msg.pose.position.y = y
                goal_msg.pose.position.z = 0.0
                goal_msg.pose.orientation.w = orientation
                self.publish_goal_marker(x, y)

                return goal_msg
                

    def goal_callback(self, data):
        if data.status_list:
            self.goal_status = data.status_list[-1].status

    def publish_goal(self):
        while not rospy.is_shutdown():
            random_pose = self.get_random_pose()
            rospy.loginfo("Sending goal: %s", random_pose)
            self.pub.publish(random_pose)
            
            # Wait for goal to complete
            while self.goal_status != 3:  # 3 corresponds to goal succeeded
                rospy.loginfo("Executing goal!")
                rospy.sleep(0.5)
            rospy.loginfo("Goal completed successfully!")
            rospy.sleep(10)
            self.goal_status = None

            self.rate.sleep()


    def publish_goal_marker(self, x, y):
        # Create marker message
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1  # Lift the marker slightly above the ground
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0  # Red color
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Publish the marker
        self.marker_pub.publish(marker)

if __name__ == '__main__':
    try:
        goal_publisher = GoalPublisher()
        goal_publisher.publish_goal()
    except rospy.ROSInterruptException:
        pass
