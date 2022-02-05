#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class RepubMav():
    """
    get the message from openvslam and publish it to /mavros/mocap/pose after
    translation
    """

    def __init__(self):
        self.pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)

        # scan for any of the two topics published by opevnslam
        while True:
            topics = rospy.get_published_topics()
            for topic, taip in topics:
                if topic == '/run_localization/camera_pose' or topic == '/run_slam/camera_pose':
                    self.sub = rospy.Subscriber(
                        topic, Odometry, self.repub, queue_size=1)
                    rospy.loginfo("found topic to translate")
                    break
                else:
                    rospy.loginfo("topics not found, yet")
            rospy.sleep(0.1)

        self.sub = rospy.Subscriber(
            '/run_localization/camera_pose', Odometry, self.repub, queue_size=1)

    def repub(self, from_vslam: Odometry):
        output = PoseStamped()

        output.header = from_vslam.header
        output.header.stamp = rospy.Time.now()
        output.pose = from_vslam.pose.pose
        self.pub.publish(output)
        rospy.loginfo("translated message has been published")


if __name__ == "__main__":
    rospy.init_node('translate_odom_to_pose')
    respublica = RepubMav()
    rospy.spin()
