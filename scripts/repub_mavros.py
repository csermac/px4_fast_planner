#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf


class RepubMav():
    """
    get the message from openvslam and publish it to /mavros/mocap/pose after
    translation
    """

    def __init__(self):
        self.pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)

        # initialize boolean variable that registers is the sought topic has 
        # been found
        found = False
        # scan for any of the two topics published by opevnslam
        while found == False:
            topics = rospy.get_published_topics()
            for topic, taip in topics:
                if topic == '/run_localization/camera_pose' or topic == '/run_slam/camera_pose':
                    self.sub = rospy.Subscriber( topic, Odometry, self.repub, queue_size=1)
                    rospy.loginfo("found topic %s to translate",topic )
                    found = True
                else:
                    #rospy.loginfo("topics not found, yet")
                    rospy.sleep(0.1)

        self.sub = rospy.Subscriber('/run_localization/camera_pose', Odometry, self.repub, queue_size=1)
        rospy.loginfo("subscriber to found topic created")

    def repub(self, from_vslam: Odometry):
        output = PoseStamped()

        tf.TransformBroadcaster.sendTransform(from_vslam.pose.pose.position, from_vslam.pose.pose.orientation, rospy.Time.now(), 'slam_pose', 'map')

        output.header = from_vslam.header
        output.header.stamp = rospy.Time.now()
        output.pose = from_vslam.pose.pose
        self.pub.publish(output)


if __name__ == "__main__":
    rospy.init_node('translate_odom_to_pose')
    respublica = RepubMav()
    rospy.spin()
