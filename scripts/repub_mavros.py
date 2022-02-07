#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


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

    def repub(self, from_vslam):
        output = PoseStamped()

        pos = from_vslam.pose.pose.position

        array = np.array([pos.x, pos.y, pos.z])
        rot_matrice = np.array([[1,0,0], [0,-1,0], [0,0,-1]])
        rotated = np.dot(rot_matrice, array)

        output.header = from_vslam.header
        output.header.stamp = rospy.Time.now()

        output.pose.position.x = rotated[0]
        output.pose.position.y = rotated[1]
        output.pose.position.z = rotated[2]

        # output.pose.position = from_vslam.pose.pose.position

        self.pub.publish(output)


if __name__ == "__main__":
    rospy.init_node('translate_odom_to_pose')
    respublica = RepubMav()
    rospy.spin()
