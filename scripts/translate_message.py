#!/usr/bin/env python


import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class Translator():
    def __init__(self):
        self.sub_camera = rospy.Subscriber('/run_slam/camera_pose', Odometry, self.translation, queue_size=1)
        self.pub_mavros = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
        self.output = PoseStamped()

    def translation(self, mes_from_slam):
        self.output.header = mes_from_slam.header
        self.output.pose = mes_from_slam.pose.pose
        self.pub_mavros.publish(self.output)
        rospy.loginfo("message translated and published")


if __name__ == '__main__':
    rospy.init_node('message_translation')
    translator = Translator()

    # eventually scan until either /run_localization/ camera_pose or /run_slam/camera_pose
    # shows up and set up the subscriber to get from the existing topic of the two
    # while found == False:
    #     rospy.get_published_topics()

    rospy.spin()
