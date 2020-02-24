#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PointStamped, PoseStamped
from crazyflie_driver.srv import UpdateParams, Takeoff, Land

def onNewTransform(transform):
    global msg
    global pub
    global firstTransform

    if firstTransform:
        # initialize kalman filter
        rospy.set_param("kalman/initialX", transform.pose.position.x)
        rospy.set_param("kalman/initialY", transform.pose.position.y)
        rospy.set_param("kalman/initialZ", transform.pose.position.z)
        update_params(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])

        rospy.set_param("kalman/resetEstimation", 1)
        # rospy.set_param("locSrv/extPosStdDev", 1e-4)
        update_params(["kalman/resetEstimation"]) #, "locSrv/extPosStdDev"])
        firstTransform = False
    else:
        msg.header.frame_id = transform.header.frame_id
        msg.header.stamp = transform.header.stamp
        msg.header.seq += 1

	# add transform between camera and cf system!?
        #rospy.loginfo("updated position with slam pose")

        msg.point.x = transform.pose.position.x
        msg.point.y = transform.pose.position.y
        msg.point.z = transform.pose.position.z
        pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('publish_external_position_vicon', anonymous=True)
    topic = rospy.get_param("~position_topic", "")

    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)

    firstTransform = True

    msg = PointStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()

    pub = rospy.Publisher("external_position", PointStamped, queue_size=1)
    rospy.Subscriber(topic, PoseStamped, onNewTransform)

    rospy.loginfo("subscribed to slam pose")

    rospy.spin()
