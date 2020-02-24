#!/usr/bin/env python

import rospy
import crazyflie
import time
import uav_trajectory

if __name__ == '__main__':
    rospy.init_node('test_high_level')

    cf = crazyflie.Crazyflie("crazyflie", "/vicon/crazyflie/crazyflie")

    # update all relevant parametes before flight.


    # enable high-level controller

    cf.setParam("commander/enHighLevel", 1)


    # reset kalman
    cf.setParam("kalman/resetEstimation", 1)


    # wait for kalman convergance


    # takeoff

    cf.takeoff(targetHeight = 1, duration = 3)
    time.sleep(4.0)
    

    # wait for valid position from SLAM

    # fly to point
    cf.goTo(goal = [1, 0.0, 0.0], yaw=0, duration = 3.0, relative = True)
    time.sleep(4.0)

    # land
    # cf.land(targetHeight = 0.0, duration = 2.0)

    cf.stop()
    cf.setParam("commander/enHighLevel", 0)
