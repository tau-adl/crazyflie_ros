#!/usr/bin/env python

import rospy
import crazyflie
import time
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Float32
import tf
from tf import TransformListener
from math import sqrt
import subprocess
import os

### FOR PLOT ##
import numpy
import matplotlib.pyplot as plt
from scipy.stats import norm
import matplotlib.mlab as mlab
### --------- ###


cf_pose = 0
time_of_fist_slam_pose = 0 
numberOfPointsForScaleCalibration = 100
sendExternalPosition = False
scaleSet = False
cf_altitude = 0
takeoff_height = 1
pose_at_init = 0
startScaleCalibration = False
norms_cf = []
norms_slam = []
slam_scale = 0
cf_external_pos_msg = 0

cf_poses = []
slam_scaled_poses = []

def new_slam_pose(slam_pose):
    global firstSLAMPose
    global scaleSet
    global gotCFPose
    global cf_external_pos_publisher
    global time_of_fist_slam_pose
    global pose_at_init
    global tf_br    
    global tf_listener
    global norms_cf
    global norms_slam
    global numberOfPointsForScaleCalibration
    global startScaleCalibration
    global slam_scale
    global scale_mean_publisher
    global scale_stdev_publisher
    global cf_external_pos_msg
    
    if pose_at_init is not 0 and not scaleSet:
        t = rospy.Time.now()
        tf_br.sendTransform((pose_at_init.pose.position.x, pose_at_init.pose.position.y, pose_at_init.pose.position.z),
                        (pose_at_init.pose.orientation.x, pose_at_init.pose.orientation.y, pose_at_init.pose.orientation.z,pose_at_init.pose.orientation.w),
                        t,
                        "cf0",
                         "base")
        tf_br.sendTransform((0, 0, 0),
                           (-0.5, -0.5,  0.5,  0.5),
                           t,
                           "map",
                           "cf0")
        
    #print "published base->cf0"
    if firstSLAMPose:      
        rospy.loginfo("fist SLAM pose received!")

        if gotCFPose:
            time_of_fist_slam_pose = rospy.Time.now()
            pose_at_init = PoseStamped() #cf_pose    
            pose_at_init.pose.orientation.w = 1
            pose_at_init.pose.position.z = cf_pose.pose.position.z
            pose_at_init = cf_pose
        else:
            # crash here?
            rospy.logerr("need CF pose before SLAM is initiallized!")
                        
        firstSLAMPose = False
        
    elif startScaleCalibration:
        if len(norms_cf) < numberOfPointsForScaleCalibration:            
            
            tf_listener.waitForTransform("/cf0", "/cf", rospy.Time.now() - rospy.Duration(0.06),rospy.Duration(1))
            tf_listener.waitForTransform("/cf0", "/camera_link", rospy.Time.now() - rospy.Duration(0.06),rospy.Duration(1))
            
            position_cf, _   =  tf_listener.lookupTransform("/cf0", "/cf", rospy.Time())
            position_slam, _ =  tf_listener.lookupTransform("/cf0", "/camera_link", rospy.Time())

            norms_cf.append(tf.transformations.vector_norm(position_cf))
            norms_slam.append(tf.transformations.vector_norm(position_slam))
            print str(tf.transformations.vector_norm(position_cf)) + " " +  str(tf.transformations.vector_norm(position_slam))            
            
            rospy.loginfo("added calibration point: "+str(len(norms_cf)))
                
        else:

            (mu_slam, sigma_slam) = norm.fit(norms_slam)
            (mu_cf, sigma_cf) = norm.fit(norms_cf)
            
            #PLOT###########
            
            num_bins = 100
            fig, ax = plt.subplots()            
            # the histogram of the data
            n, bins, patches = ax.hist([norms_cf,norms_slam], num_bins, label=['cf', 'slam'], normed=1)  
            y_cf = mlab.normpdf( bins, mu_cf, sigma_cf)
            y_slam = mlab.normpdf( bins, mu_slam, sigma_slam)
            l_cf = ax.plot(bins, y_cf, 'b--', linewidth=2)
            l_slam = ax.plot(bins, y_slam, 'r--', linewidth=2)
            plt.legend(loc='upper right')          
            plt.show() # remove and turn into service. -- add calibration service?
            ################
            
            # good calibration
            qf_cf = sigma_cf / mu_cf
            qf_slam = sigma_slam / mu_slam
            if (qf_cf < 0.1) and ( qf_slam< 0.1):
                slam_scale = mu_cf / mu_slam
                startScaleCalibration = False
                scaleSet = True
                rospy.loginfo("good calibration! qf_sc: "+str(qf_cf)+" qf_slam: "+str(qf_slam) + " scale: " + str(slam_scale))
            # bad calibration
            else:
                slam_scale = mu_cf / mu_slam
                startScaleCalibration = True
                scaleSet = False    
                rospy.logerr("bad calibration! f_sc: "+str(qf_cf)+" qf_slam: "+str(qf_slam) + " scale: " + str(slam_scale))
                norms_cf = []
                norms_slam = []
                rospy.loginfo("restarting calibration...")
                # crash here?             
    
    if scaleSet:
        t = rospy.Time.now()
        tf_br.sendTransform((slam_pose.pose.position.x*slam_scale, slam_pose.pose.position.y*slam_scale, slam_pose.pose.position.z*slam_scale),
                (slam_pose.pose.orientation.x, slam_pose.pose.orientation.y, slam_pose.pose.orientation.z,slam_pose.pose.orientation.w),
                t,
                "scaled_camera_link",
                 "map")
        tf_br.sendTransform((pose_at_init.pose.position.x, pose_at_init.pose.position.y, pose_at_init.pose.position.z),
                (pose_at_init.pose.orientation.x, pose_at_init.pose.orientation.y, pose_at_init.pose.orientation.z,pose_at_init.pose.orientation.w),
                t,
                "cf0",
                 "base")
        tf_br.sendTransform((0, 0, 0),
                (-0.5, -0.5,  0.5,  0.5),
                t,
                "map",
                 "cf0")
        
        tf_listener.waitForTransform("/base", "/cf", rospy.Time.now() - rospy.Duration(0.03),rospy.Duration(1))
        tf_listener.waitForTransform("/base", "/scaled_camera_link", rospy.Time.now() - rospy.Duration(0.03),rospy.Duration(1))        
        
        temp_position_cf, _   =  tf_listener.lookupTransform("/base", "/cf", rospy.Time())
        temp_position_slam, _ =  tf_listener.lookupTransform("/base", "/scaled_camera_link", rospy.Time())
        print "cf position:   " + str(temp_position_cf)
        print "slam position: " + str(temp_position_slam) + "\n"
        #print tf.transformations.vector_norm([x-y for (x,y) in zip(temp_position_cf,temp_position_slam)])    
        if sendExternalPosition:                        
            cf_external_pos_msg = PointStamped()
            cf_external_pos_msg.header.seq = 0
            cf_external_pos_msg.header.stamp = rospy.Time.now()
            cf_external_pos_msg.header.frame_id = "base"
            cf_external_pos_msg.header.stamp = slam_pose.header.stamp
            cf_external_pos_msg.header.seq += 1
            cf_external_pos_msg.point.x = temp_position_slam[0]
            cf_external_pos_msg.point.y = temp_position_slam[1]
            cf_external_pos_msg.point.z = temp_position_slam[2]
            cf_external_pos_publisher.publish(cf_external_pos_msg)
    return
        

def new_cf_pose(new_pose):
    global cf_pose
    global gotCFPose
    global tf_br
    global cf_altitude
    
    cf_pose = new_pose       
    gotCFPose = True
    tf_br.sendTransform((cf_pose.pose.position.x, cf_pose.pose.position.y, cf_pose.pose.position.z),
                             (cf_pose.pose.orientation.x, cf_pose.pose.orientation.y, cf_pose.pose.orientation.z,cf_pose.pose.orientation.w),
                             rospy.Time.now(),
                             "cf",
                             "base")         
    

if __name__ == '__main__':
    
    firstSLAMPose = True
    gotCFPose = False
    
    rospy.init_node('crazybal_slam_demo')
    
    rospy.loginfo("started commander")
    cf = crazyflie.Crazyflie("/crazyflie", "/cf")
    rospy.loginfo("started cf")
    
    # update all relevant parametes before flight?
        
    orbslam_pose_topic = rospy.get_param("~orbslam_pose_topic", "crazyflie/orb_slam2_mono/pose")
    crazy_flie_pose_topic = rospy.get_param("~crazy_flie_pose_topic", "/crazyflie/pose")

    tf_br = tf.TransformBroadcaster()
    tf_listener = TransformListener()
    # SUBSCRIBERS

    rospy.Subscriber(orbslam_pose_topic, PoseStamped, new_slam_pose)    
    rospy.Subscriber(crazy_flie_pose_topic, PoseStamped, new_cf_pose)
    
    # PUBLISHERS
    
    cf_external_pos_publisher = rospy.Publisher("external_position", PointStamped, queue_size=1)
    scale_mean_publisher = rospy.Publisher("scale_mean",Float32,queue_size = 10)
    scale_stdev_publisher = rospy.Publisher("scale_stdev",Float32,queue_size = 10)
    
    cf_external_pos_msg = PointStamped()
    cf_external_pos_msg.header.seq = 0
    cf_external_pos_msg.header.frame_id = "base"
    
    
    # enable high-level controller

    cf.setParam("commander/enHighLevel", 1)
    rospy.loginfo("enable high-level controller")

    # reset kalman
    cf.setParam("kalman/resetEstimation", 1)
    rospy.loginfo("reset kalman")

    # wait for kalman convergance
    rospy.sleep(5.)
    
    # wait until we have cf pose data
    rospy.loginfo("cf publishing pose from Kalman")
    #tf_listener.waitForTransform("/cf", "/base", rospy.Time.now(),rospy.Duration(5))
    tf_listener.waitForTransform("/cf", "/base", rospy.Time.now() + rospy.Duration(4),rospy.Duration(5))
    
    # takeoff

    rospy.loginfo("takeoff begin...")  
    #cf.takeoff(takeoff_height, duration = 3)
    rospy.sleep(5.)
    # kill running slam node and start it again.
    DEVNULL = open(os.devnull, 'wb')
    child = subprocess.Popen(["rosnode","kill","/crazyflie/orb_slam2_mono"],stdout=DEVNULL ,stderr=DEVNULL )
    child = subprocess.Popen(["roslaunch","orb_slam2_ros","orb_slam2_crazybal_mono.launch"],stdout=DEVNULL )
    
    rospy.sleep(5.)
    
    rospy.loginfo("takeoff complete!")        

    # wait untill we have slam for 2 seconds straight...
    #tf_listener.waitForTransform("/cf0", "/base", rospy.Time.now() ,rospy.Duration(10))
    tf_listener.waitForTransform("/cf0", "/base", rospy.Time.now() + rospy.Duration(4),rospy.Duration(5))
    # start scale calibration
    startScaleCalibration = True
    rospy.loginfo("starting scale calibration...")      
    #cf.goTo(goal = [0.0, 0.0, takeoff_height + 0.5], yaw=0, duration = 3.0, relative = True)
    
    # wait for scale set
    #rospy.spin() # hang here...
    rospy.sleep(10.)

    if not scaleSet:
        # bad calibration
        rospy.logerr("calibration failed...aborting")
        #cf.land(0,duration = 2)
        rospy.sleep(2.)
        cf.stop()
        cf.setParam("commander/enHighLevel", 0)
        rospy.spin()
        #quit() # crash here?
    else:
        rospy.loginfo("calibration succeeded!")
    
    # sending SLAM position to cf
    sendExternalPosition = True
    rospy.sleep(1.)
    # fly to point
    #cf.goTo(goal = [1, 0.0, 0.0], yaw=0, duration = 5.0, relative = True)
    #time.sleep(10.0)
    #cf.goTo(goal = [0, 1, 0.0], yaw=0, duration = 5.0, relative = True)
    #time.sleep(10.0)
    #cf.goTo(goal = [-1, 0.0, 0.0], yaw=0, duration = 5.0, relative = True)
    #time.sleep(10.0)
    #cf.goTo(goal = [0, -1, 0.0], yaw=0, duration = 5.0, relative = True)
    #time.sleep(10.0)
    
    # land
    #cf.land(targetHeight = 0.0, duration = 2.0)
    #input("Press Enter to stop test")
    
    cf.stop()
    cf.setParam("commander/enHighLevel", 0)
    rospy.spin()
