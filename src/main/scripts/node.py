#!/usr/bin/python3

"""
This is the main entry point for the particle filter exercise node. It
subscribes to laser, map, and odometry and creates an instance of
pf.PFLocaliser() to do the localisation.
"""

import rospy
import main.pf
import main.fit_image
import main.color_sensor
import main.painter_navigator
from main.util import *
from PIL import Image
import numpy as np

from geometry_msgs.msg import ( Pose, PoseStamped, PoseWithCovarianceStamped,
                                PoseArray, Quaternion, Transform, TransformStamped, Twist )
from tf.msg import tfMessage
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import main
from threading import Lock

import sys
from copy import deepcopy

class PaintingNode(object):
    def __init__(self):
        # ----- Minimum change (m/radians) before publishing new particle cloud and pose
        self._PUBLISH_DELTA = rospy.get_param("publish_delta", 0.1)  
        
        self._latest_scan = None
        self._last_published_pose = None
        self._initial_pose_received = False
        
        self._image = Image.open("src/main/data/face.png")
        

        self._pose_publisher = rospy.Publisher("/estimatedpose", PoseStamped)
        self._truth_publisher = rospy.Publisher("/truthpose", PoseStamped) # Cheaty and only to be used for real world sensor type things
        self._amcl_pose_publisher = rospy.Publisher("/amcl_pose",
                                                    PoseWithCovarianceStamped)
        self._cloud_publisher = rospy.Publisher("/particlecloud", PoseArray)
        self._tf_publisher = rospy.Publisher("/tf", tfMessage)
        

        rospy.loginfo("Waiting for a map...")
        try:
            ocuccupancy_map = rospy.wait_for_message("/map", OccupancyGrid, 20)
        except:
            rospy.logerr("Problem getting a map. Check that you have a map_server"
                     " running: rosrun map_server map_server <mapname> " )
            sys.exit(1)
        rospy.loginfo("Map received. %d X %d, %f px/m." %
                      (ocuccupancy_map.info.width, ocuccupancy_map.info.height,
                       ocuccupancy_map.info.resolution))
        
        self._image_fitter = main.fit_image.ImageFitter(self._image)
        height = ocuccupancy_map.info.height
        width = ocuccupancy_map.info.width
        self._colour_map = np.zeros((height, width, 3), dtype=np.uint8)
        self._colour_map[0:height][0:width] = [127, 127, 127]
        #self._colour_map = [[None]*ocuccupancy_map.info.height]*ocuccupancy_map.info.width#This may or may not be the wrong way round
        # for xrow in self._colour_map:
        #     for i in range(0, len(xrow)):
        #         xrow[i] = (255, 255, 255)
        self._colour_sensor = main.color_sensor.ColorSensor(self._colour_map)
        
        self._paint_location = self._image_fitter.findLocation(ocuccupancy_map)
        rospy.loginfo(self._paint_location)
        
        self._paint_and_nav_thread = main.painter_navigator.PainterNavigator(1, "PainterNavigator", 1, self._paint_location, self._colour_map, self._image, ocuccupancy_map)
        
        self._particle_filter = main.pf.PFLocaliser(self._colour_sensor)
        
        self._particle_filter.set_map(ocuccupancy_map)
        
        self._laser_subscriber = rospy.Subscriber("/base_scan", LaserScan,
                                                  self._laser_callback,
                                                  queue_size=1)
        self._initial_pose_subscriber = rospy.Subscriber("/initialpose",
                                                         PoseWithCovarianceStamped,
                                                         self._initial_pose_callback)
        self._odometry_subscriber = rospy.Subscriber("/odom", Odometry,
                                                     self._odometry_callback,
                                                     queue_size=1)
        self._truth_subscriber = rospy.Subscriber("/base_pose_ground_truth",Odometry,self._ground_truth_callback,queue_size=1)


    def _initial_pose_callback(self, pose):
        """ called when RViz sends a user supplied initial pose estimate """
        self._particle_filter.set_initial_pose(pose)
        self._last_published_pose = deepcopy(self._particle_filter.estimatedpose)
        self._initial_pose_received = True
        self._cloud_publisher.publish(self._particle_filter.particlecloud)
        self._paint_and_nav_thread.start()

    def _odometry_callback(self, odometry):
        """
        Odometry received. If the filter is initialised then execute
        a filter predict step with odeometry followed by an update step using
        the latest laser.
        """
        if self._initial_pose_received:
            t_odom = self._particle_filter.predict_from_odometry(odometry)
            t_filter = self._particle_filter.update_filter(self._latest_scan)
            # ----- Get updated particle cloud and publish it
            self._cloud_publisher.publish(self._particle_filter.particlecloud)
        
            # ----- Get updated transform and publish it
            self._tf_publisher.publish(self._particle_filter.tf_message)            
            #if t_odom + t_filter > 0.1:
                #rospy.logwarn("Filter cycle overran timeslot")
                #rospy.loginfo("Odometry update: %fs"%t_odom)
                #rospy.loginfo("Particle update: %fs"%t_filter)
            # ----- Get updated pose estimate and publish it
            estimatedpose =  PoseStamped()
            estimatedpose.pose = self._particle_filter.estimatedpose.pose.pose
            estimatedpose.header.frame_id = "map"
            self._pose_publisher.publish(estimatedpose)
                
    def _ground_truth_callback(self,odometry):
        transform = Transform()
        transform.translation.x = odometry.pose.pose.position.x + 15.050000224262476 #TODO: Hardcoded & bad (stolen from sensor model map origin)
        transform.translation.y = odometry.pose.pose.position.y +15.050000224262476
        transform.rotation.x=odometry.pose.pose.orientation.x
        transform.rotation.y=odometry.pose.pose.orientation.y
        transform.rotation.z=odometry.pose.pose.orientation.z
        transform.rotation.w=odometry.pose.pose.orientation.w
        new_tfstamped = TransformStamped()
        new_tfstamped.child_frame_id = "/base_pose_ground_truth"
        new_tfstamped.header.frame_id = "map"
        new_tfstamped.header.stamp = rospy.Time.now()
        new_tfstamped.transform = transform
        self._tf_publisher.publish(tfMessage(transforms=[new_tfstamped]))

        truthPose = Pose()
        truthPose.position.x = transform.translation.x
        truthPose.position.y = transform.translation.y
        truthPose.orientation.x=transform.rotation.x
        truthPose.orientation.y=transform.rotation.y
        truthPose.orientation.z=transform.rotation.z
        truthPose.orientation.w=transform.rotation.w
        ps = PoseStamped()
        ps.pose = truthPose
        ps.header.frame_id = "map"
        self._truth_publisher.publish(ps)
        
    
    def _laser_callback(self, scan):
        """
        Laser received. Store a ref to the latest scan. 
        """
        self._latest_scan = scan

    def _cmd_vel_callback(self, twist):
        pass

if __name__ == '__main__':
    # --- Main Program  ---
    rospy.init_node("main")
    node = PaintingNode()
    rospy.spin()
