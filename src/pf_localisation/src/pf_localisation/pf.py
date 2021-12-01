from geometry_msgs.msg import Pose, PoseArray, Quaternion
from . pf_base import PFLocaliserBase
import math
import rospy
import numpy
import copy
import statistics

from . util import rotateQuaternion, getHeading
import random

from time import time


class PFLocaliser(PFLocaliserBase):
    
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # ----- Set motion model parameters
        self.NUM_PARTICLES = 300
        self.ODOM_ROTATION_NOISE = 0 		# Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0 	# Odometry x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0 			# Odometry y axis (side-side) noise
        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict
        self.VALID_POINTS = []  #A List of valid map points which you may wish to take a sample from to get random points. Not for use to detect if a point is valid.
        self.GAUSS_POSITION_SIGMA = 0.05
        self.GAUSS_ANGLE_SIGMA = math.pi/8
        self.MUTATION_PERCENTAGE = 5
        self.CLUSTER_SIZE = 2
        
       
    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise
        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        ip = initialpose.pose.pose
        
        initX = ip.position.x
        initY = ip.position.y
        
        rng = numpy.random.default_rng()
        coordSigma = 2
        rotSigma = math.pi/2
        
        pa = PoseArray() #particle cloud
        pa.poses.append(ip)
        
        for i in range(0,self.NUM_PARTICLES-1): #initialising particle cloud
            tries = 0
            while True: # generate random params that give us in bound particles for the particle cloud
                randParams = rng.normal(0, 1, 3)
                if self.findGridProb(initX + (coordSigma * randParams[0]), initY + (coordSigma * randParams[1])) == 0:
                    break
                if tries > 20: # Prevent code hanging
                    break
                tries += 1
            
            newPose = Pose() # create particle
            newPose.position.x = initX + (coordSigma * randParams[0])
            newPose.position.y = initY + (coordSigma * randParams[1])
            newPose.orientation = rotateQuaternion(ip.orientation,rotSigma * randParams[2])
            
            pa.poses.append(newPose) # append to particle cloud
        
        
        for i in range(len(self.occupancy_map.data)): 
            if self.occupancy_map.data[i] == 0: #Valid location
                x = i % self.occupancy_map.info.width
                y = i // self.occupancy_map.info.height
                x *= self.occupancy_map.info.resolution
                y *= self.occupancy_map.info.resolution
                self.VALID_POINTS.append([x,y])
                
        return pa

 
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update
         """
        #Get Weights and Normalise
        weightList = [self.sensor_model.get_weight(scan, particle) for particle in self.particlecloud.poses]
        totalWeight = sum(weightList)
        
        #Resample
        c = weightList[0] / totalWeight
        u = 0
        newPoseArray = []
        i = 0
        while u < 1:
            if c > u:
                newPoseArray.append(copy.deepcopy(self.particlecloud.poses[i]))
                u += 1/self.NUM_PARTICLES
            else:
                i = i + 1
                if i == self.NUM_PARTICLES:
                	break
                c = c + weightList[i] / totalWeight
        newPoseArray = self.addGausian(self.GAUSS_POSITION_SIGMA,self.GAUSS_ANGLE_SIGMA,newPoseArray)

        #reroll invalid particles
        for i in range(len(newPoseArray)):
            if  self.findGridProb(newPoseArray[i].position.x,newPoseArray[i].position.y) != 0 or i % (100//self.MUTATION_PERCENTAGE) == 0:
                newPos = random.choice(self.VALID_POINTS)
                newPoseArray[i].position.x = newPos[0]
                newPoseArray[i].position.y = newPos[1]
        
        self.particlecloud.poses = newPoseArray

        

    def estimate_pose(self):
        
        poses = PoseArray()
        poses = self.particlecloud.poses
        numposes = len(poses)
        
        xposes = [0]*numposes
        yposes = [0]*numposes
        for i in range(0, numposes): # creating grid
            xposes[i] = round(poses[i].position.x) # rounding to nearest m
            yposes[i] = round(poses[i].position.y)
        xmode = statistics.mode(xposes) # find mode
        ymode = statistics.mode(yposes)
        lowerx = xmode - self.CLUSTER_SIZE/2 # setting bounds of cluster
        higherx = xmode + self.CLUSTER_SIZE/2
        lowery = ymode - self.CLUSTER_SIZE/2
        highery = ymode + self.CLUSTER_SIZE/2
        ep = Pose() # estimated pose
        x = 0 # position
        y = 0
        rx = 0 # quaternion
        ry = 0
        rz = 0
        rw = 0
        for pose in (poses): # find poses inside the cluster
            if lowerx < pose.position.x < higherx and lowery < pose.position.y < highery:
                x = x + pose.position.x
                y = y + pose.position.y
                rx = rx + pose.orientation.x
                ry = ry + pose.orientation.y
                rz = rz + pose.orientation.z
                rw = rw + pose.orientation.w
            else:
                numposes = numposes - 1
        if numposes  == 0: # divide by 0!
        	numposes = 1
        ep.position.x = x / numposes # averages
        ep.position.y = y / numposes
        ep.orientation.x = rx / numposes
        ep.orientation.y = ry / numposes
        ep.orientation.z = rz / numposes
        ep.orientation.w = rw / numposes
        return ep

    def addGausian(self, coordSigma,rotSigma,poseArray): #wysiwyg
        rng = numpy.random.default_rng()
        for i in range(len(poseArray)):
            randParams = rng.normal(0,1,3)
            poseArray[i].position.x = poseArray[i].position.x + (coordSigma * randParams[0])
            poseArray[i].position.y = poseArray[i].position.y + (coordSigma * randParams[1])
            poseArray[i].orientation = rotateQuaternion(poseArray[i].orientation,rotSigma * randParams[2])
        return poseArray
        
        # checks x y position on map
    def findGridProb(self,x, y): # 0 = clear, 100 = wall, -1 = unknown
        i = int(x / self.occupancy_map.info.resolution)
        j = int(y / self.occupancy_map.info.resolution)
        if i < 0 or i >= self.occupancy_map.info.width or j < 0 or j >= self.occupancy_map.info.height: # is it out of bounds
            return -1
        return self.occupancy_map.data[i+j*self.occupancy_map.info.height]
        u = 0
        newPoseArray = []
        i = 0
        while u < 1:
            if c > u:
                newPoseArray.append(copy.deepcopy(self.particlecloud.poses[i]))
                u += 1/self.NUM_PARTICLES
            else:
                i = i + 1
                if i == self.NUM_PARTICLES:
                	break
                c = c + weightList[i] / totalWeight
        newPoseArray = self.addGausian(self.GAUSS_POSITION_SIGMA,self.GAUSS_ANGLE_SIGMA,newPoseArray)

        #reroll invalid particles
        for i in range(len(newPoseArray)):
            if  self.findGridProb(newPoseArray[i].position.x,newPoseArray[i].position.y) != 0 or i % (100//self.MUTATION_PERCENTAGE) == 0:
                newPos = random.choice(self.VALID_POINTS)
                newPoseArray[i].position.x = newPos[0]
                newPoseArray[i].position.y = newPos[1]
        
        self.particlecloud.poses = newPoseArray

        

    def estimate_pose(self):
        
        poses = PoseArray()
        poses = self.particlecloud.poses
        numposes = len(poses)
        
        xposes = [0]*numposes
        yposes = [0]*numposes
        for i in range(0, numposes): # creating grid
            xposes[i] = round(poses[i].position.x) # rounding to nearest m
            yposes[i] = round(poses[i].position.y)
        xmode = statistics.mode(xposes) # find mode
        ymode = statistics.mode(yposes)
        lowerx = xmode - self.CLUSTER_SIZE/2 # setting bounds of cluster
        higherx = xmode + self.CLUSTER_SIZE/2
        lowery = ymode - self.CLUSTER_SIZE/2
        highery = ymode + self.CLUSTER_SIZE/2
        ep = Pose() # estimated pose
        x = 0 # position
        y = 0
        rx = 0 # quaternion
        ry = 0
        rz = 0
        rw = 0
        for pose in (poses): # find poses inside the cluster
            if lowerx < pose.position.x < higherx and lowery < pose.position.y < highery:
                x = x + pose.position.x
                y = y + pose.position.y
                rx = rx + pose.orientation.x
                ry = ry + pose.orientation.y
                rz = rz + pose.orientation.z
                rw = rw + pose.orientation.w
            else:
                numposes = numposes - 1
        if numposes  == 0: # divide by 0!
        	numposes = 1
        ep.position.x = x / numposes # averages
        ep.position.y = y / numposes
        ep.orientation.x = rx / numposes
        ep.orientation.y = ry / numposes
        ep.orientation.z = rz / numposes
        ep.orientation.w = rw / numposes
        return ep

    def addGausian(self, coordSigma,rotSigma,poseArray): #wysiwyg
        rng = numpy.random.default_rng()
        for i in range(len(poseArray)):
            randParams = rng.normal(0,1,3)
            poseArray[i].position.x = poseArray[i].position.x + (coordSigma * randParams[0])
            poseArray[i].position.y = poseArray[i].position.y + (coordSigma * randParams[1])
            poseArray[i].orientation = rotateQuaternion(poseArray[i].orientation,rotSigma * randParams[2])
        return poseArray
        
        # checks x y position on map
    def findGridProb(self,x, y): # 0 = clear, 100 = wall, -1 = unknown
        i = int(x / self.occupancy_map.info.resolution)
        j = int(y / self.occupancy_map.info.resolution)
        if i < 0 or i >= self.occupancy_map.info.width or j < 0 or j >= self.occupancy_map.info.height: # is it out of bounds
            return -1
        return self.occupancy_map.data[i+j*self.occupancy_map.info.height]
        
        
