from geometry_msgs.msg import  (PoseStamped, Pose, PoseWithCovarianceStamped, Twist)
import threading
import rospy
import math
from . util import (getHeading, rotateQuaternion)
import csv

class Navigator(threading.Thread):
    def __init__(self, threadID, name, counter, goal_coord, map):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.map = map
        self.counter = counter
        self.command_queue = []
        self.latest_pose_estimate = Pose()
        self._cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.grid_res = map.info.width//60
        self.goal_coord = goal_coord
        self.goal_grid = (goal_coord[0]//self.grid_res, goal_coord[1]//self.grid_res)
        self.rate = rospy.Rate(25)
        self.nav_grid = []
        for i in range (0, map.info.width//self.grid_res):
            blank = []
            for j in range (0, map.info.height//self.grid_res):
                blank.append(False)
            self.nav_grid.append(blank)

        x = 0
        y = 0
        for x in range (0, map.info.width//self.grid_res):
            for y in range (0, map.info.height//self.grid_res):
                self.nav_grid[x][y] = self.checkGridCell(x, y)
        rospy.loginfo("Made Nav Grid")
        print(self.goal_grid)
        print(self.nav_grid[self.goal_grid[0]][self.goal_grid[1]])     
    def checkGridCell(self, x, y):
        for i in range (0, self.grid_res):
            for j in range (0, self.grid_res):
                if self.findGridProb(x*self.grid_res+i, y*self.grid_res+j) != 0:
                    return False
        return True        
        
    def run(self):
        rospy.sleep(20)
        self.navigate()
        
    def generatePath(self):
        rospy.loginfo("Start pathing")
        self.latest_pose_estimate = rospy.wait_for_message("/estimatedpose", PoseStamped, timeout=None)
        rospy.loginfo("Got pose estimate")
        cp = self.latest_pose_estimate.pose
        print(cp)
        cpx = int((cp.position.x/self.grid_res)/self.map.info.resolution)
        cpy = int((cp.position.y/self.grid_res)/self.map.info.resolution)
        h = (cpx-self.goal_grid[0])**2 + (cpy-self.goal_grid[1])**2
        open_list = [(cpx, cpy, 0, h, None)]
        closed_list = []
        while len(open_list) > 0:
            current_square = open_list[0]
            current_i = 0
            for i, square in enumerate(open_list):
                if square[2] + square[3] < current_square[2] + current_square[3]:
                    current_square = square
                    current_i = i
            open_list.pop(current_i)
            closed_list.append(current_square)
            if current_square[0] == self.goal_grid[0] and current_square[1] == self.goal_grid[1]:
                print("found path")
                path = []
                while current_square is not None:
                    path.append((current_square[0]*self.grid_res+self.grid_res//2,current_square[1]*self.grid_res+self.grid_res//2))
                    current_square = current_square[4]
                return path[::-1]
            for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
                new_x = current_square[0] + new_position[0]
                new_y = current_square[1] + new_position[1]
                if new_x < len(self.nav_grid) and new_y < len(self.nav_grid[0]):
                    if self.nav_grid[new_x][new_y]:
                        closed = False
                        for closed_square in closed_list:
                            if closed_square[0] == new_x and closed_square[1] == new_y:
                                closed = True
                                break
                        if not closed:
                            h = (new_x-self.goal_grid[0])**2 + (new_y-self.goal_grid[1])**2
                            child = (new_x, new_y, current_square[2] + 1, h, current_square)
                            better_open = False
                            for open_square in open_list:
                                if open_square[0] == new_x and open_square[1] == new_y and open_square[2] < child[2]:
                                    better_open = True
                                    break
                            if not better_open:
                                open_list.append(child)
        
    def navigate(self):
        self.latest_pose_estimate = rospy.wait_for_message("/estimatedpose", PoseStamped, timeout=None)
        while not self.checkPosition(self.goal_coord[0], self.goal_coord[1]):
            self.command_queue = self.generatePath()
            print(self.command_queue)
            while(len(self.command_queue) > 0):
                self.latest_pose_estimate = rospy.wait_for_message("/estimatedpose", PoseStamped, timeout=None)
                # if we are at the correct position, pop the current command
                # if not then move to the correct position
                x, y = self.command_queue[0]
                # print(self.command_queue[0])
                if self.checkPosition(x,y):
                    print("in position")
                    self.command_queue.pop(0)
                else:
                    self.moveTowards(x, y)

    def checkPosition(self, targetx, targety):
        prange = 3
        cp = self.latest_pose_estimate.pose
        cpx = cp.position.x/self.map.info.resolution
        cpy = cp.position.y/self.map.info.resolution
        if cpx > targetx - prange and cpx < targetx + prange:
            if cpy > targety - prange and cpy < targety + prange:
                return True
        return False

    def moveTowards(self, x, y):
        x = x*self.map.info.resolution
        y = y*self.map.info.resolution
        cp = self.latest_pose_estimate.pose
        cpx = cp.position.x
        cpy = cp.position.y
        cpquat = cp.orientation
        #sorting out the angles and distance
        cpr = getHeading(cp.orientation) # current angle
        print("Initial Heading:", cpr)
        angle = (math.atan2((y-cpy), (x-cpx))) # goal angle
        if angle < 0: #converting goal angle from 0->180 -180->-360 to 0->360
            angle = (2*math.pi) + angle
        if cpr < 0: # as we did with goal angle
            cpr = (2*math.pi) + cpr
        correction_angle = (angle - cpr)
        if math.degrees(correction_angle) > 180: #if we're turning over 180deg left or right, take the opposite direction
            correction_angle =  2*math.pi - correction_angle
        elif math.degrees(correction_angle) < -180:
            correction_angle = 2*math.pi + correction_angle
        distance = math.sqrt(((x-cpx)*(x-cpx)) +((y-cpy)*(y-cpy)))
        if distance > 1:
            self.command_queue = []
            print("bad pose estimate, repathing")
            return
        #on an arc, 2x an angle corrects for movement during rotation
        arcradius = distance/(2*math.sin(correction_angle)) #distance to centre of imaginary circle of which the robot's path is an arc
        arclength = arcradius*2*correction_angle #length of the arc to be moved down
        arctime = 4 #time to execute the maneuver in seconds (must be capable of turning pi radians in this time)
        rotationalvel = (2*correction_angle)/arctime
        forwardvel = arclength/arctime
        rotationalvel = correction_angle/arctime
        forwardvel = 0.15
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = rotationalvel
        print("Correctional Heading:", angle)
        print("Correctional Distance:", distance)
        print("Arc Radius: ", arcradius)
        print("Angular Vel: ", rotationalvel)
        print("Forward Vel: ", forwardvel)
        # send movement commands to the bot
        twista = Twist()
        # move once the current and goal angles are alligned to 2dp otherwise correct the angle
        if math.floor(10*cpr) == math.floor(10*angle):
            print("moving:", distance)
            twista.linear.x = 2*distance
            twista.angular.z = 0
        else:
            print("turning", math.degrees(correction_angle))
            twista.linear.x = 0
            twista.angular.z = 2*correction_angle
        self._cmd_vel_publisher.publish(twista)
        self.rate.sleep()
        
        #t=0.2
        #i=0
        #while i < arctime:
        #    self._cmd_vel_publisher.publish(twist)
        #    rospy.sleep(t)
        #    i = i + t
        #twist.linear.x = 0
        #twist.angular.z = 0
        #self._cmd_vel_publisher.publish(twist)
        #
        #twist.linear.x = forwardvel
        #twist.angular.z = 0
        #t=0.1
        #i=0
        #while i < arctime:
        #    self._cmd_vel_publisher.publish(twist)
        #    rospy.sleep(t)
        #    i = i + t
        #twist.linear.x = 0
        #twist.angular.z = 0
        #self._cmd_vel_publisher.publish(twist)

    def findGridProb(self,x, y): # 0 = clear, 100 = wall, -1 = unknown
        if x < 0 or x >= self.map.info.width or y < 0 or y >= self.map.info.height: # is it out of bounds
            return -1
        return self.map.data[x+y*self.map.info.height]
