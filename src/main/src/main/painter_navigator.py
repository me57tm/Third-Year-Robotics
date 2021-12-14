from PIL import Image
from geometry_msgs.msg import  (PoseStamped, Pose, PoseWithCovarianceStamped, Twist)
import threading
import rospy
import math
from . util import (getHeading, rotateQuaternion)
import numpy

class PainterNavigator(threading.Thread):
    def __init__(self, threadID, name, counter, goal_coord, colour_map, image, map):
        """ 
        Args:
            | starting_coord: the first coordinate of the image to draw
        """
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.map = map
        self.counter = counter
        self.goal_coord = goal_coord
        self.command_queue = []
        self.colour_map = colour_map
        self.image = image
        self.latest_ground_truth = None
        self.output_image = False
        self.latest_pose_estimate = Pose()
        self.debug_image_generation = True
        self.rate = rospy.Rate(50)
        value = self.image.getpixel((0,0))
        for x in range(0, self.map.info.width):
            for y in range(0, self.map.info.height):
                if self.findGridProb(x,y) == 100:
                    self.colour_map[x][y] = [255, 0, 0]

        array1 = numpy.array(self.colour_map)
        self.output_image = Image.fromarray(array1, "RGB")
        self.output_image.save("images/before.png", "PNG")
        
        self._cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # "hyper paramaters"
        self.prange = 0 # lower the number more accurate the painting but takes longer per pixel

        self.grid_res = map.info.width//60
        self.goal_grid = (goal_coord[0]//self.grid_res, goal_coord[1]//self.grid_res)
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
        for i in range (-5, 15):
            for j in range (-5, 15):
                if self.findGridProb(x*self.grid_res+i, y*self.grid_res+j) != 0:
                    return False
        return True
    
    def run(self):
        rospy.sleep(20)
        cached_prange = 0
        cached_prange = self.prange
        self.prange = 3
        self.navigate(self.goal_coord[0], self.goal_coord[1])
        self.prange = cached_prange
        self.generateCommands(self.goal_coord)
        self.drawOrMove()
        
    def generatePath(self, goalx, goaly):
        rospy.loginfo("Start pathing")
        goal_grid = (goalx//self.grid_res, goaly//self.grid_res)
        self.latest_pose_estimate = rospy.wait_for_message("/estimatedpose", PoseStamped, timeout=None)
        rospy.loginfo("Got pose estimate")
        cp = self.latest_pose_estimate.pose
        print(cp)
        cpx = int((cp.position.x/self.grid_res)/self.map.info.resolution)
        cpy = int((cp.position.y/self.grid_res)/self.map.info.resolution)
        h = (cpx-goal_grid[0])**2 + (cpy-goal_grid[1])**2
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
            if current_square[0] == goal_grid[0] and current_square[1] == goal_grid[1]:
                print("found path")
                path = []
                while current_square is not None:
                    path.append((current_square[0]*self.grid_res+self.grid_res//2,current_square[1]*self.grid_res+self.grid_res//2))
                    current_square = current_square[4]
                return path[::-1]
            for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (1, 1), (-1, 1), (1, -1)]:
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
                            h = (new_x-goal_grid[0])**2 + (new_y-goal_grid[1])**2
                            child = (new_x, new_y, current_square[2] + 1, h, current_square)
                            better_open = False
                            for open_square in open_list:
                                if open_square[0] == new_x and open_square[1] == new_y and open_square[2] < child[2]:
                                    better_open = True
                                    break
                            if not better_open:
                                open_list.append(child)
                                
    def generateCommands(self, starting_coord):
        self.command_queue = []
        for y in range(starting_coord[1], starting_coord[1] + self.image.size[1]):
            for x in range(starting_coord[0], starting_coord[0] + self.image.size[0]):
                pixelx = x-starting_coord[0]
                pixely = y-starting_coord[1]
                rgb = self.image.getpixel((pixelx,pixely))
                self.command_queue.append((x,y,rgb))
        
    def navigate(self, goalx, goaly):
        self.latest_pose_estimate = rospy.wait_for_message("/estimatedpose", PoseStamped, timeout=None)
        while True:
            self.command_queue = self.generatePath(goalx, goaly)
            if self.command_queue is None:
                print("Path was none")
                rospy.sleep(1)
                continue
            if len(self.command_queue) == 1:
                break
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
                    lost = False
                    lost = self.moveTowards(x, y)
                    if lost:
                        self.command_queue = []
                    
    def drawOrMove(self):
        
        if self.debug_image_generation:
            print("saved goal image")
            for command in self.command_queue:
                x,y,rgb = command
                self.paint(x,y,(0, 0, 255))
            array2 = numpy.array(self.colour_map)
            self.output_image = Image.fromarray(array2, "RGB")
            self.output_image.save("images/goal.png", "PNG")
            for command in self.command_queue:
                x,y,rgb = command
                self.paint(x,y,(255,255,255))
            self.debug_image_generation = False
            
        while(len(self.command_queue) > 0):
            self.latest_pose_estimate = rospy.wait_for_message("/estimatedpose", PoseStamped, timeout=None)
            self.latest_ground_truth = rospy.wait_for_message("/truthpose", PoseStamped, timeout=None)
            cp = self.latest_ground_truth.pose
            cpx = cp.position.x/self.map.info.resolution
            cpy = cp.position.y/self.map.info.resolution
            x, y, rgb = self.command_queue[0]
            if self.checkPosition(x,y):     # if we are at the correct position, draw the pixel and pop the current command
                if  self.paint(round(cpx), round(cpy), rgb):
                    if len(self.command_queue)%self.image.size[0] == 0:
                        array3 = numpy.array(self.colour_map)
                        self.output_image = Image.fromarray(array3, "RGB")
                        self.output_image.save("images/during"+str(len(self.command_queue))+".png", "PNG")
                    self.command_queue.pop(0)
                    print("pixels remaining:", len(self.command_queue))
                else:   # if not then move to the correct position
                    print("already painted")
                    # move the bot slightly to get out of here asap
                    twist = Twist()
                    twist.linear.x = 0.1
                    self._cmd_vel_publisher.publish(twist)
            else:
                lost = False
                lost = self.moveTowards(x, y)
                if lost:
                    cached_queue = []
                    cached_queue = self.command_queue
                    self.command_queue = []
                    self.navigate(x, y)
                    self.command_queue = cached_queue
        print("finished image")
        array4 = numpy.array(self.colour_map)
        self.output_image = Image.fromarray(array4, "RGB")
        self.output_image.save("images/finished.png", "PNG")

    def checkPosition(self, targetx, targety):
        cp = self.latest_pose_estimate.pose
        cpx = round(cp.position.x/self.map.info.resolution)
        cpy = round(cp.position.y/self.map.info.resolution)
        distance = math.sqrt(((targetx-cpx)*(targetx-cpx)) +((targety-cpy)*(targety-cpy)))
        print("Pixel Distance:", math.floor(distance))
        if math.floor(distance) <= self.prange:
            return True
        return False

    def moveTowards(self, x, y):
        newx = x*self.map.info.resolution
        newy = y*self.map.info.resolution
        cp = self.latest_pose_estimate.pose
        cpx = cp.position.x
        cpy = cp.position.y
        if cpx == 0.0 or cpy == 0.0:
            return
        #sorting out the angles and distance
        cpr = getHeading(cp.orientation) # current angle
        angle = (math.atan2((newy-cpy), (newx-cpx))) # goal angle
        if angle < 0: #converting goal angle from 0->180 -180->-360 to 0->360
            angle = (2*math.pi) + angle
        if cpr < 0: # as we did with goal angle
            cpr = (2*math.pi) + cpr
        correction_angle = (angle - cpr)
        if math.degrees(correction_angle) > 180: #if we're turning over 180deg left or right, take the opposite direction
            correction_angle =  2*math.pi - correction_angle
        elif math.degrees(correction_angle) < -180:
            correction_angle = 2*math.pi + correction_angle
        distance = math.sqrt(((newx-cpx)*(newx-cpx)) +((newy-cpy)*(newy-cpy)))
        if distance > 1:
            print("bad pose estimate, repathing")
            return True
        print("Goal:", x, y, "Current:", cpx/self.map.info.resolution, cpy/self.map.info.resolution)
        print("Goal Angle:", math.degrees(angle), "Current Angle:", math.degrees(cpr))

        # send movement commands to the bot
        twista = Twist()
        # move once the current and goal angles are alligned to 2dp otherwise correct the angle
        if math.floor(10*cpr) == math.floor(10*angle):
            #print("moving:", distance)
            twista.linear.x = distance
            twista.angular.z = 0
        else:
            print("turning", math.degrees(correction_angle))
            twista.linear.x = 0
            twista.angular.z = correction_angle
        self._cmd_vel_publisher.publish(twista)
        self.rate.sleep()
        return False
        
    def paint(self, x, y, rgb):
        r, g, b = rgb
        m,n,o = self.colour_map[x][y]
        if m == r and n == g and o == b:
            return False
        self.colour_map[x][y] = [r, g, b]
        return True        

    def findGridProb(self,x, y): # 0 = clear, 100 = wall, -1 = unknown
        if x < 0 or x >= self.map.info.width or y < 0 or y >= self.map.info.height: # is it out of bounds
            return -1
        return self.map.data[x+y*self.map.info.height]
