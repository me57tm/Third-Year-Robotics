from PIL import Image
from geometry_msgs.msg import  (PoseStamped, Pose, PoseWithCovarianceStamped, Twist)
import threading
import rospy
import math
import numpy
from . util import getHeading

class ImagePainter(threading.Thread):
    def __init__(self, threadID, name, counter, starting_coord, colour_map, image, map):
        """ 
        Args:
            | starting_coord: the first coordinate of the image to draw
        """
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.map = map
        self.counter = counter
        self.command_queue = []
        self.colour_map = colour_map
        self.image = image
        self.output_image = False
        self.latest_pose_estimate = Pose()
        self.debug_image_generation = True
        self.rate = rospy.Rate(50)
        value = self.image.getpixel((0,0))
        for x in range(0, self.map.info.width):
            for y in range(0, self.map.info.height):
                if self.findGridProb(x,y) == 100:
                    self.colour_map[y][x] = [255, 0, 0]

        array1 = numpy.array(self.colour_map)
        self.output_image = Image.fromarray(array1, "RGB")
        self.output_image.save("images/before.png", "PNG")
        self.generateCommands(starting_coord)
        self._cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # "hyper paramaters"
        self.prange = 0 # lower the number more accurate the painting but takes longer per pixel
        
    def run(self):
        self.drawOrMove()

    def generateCommands(self, starting_coord):
        for y in range(starting_coord[1], starting_coord[1] + self.image.size[1]):
            for x in range(starting_coord[0], starting_coord[0] + self.image.size[0]):
                pixelx = x-starting_coord[0]
                pixely = y-starting_coord[1]
                rgb = self.image.getpixel((pixelx,pixely))
                self.command_queue.append((x,y,rgb))
        
    def drawOrMove(self):
        rospy.loginfo("you!")
        
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
            cp = self.latest_pose_estimate.pose
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
                self.moveTowards(x, y)
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

    def paint(self, x, y, rgb):
        r, g, b = rgb
        m,n,o = self.colour_map[x][y]
        if m == r and n == g and o == b:
            return False
        self.colour_map[x][y] = [r, g, b]
        return True

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

        print("Goal:", x, y, "Current:", cpx/self.map.info.resolution, cpy/self.map.info.resolution)
        print("Goal Angle:", math.degrees(angle), "Current Angle:", math.degrees(cpr))

        # send movement commands to the bot
        twista = Twist()
        # move once the current and goal angles are alligned to 2dp otherwise correct the angle
        if math.floor(10*cpr) == math.floor(10*angle):
            print("moving:", distance)
            twista.linear.x = distance
            twista.angular.z = 0
        else:
            print("turning", math.degrees(correction_angle))
            twista.linear.x = 0
            twista.angular.z = correction_angle
        self._cmd_vel_publisher.publish(twista)
        self.rate.sleep()
        
    def findGridProb(self,x, y): # 0 = clear, 100 = wall, -1 = unknown
        if x < 0 or x >= self.map.info.width or y < 0 or y >= self.map.info.height: # is it out of bounds
            return -1
        return self.map.data[x+y*self.map.info.height]