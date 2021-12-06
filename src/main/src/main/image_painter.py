from PIL import Image
from geometry_msgs.msg import  (PoseStamped, Pose, PoseWithCovarianceStamped, Twist)
import threading
import rospy
import math

class ImagePainter(threading.Thread):
    def __init__(self, threadID, name, counter, starting_coord, colour_map, image):
        """ 
        Args:
            | starting_coord: the first coordinate of the image to draw
        """
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter
        self.command_queue = []
        self.colour_map = colour_map
        self.image = image
        self.latest_pose_estimate = Pose()
        value = self.image.getpixel((0,0))
        # print(starting_coord)
        # print(value)
        # print(self.image.format, self.image.size, self.image.mode)
        self.generateCommands(starting_coord)
        self._cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        #self._cmd_vel_subscriber = rospy.Subscriber("/cmd_vel", Twist, self._cmd_vel_callback, queue_size=1)
        
    def run(self):
        self.drawOrMove()
        
    def generateCommands(self, starting_coord):
        for y in range(starting_coord[1], starting_coord[1] + self.image.size[1]):
            for x in range(starting_coord[0], starting_coord[0] + self.image.size[0]):
                # print((x-starting_coord[0])/self.image.size[0],(y-starting_coord[1])/self.image.size[1])
                rgb = self.image.getpixel(((x-starting_coord[0])/self.image.size[0],(y-starting_coord[1])/self.image.size[1]))
                self.command_queue.append((x,y,rgb))
                # print(x,y,rgb)
        
    def drawOrMove(self):
        rospy.loginfo("you!")
        while(len(self.command_queue) > 0):
            # TODO get estimated pose
            self.latest_pose_estimate = rospy.wait_for_message("/estimatedpose", PoseStamped, timeout=None)
            # if we are at the correct position, draw the pixel and pop the current command
            # if not then move to the correct position
            rospy.loginfo("got pose")
            x, y, rgb = self.command_queue[0]
            # print(self.command_queue[0])
            print(x, y, rgb)
            if self.checkPosition(x,y):
                print("in position")
                self.paint(x, y, rgb)
                self.command_queue.pop(0)
            else:
                print("moving")
                self.moveTowards(x, y)

    def checkPosition(self, targetx, targety):
        range = 1
        cp = self.latest_pose_estimate.pose.pose
        cpx = cp.position.x
        cpy = cp.position.y
        if cpx > targetx - range and cpx < targetx + range:
            if cpy > targety - range and cpy < targety + range:
                return True
        return False

    def paint(self, x, y, rgb):
        self.colour_map[y][x] = rgb

    def moveTowards(self, x, y):
        rospy.loginfo("you made it!")
        cp = self.current_pose.pose.pose
        cpx = cp.position.x
        cpy = cp.position.y
        angle = math.atan2(y-cpy, x-cpx)
        distance = math.sqrt(((x-cpx)*(x-cpx)) +((y-cpy)*(y-cpy)))
        twist = Twist()
        twist.linear.x = distance
        twist.angular.z = angle
        self._cmd_vel_publisher.publish(twist)
