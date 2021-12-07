from geometry_msgs.msg import  (PoseStamped, Pose, PoseWithCovarianceStamped, Twist)
import threading
import rospy
import math
from . util import getHeading
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
        
        self.nav_grid = []
        rospy.loginfo("first loop")
        for i in range (0, map.info.width//self.grid_res):
            blank = []
            for j in range (0, map.info.height//self.grid_res):
                blank.append(False)
            self.nav_grid.append(blank)

        x = 0
        y = 0
        rospy.loginfo("second loop")
        for x in range (0, map.info.width//self.grid_res):
            for y in range (0, map.info.height//self.grid_res):
                print(x, y)
                self.nav_grid[x][y] = self.checkGridCell(x, y)
        rospy.loginfo("done")
        with open("new_file.csv","w+") as my_csv:
            csvWriter = csv.writer(my_csv,delimiter=',')
            csvWriter.writerows(self.nav_grid)
        print(self.goal_coord)
        print(self.goal_grid)
        print(self.nav_grid[self.goal_grid[0]][self.goal_grid[1]])     
    def checkGridCell(self, x, y):
        for i in range (0, self.grid_res):
            for j in range (0, self.grid_res):
                if self.findGridProb(x*self.grid_res+i, y*self.grid_res+j) != 0:
                    return False
        rospy.loginfo("Found a clear cell")
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
            print(current_square)
            if current_square[0] == self.goal_grid[0] and current_square[1] == self.goal_grid[1]:
                print("found path")
                path = []
                while current_square is not None:
                    path.append((current_square[0]*self.grid_res+self.grid_res//2,current_square[1]*self.grid_res+self.grid_res//2))
                    current_square = current_square[4]
                return path[::-1]
            for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
                print("does for at least")
                new_x = current_square[0] + new_position[0]
                new_y = current_square[1] + new_position[1]
                if new_x < len(self.nav_grid) and new_y < len(self.nav_grid[0]):
                    print("new positions in grid")
                    print(self.nav_grid[new_x])
                    if self.nav_grid[new_x][new_y]:
                        print("non walled value")
                        closed = False
                        for closed_square in closed_list:
                            if closed_square[0] == new_x and closed_square[1] == new_y:
                                closed = True
                                print("oh is this it?")
                                break
                        print("closed bit")
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
        self.command_queue = self.generatePath()
        print(self.command_queue)
        while(len(self.command_queue) > 0):
            self.latest_pose_estimate = rospy.wait_for_message("/estimatedpose", PoseStamped, timeout=None)
            # if we are at the correct position, draw the pixel and pop the current command
            # if not then move to the correct position
            x, y = self.command_queue[0]
            # print(self.command_queue[0])
            if self.checkPosition(x,y):
                print("in position")
                self.command_queue.pop(0)
            else:
                self.moveTowards(x, y)

    def checkPosition(self, targetx, targety):
        prange = 5
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
        cpr = getHeading(cp.orientation)
        angle = math.atan2(y-cpy, x-cpx)
        distance = math.sqrt(((x-cpx)*(x-cpx)) +((y-cpy)*(y-cpy)))
        twista = Twist()
        twista.linear.x = 0.05
        twista.angular.z = (angle - cpr)
        t=0.1
        #i=0
        #while i<1:
        self._cmd_vel_publisher.publish(twista)
        rospy.sleep(t)
        #    i = i + t
        #twistd = Twist()
        #twistd.linear.x = distance
        #i=0
        #while i<1:
        #    self._cmd_vel_publisher.publish(twistd)
        #    rospy.sleep(t)
        #    i = i + t

    def findGridProb(self,x, y): # 0 = clear, 100 = wall, -1 = unknown
        if x < 0 or x >= self.map.info.width or y < 0 or y >= self.map.info.height: # is it out of bounds
            return -1
        return self.map.data[x+y*self.map.info.height]
