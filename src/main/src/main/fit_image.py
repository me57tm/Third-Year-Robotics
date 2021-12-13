from PIL import Image

class ImageFitter(object):
    
    def __init__(self, image):
        self.IMAGE_RES = 0.05
        self.image = image
        self.robotBuffer = 0
       
    def findLocation(self, map):
        """
        Find a location on the given map to print the given image.
        Called when the robot has finished mapping the environment
        (in early test runs this will be called straight away on the 
        provided map).
        
        :Args:
            | map: the map created from SLAM or given to the robot
        :Return:
            | (x,y) map coordinates to start drawing image
        """
        self.robotBuffer = int(2/map.info.resolution)#0.45 for robot sized buffer 0.6 is for extra space 1.2 is for both sides of image
        print(self.robotBuffer)
        for x in range (0, map.info.width + self.robotBuffer): 
            for y in range (0, map.info.height + self.robotBuffer):
                if self.findGridProb(x, y, map) == 0:
                    if self.checkImage(x, y, map) == True:
                        return (x + self.robotBuffer,y + self.robotBuffer)
        return "hi"
                            
                
    def checkImage(self,x, y, map): # check the image and a buffer around each edge of the image fits without touching a wall
        for i in range (0, self.image.size[0] + self.robotBuffer):
            for j in range (0, self.image.size[1] + self.robotBuffer):
                if self.findGridProb(x+i, y+j, map) != 0:
                    return False
        return True

    def findGridProb(self,x, y, map): # 0 = clear, 100 = wall, -1 = unknown
        if x < 0 or x >= map.info.width or y < 0 or y >= map.info.height: # is it out of bounds
            return -1
        return map.data[x+y*map.info.height]
