from PIL import Image

class ImageFitter(object):
    
    def __init__(self):
        self.IMAGE_RES = 0.05
        self.image = Image.open("src/main/data/blacksquare.png")
        
       
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
        for x in range (0, map.info.width):
            for y in range (0, map.info.height):
                if self.findGridProb(x, y, map) == 0:
                    print("made it to loop")
                    if self.checkImage(x, y, map) == True:
                        return (x,y)
        return "hi"
                            
                
    def checkImage(self,x, y, map):
        for i in range (0, self.image.size[0]):
            for j in range (0, self.image.size[1]):
                if self.findGridProb(x+i, y+j, map) != 0:
                    return False
        return True

    def findGridProb(self,x, y, map): # 0 = clear, 100 = wall, -1 = unknown
        if x < 0 or x >= map.info.width or y < 0 or y >= map.info.height: # is it out of bounds
            return -1
        return map.data[x+y*map.info.height]
