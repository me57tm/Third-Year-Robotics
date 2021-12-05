import rospy
import math
import random

class ColorSensor():
    def __init__(self,color_map):
        self.color_map = color_map
    
    def getReading(self,x,y):
        reading = self.color_map[x,y] #these or the definition might be the wrong way round.
        if reading is None:
            return getNoise()
        else:
            return reading
        
    def getNoise():
        return [100+int(25-random.random()*50),100+int(25-random.random()*50),100+int(25-random.random()*50)]
