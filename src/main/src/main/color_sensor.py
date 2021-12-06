import rospy
import math
import random

class ColorSensor():
    def __init__(self,color_map):
        self.color_map = color_map
    
    def getReading(self,x,y):
        reading = self.color_map[x,y]
        if reading is None:
            return self.getNoise()
        else:
            return reading
        
    def getNoise():
        return [100+int(25-random.random()*50),100+int(25-random.random()*50),100+int(25-random.random()*50)]
