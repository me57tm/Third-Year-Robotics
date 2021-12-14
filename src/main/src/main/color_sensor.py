import rospy
import math
import random
import numpy as np

class ColorSensor():
    def __init__(self,color_map):
        self.color_map = color_map
        self.COLOUR_SIGMA = 5
    
    def getReading(self,x,y):
        try:
            reading = self.color_map[x][y]
        except:
            print("Oops, tried to read colour from:",x,y)
            reading = self.getNoise()
        '''for z in reading:
            if z > 135 or z < 85:
                print("Colour read! :",reading,"@",x,y)'''
        '''if reading is None:
            reading = self.getNoise()
        else:'''
        #rng = np.random.default_rng()
        #randParams = rng.normal(0,1,3)
        #reading[0] = reading[0] + (self.COLOUR_SIGMA * randParams[0])
        #reading[1] = reading[1] + (self.COLOUR_SIGMA * randParams[1])
        #reading[2] = reading[2] + (self.COLOUR_SIGMA * randParams[2])
        #for col in reading:
        #    if col > 255:
        #        col = 255
        #    if col < 0:
        #       col = 0
        return reading
        
    def getNoise(self):
        return [110+int(25-random.random()*50),110+int(25-random.random()*50),110+int(25-random.random()*50)]
