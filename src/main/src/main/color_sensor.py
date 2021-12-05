import rospy
import math
import random

class ColorSensor():
    def __init__(self):
        self.color_map = [[]]
    
    def set_map(self,occupancy_map):
        self.map_width = occupancy_map.info.width
        self.map_height = occupancy_map.info.height
        '''self.map_resolution = occupancy_map.info.resolution # in m per pixel
        self.map_data =  occupancy_map.data 
        self.map_origin_x = ( occupancy_map.info.origin.position.x +
                             (self.map_width / 2.0) * self.map_resolution )
        self.map_origin_y = ( occupancy_map.info.origin.position.y +
                              (self.map_height / 2.0) * self.map_resolution )'''
        self.color_map = [[None]*self.map_height]*self.map_width
    
    def getReading(self,x,y):
        reading = self.color_map[x,y] #these or the definition might be the wrong way round.
        if reading is None:
            return self.getNoise()
        else:
            return reading
        
    def getNoise():
        return [100+int(25-random.random()*50),100+int(25-random.random()*50),100+int(25-random.random()*50)]
