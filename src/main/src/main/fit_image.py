

class ImageFitter(object):
    
    def __init__(self):
        self.IMAGE_RES = 0.05
        
       
    def findLocation(self, map, image):
        """
        Find a location on the given map to print the given image.
        Called when the robot has finished mapping the environment
        (in early test runs this will be called straight away on the 
        provided map).
        
        :Args:
            | map: the map created from SLAM or given to the robot
            | image: the image represented as a 2D array
        :Return:
            | (x,y) map coordinates to start drawing image
        """
        self.position.x = []
        self.position.y = []
        self.position.z = []
        
        # RAIN(XT)
select randomly W ⊂X
Nt←|{i : ti = t}| for t = −1,+1
Bi ←√MAX(N−1,N+1)/Nti for i = 1,...,NˆH ←B ·(XT W)/(⊮X + ⊮W −XT W)
β ←
(
I/C + ˆHT ˆH
)−1
( ˆHT B ·T)
return W,β
PREDICT(X)
H ←(XT W)/(⊮X + ⊮W −XT W)
return SIGN(Hβ)#
        
        return (0,0)
