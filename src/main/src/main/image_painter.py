from PIL import Image

class ImagePainter():
    def __init__(self, starting_coord):
        """ 
        Args:
            | starting_coord: the first coordinate of the image to draw
        """
        self.command_queue = []
        self.image = Image.open("src/main/data/blacksquare.png")
        value = self.image.getpixel((0,0))
        # print(value)
        # print(self.image.format, self.image.size, self.image.mode)
        self.running = False
        self.generateCommands(starting_coord)
        pass

    def toggle(self): # in case we need a while loop /shrug
        self.running = not self.running

    def generateCommands(self, starting_coord):
        for y in range(starting_coord[1], starting_coord[1] + self.image.size[1]):
            for x in range(starting_coord[0], starting_coord[0] + self.image.size[0]):
                # print((x-starting_coord[0])/self.image.size[0],(y-starting_coord[1])/self.image.size[1])
                rgb = self.image.getpixel(((x-starting_coord[0])/self.image.size[0],(y-starting_coord[1])/self.image.size[1]))
                self.command_queue.append((x,y,rgb))
                print(x,y,rgb)
        pass