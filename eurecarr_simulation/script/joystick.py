# code based on: https://stackoverflow.com/questions/49352561/sending-joystick-input-to-program-using-python

import pygame

# Define some colors.
BLACK = pygame.Color('black')
WHITE = pygame.Color('white')

# This is a simple class that will help us print to the screen.
# It has nothing to do with the joysticks, just outputting the
# information.
class TextPrint(object):
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def tprint(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, (self.x, self.y))
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10

class Joystick(object):
    def __init__(self):
        pygame.init()
        # Set the width and height of the screen (width, height).
        self.screen = pygame.display.set_mode((500, 200))
        # pygame.display.set_caption("AI fly")
        # Loop until the user clicks the close button.
        self.done = False
        # Used to manage how fast the screen updates.
        self.clock = pygame.time.Clock()
        # Initialize the joysticks.
        pygame.joystick.init()
        # Get ready to print.
        self.textPrint = TextPrint()
        self.axis = []
        self.toggle = False
        self.last_button = False
        self.printText = False

    def get_value(self):
        for event in pygame.event.get(): # User did something.
            if event.type == pygame.QUIT: # If user clicked close.
                done = True # Flag that we are done so we exit this loop.
            elif event.type == pygame.JOYBUTTONDOWN:
                if self.printText:
                    print("Joystick button pressed.")
                self.last_button = True
            elif event.type == pygame.JOYBUTTONUP:
                if self.printText:
                    print("Joystick button released.")
                if self.last_button == True:
                    self.toggle = not self.toggle
                self.last_button == False

        self.screen.fill(WHITE)
        self.textPrint.reset()
        # Get count of joysticks.
        joystick_count = pygame.joystick.get_count()

        # For each joystick:
        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            axis0 = joystick.get_axis(0)
            axis1 = joystick.get_axis(1)
            axis2 = joystick.get_axis(2)
            axis3 = joystick.get_axis(3)
            axis4  = joystick.get_axis(4)
            axis5  = joystick.get_axis(5)

            self.axis = [axis0, axis1, axis2, axis3, axis4,axis5]

            if self.printText:
                self.textPrint.tprint(self.screen, "Joystick {}".format(i))
                self.textPrint.indent()
                self.textPrint.tprint(self.screen, "Axis {} value: {:>6.3f}".format(0, axis0))
                self.textPrint.tprint(self.screen, "Axis {} value: {:>6.3f}".format(1, axis1))
                self.textPrint.tprint(self.screen, "Axis {} value: {:>6.3f}".format(2, axis2))
                self.textPrint.tprint(self.screen, "Axis {} value: {:>6.3f}".format(3, axis3))
                self.textPrint.tprint(self.screen, "Axis {} value: {:>6.3f}".format(4, axis4))
                self.textPrint.tprint(self.screen, "Axis {} value: {:>6.3f}".format(5, axis5))


        # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT
        #
        # Go ahead and update the screen with what we've drawn.
        if self.printText:
            pygame.display.flip()
            # Limit to 20 frames per second.
            self.clock.tick(20)

def main():
    joy = Joystick()
    joy.printText = True
    while not joy.done:
        joy.get_value()
        # print(joy.toggle)
        # print(joy.axis)
    pygame.quit()


if __name__ == "__main__":
    main()