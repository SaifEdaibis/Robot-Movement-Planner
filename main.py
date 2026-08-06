import pygame

pygame.init()

#--- Constants -------------------------------------------------------------------------

SCREEN_WIDTH = 400
SCREEN_HEIGHT = SCREEN_WIDTH
background_color = (0, 0, 0)

framerate = 60

grid_dimension = 40
grid_color = (255, 255, 255)
grid_line_width = 2

#--- Main Screen -----------------------------------------------------------------------------------------

class Front_Display:
    def __init__(self):

        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption('Demo')


        self.font = pygame.font.Font(None, 18)

    def grid(self, screen):
        for line in range(int((SCREEN_HEIGHT / grid_dimension)-1)):
            start_pos = (0, grid_dimension * (line +1))
            end_pos = (SCREEN_WIDTH, grid_dimension * (line +1) )
            pygame.draw.line(screen, grid_color, start_pos, end_pos, grid_line_width)

        for line in range(int((SCREEN_WIDTH / grid_dimension)-1)):
            start_pos = (grid_dimension * (line +1), 0)
            end_pos = ( grid_dimension * (line +1), SCREEN_HEIGHT)
            pygame.draw.line(screen, grid_color, start_pos, end_pos, grid_line_width)

        

#--- Application -----------------------------------------------------------------------------------------------

class Application:
    def __init__(self):
        self.display_front = Front_Display()
        self.run = True

        self.clock = pygame.time.Clock()

    def timing(self):
        self.dt = self.clock.tick(framerate) / 1000

    ### processess and responds to all major user inputs
    def process_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:                      
                self.run = False

    ### The main_loop that draws and calls every repeated functions
    def main_loop(self):    
        while self.run:
            
            self.display_front.screen.fill(background_color)

            self.timing()
            self.process_events()


            self.display_front.grid(self.display_front.screen)
            
            pygame.display.update()
            



Main = Application()
Main.main_loop()

pygame.quit()