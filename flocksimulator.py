import sgc
from sgc.locals import *
import pygame, sys, random
from pygame.locals import *
from world import World

class Slider(sgc.Scale):
    def __init__(self, **kwargs):
        super().__init__((100,35), label_col=BLACK, **kwargs)
        self.add(0)


BLACK = (0,0,0)

WHITE = (255,255,255)
BG_COLOR = WHITE

#Screen width and height
width, height = 800, 480
dim = [width, height]

nr_of_boids = 10
nr_of_predators = 2

boid_radius = 8.0
boid_sight = 80.0

pygame.init()

screen = pygame.display.set_mode(dim)
controls= sgc.surface.Screen(dim)
sep_slider = Slider(label="Seperation", pos=(10, 10), min=0, max=200, min_step=1)
coh_slider = Slider(label="Cohesion", pos=(10, 50), min=0, max=50, min_step=1,)
align_slider = Slider(label="Aligmnent", pos=(10, 90), min=0, max=50, min_step=1,)
avoid_slider = Slider(label="Avoidance", pos=(10, 130), min=0, max=100, min_step=1)

pygame.display.set_caption('Boids')

world = World(screen, coh_slider,align_slider, sep_slider, avoid_slider)
world.populate(nr_of_boids,boid_radius,boid_sight)


#Could abstract, make each creature decide what to do, and let another class
#Calculate the new absolute position
clock = pygame.time.Clock()

stopping = False
while not stopping:
    time_passed = clock.tick(30)

    #Event handling
    for event in pygame.event.get():
        sgc.event(event)
        if event.type == QUIT:
            stopping = True
        if event.type == MOUSEBUTTONDOWN:
            if event.button == 3:
                world.add_obstacle(pygame.mouse.get_pos())

        #if event.type == GUI:
        #    pass

    screen.fill(BG_COLOR)

    #Update  and redraw all creatures on screen
    for creature in world.all_things:
        creature.update_velocity()

    world.all_things.update(time_passed)
    world.all_things.draw(screen)
    sgc.update(time_passed)
    #When all object is drawn, the graphics is rendered
    pygame.display.update()




pygame.quit()
sys.exit()


