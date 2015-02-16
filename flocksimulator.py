import sgc
from sgc.locals import *
import pygame, sys, random
from pygame.locals import *
from world import World
import cProfile


BLACK = (0,0,0)
WHITE = (255,255,255)
BG_COLOR = WHITE

#Screen width and height
width, height = 1200, 600
dim = [width, height]

nr_of_boids = 200

pygame.init()

screen = pygame.display.set_mode(dim)
pygame.display.set_caption('Boids')
controls= sgc.surface.Screen(dim)


#Slider gui elements
sep_slider = sgc.Scale((100,35),label_col=BLACK,label="Seperation", pos=(10, 10), min=0, max=200, min_step=1)
sep_slider.add(0)
coh_slider = sgc.Scale((100,35),label_col=BLACK ,label="Cohesion", pos=(10, 50), min=0, max=100, min_step=1,)
coh_slider.add(0)
align_slider = sgc.Scale((100,35),label_col=BLACK,label="Aligmnent", pos=(10, 90), min=0, max=100, min_step=1,)
align_slider.add(0)
avoid_slider = sgc.Scale((100,35),label_col=BLACK,label="Avoidance", pos=(10, 130), min=0, max=1000, min_step=1)
avoid_slider.add(0)


#World init and populating
world = World(screen, coh_slider,align_slider, sep_slider, avoid_slider)
world.populate(nr_of_boids)


#Could abstract, make each creature decide what to do, and let another class
#Calculate the new absolute position
def run_simulation():
    clock = pygame.time.Clock()
    number = 0
    stopping = False
    while not stopping:
        time_passed = clock.tick(30)

        #Event handling
        for event in pygame.event.get():
            sgc.event(event)
            if event.type == QUIT:
                stopping = True
            if event.type == pygame.KEYDOWN:
                if event.key == K_o:
                    world.add_obstacle(pygame.mouse.get_pos())
                if event.key == K_p:
                    world.add_predator(pygame.mouse.get_pos())
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 3:
                    world.remove_obstacle(pygame.mouse.get_pos())
        screen.fill(BG_COLOR)


        #Update  and redraw all creatures on screen
        for creature in world.all_things:
            creature.update_velocity()

        world.all_things.update(time_passed)
        world.all_things.draw(screen)
        sgc.update(time_passed)
        #When all object is drawn, the graphics is rendered
        pygame.display.update()

        dtime = time_passed / 1000
        fps = 1 / dtime
        if number % 10 == 0:
            print("FPS: ", fps)
        number += 1
    pygame.quit()
    sys.exit()
#run_simulation()
cProfile.run('run_simulation()')





