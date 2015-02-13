from pygame.sprite import Sprite
import pygame, random
import numpy as np
import math

MAX_VELOCITY = .1
MAX_NEIGHBORS = 20
RED = (255,0,0)
GREEN = (0,255,0)

class Thing(Sprite):
    id_counter = 0

    def __init__(self,world, y, x, width, height):
        super().__init__()
        self.thing_id = str(Thing.id_counter)
        Thing.id_counter += 1
        self.world = world
        self.image = pygame.Surface((width, height))
        self.image.set_colorkey((0,0,0))
        self.rect = self.image.get_rect()
        self.rect.x = x
        self.rect.y = y
        self.pos = np.array([x+width/2,y+height/2])

    def update(self, time_passed):
       pass

    def update_velocity(self):
        pass

    def normalize(self, value):
        if np.any(value):
            return value/np.linalg.norm(value)
        else:
            return value

    def prune(self, elements, k):
        '''
        When there are to many elements in a list, this method returns a random sample of it.
        A reasonable assumption when there are to many boids in the neighborhood for example. The boid
        will only adjust itself according to a max number of boids it can keep track of at any time.
        :param elements: A list that should be pruned
        :param elements: threshold for pruning
        :return: A sample of k elements taken with no replacement
        '''
        if len(elements) > k:
            return random.sample(elements, k)
        return elements

    def __repr__(self):
        return 'id: ' + str(self.thing_id) + ' x: ' + str(self.rect.x) + ' y: ' + str(self.rect.y)



class Obstacle(Thing):
    def __init__(self, world, y, x, radius):
        super().__init__(world, y, x, radius*2, radius*2)
        self.radius = radius
        self.draw_obstacle()

    def draw_obstacle(self):
        self.image.fill((0,0,0))
        pygame.draw.ellipse(self.image, (0, 255, 0), [0, 0, self.radius*2, self.radius*2])



class Creature(Thing):
    def __init__(self,world, y, x, velocity, radius, sight):
        self.radius = radius
        self.velocity = velocity
        self.prev_velocity = velocity

        self.sight = sight
        super(Creature, self).__init__(world, y, x, radius*2, radius*2)

    def set_new_pos(self, new_pos):
        old_pos = self.pos
        self.pos = new_pos
        self.rect.x = self.pos[0]-self.radius
        self.rect.y = self.pos[1]-self.radius
        self.world.set_new_element_position(old_pos-self.radius, new_pos-self.radius, self)

    def calc_obstacle_avoidance(self, obstacles):
        #The idea is to check if any of the obstacles is on a collisiion course by using a lookahead vector. A avoidance force
        #is created for the most threatning obstacle.
        base = self.prev_velocity/np.linalg.norm(self.prev_velocity) * self.sight
        ahead = self.pos + base
        ahead2 = (base*0.5) + self.pos
        #pygame.draw.aaline(
        #    self.world.screen,
        #    (1,255,1),
        #    self.pos,
        #    ahead2,
        #    21
        #    )
        threat = None
        for obstacle in obstacles:
            if self.line_intersect_sphere(ahead, ahead2, obstacle):
                print(obstacle)
                threat = obstacle
                #TODO: Need to find most threatning obstacle
                break
        if threat:
            #print(threat)
            #print(ahead)
            force = ahead - threat.pos
            #pygame.draw.aaline(
            #self.world.screen,
            #(1,1,255),
            #self.pos,
            #self.pos+force,
            #21
            #)
            return force/(np.linalg.norm(force))
        else:
            return np.array([0.0,0.0])

    def line_intersect_sphere(self, v1, v2, o1):
        return np.linalg.norm(v1-o1.pos) <= o1.radius or np.linalg.norm(v2-o1.pos) <= o1.radius

    def update(self, time_passed):
        pass

    def get_orientation(self):
        if not np.all(self.velocity==0):
            orientation = self.velocity/np.linalg.norm(self.velocity)*self.radius
            center = self.radius
            return orientation + center
        else:
            return np.array([self.radius, self.radius])

    def calc_seperation_force(self, n):
        force = np.array([0.0,0.0])
        if len(n) > 0:
            for b in n:
                diff = self.pos - b.pos
                distance = np.linalg.norm(diff)
                force = force + (diff/distance)
            force = force/len(n)
            #print("sep" + str(force))
            return force
        return force


    def calc_cohesion_force(self, n):
        force = np.array([0.0,0.0])
        if len(n) > 0:
            for b in n:
                force = force + b.pos
            force = force/len(n)
            #print("coh" + str(force-self.pos))
            return force - self.pos
        return force

    def calc_alignment_force(self, n):
        force = np.array([0.0,0.0])
        if len(n) > 0:
            for b in n:
                force = force + b.prev_velocity
            force = force/len(n)
            #print("align" + str(force-self.velocity))
            return force - self.prev_velocity
        return force



class Boid(Creature):
    def __init__(self, world, y, x, radius, velocity, sight):
        #Move velocity up to creature, and get_orientation
        self.color = RED
        super(Boid, self).__init__(world, y, x, velocity, radius, sight)
        self.drawBoid()

    def drawBoid(self):
        self.image.fill((0,0,0))
        pygame.draw.ellipse(self.image, self.color, [0, 0, self.radius*2, self.radius*2])
        pygame.draw.aaline(
            self.image,
            (1,1,1),
            (self.radius,self.radius),
            self.get_orientation(),
            2
            )

    def update(self, time_passed):
        self.set_new_pos((self.pos + self.velocity*time_passed) % np.array(self.world.get_size()))
        self.drawBoid()

    def update_velocity(self):
        self.prev_velocity = self.velocity
        neighbors = self.prune(self.world.get_close_neighbors(self), MAX_NEIGHBORS)
        obstacles = self.world.get_close_obstacles(self)

        sep = self.world.get_seperation_weight() * self.calc_seperation_force(neighbors)
        coh = self.world.get_cohesion_weight() * self.calc_cohesion_force(neighbors)
        align = self.world.get_alignment_weight() * self.calc_alignment_force(neighbors)
        avoid = self.world.get_avoidance_weight() * self.calc_obstacle_avoidance(obstacles)
        self.velocity += sep + align + coh + avoid
        self.velocity = np.clip(self.velocity, -MAX_VELOCITY, MAX_VELOCITY)



class Predator(Creature):
    def __init__(self, world, y, x, radius, velocity, sight):
        #Move velocity up to creature, and get_orientation
        self.color = GREEN
        super(Predator, self).__init__(world, y, x, velocity, radius, sight)
        self.drawPredator()