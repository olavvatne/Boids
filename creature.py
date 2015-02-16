from pygame.sprite import Sprite
import pygame, random
import numpy as np
import math

MAX_PRED_VELOCITY = 0.15
MAX_BOID_VELOCITY = 0.1
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
        self.rect.x = float(x)
        self.rect.y = float(y)
        self.pos = np.array([float(x)+width/2,float(y)+height/2])

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
        pygame.draw.ellipse(self.image, (0, 255, 0), [0, 0, self.radius*2-5, self.radius*2-5])



class Creature(Thing):
    def __init__(self,world, y, x, velocity, radius, sight):
        self.radius = radius
        self.velocity = velocity
        self.prev_velocity = velocity

        self.sight = sight
        super(Creature, self).__init__(world, y, x, radius*2, radius*2)

    def draw_creature(self):
        self.image.fill((0,0,0))
        pygame.draw.ellipse(self.image, self.color, [0, 0, self.radius*2, self.radius*2])
        pygame.draw.aaline( self.image, (1,1,1), (self.radius,self.radius), self.get_orientation(), 2)

    def set_new_pos(self, new_pos):
        old_pos = self.pos
        self.pos = new_pos
        self.rect.x = self.pos[0]-self.radius
        self.rect.y = self.pos[1]-self.radius
        if isinstance(self, Boid):
            self.world.set_new_element_position(old_pos-self.radius, new_pos-self.radius, self) #A way to do this exploting inheritenace i

    def perpendicular(self, a ) :
        b = np.empty_like(a)
        b[0] = -a[1]
        b[1] = a[0]
        return b

    def calc_repel_force(self, obstacles):
        #The idea is to check if any of the obstacles is on a collisiion course by using a lookahead vector. A avoidance force
        #is created for the most threatning obstacle.
        norm_base = self.prev_velocity/np.linalg.norm(self.prev_velocity)
        perp_norm = self.perpendicular(norm_base) * self.radius
        #edge1 = perp_norm + self.pos
        #edge2 = -perp_norm + self.pos
        ahead = self.pos + norm_base * self.sight
        ahead2 = (norm_base * self.sight *0.5) + self.pos
        #ahead3 = (norm_base * self.sight) + edge2
        #ahead4 = (norm_base * self.sight *0.5) + edge2
        #pygame.draw.aaline(self.world.screen,(1,255,1), self.pos,ahead,21)
        threat = None
        largest_distance = float("inf")
        for obstacle in obstacles:
            diff = self.pos - obstacle.pos
            distance = np.linalg.norm(diff)
            if self.line_intersect_sphere(ahead, ahead2, obstacle) or distance < obstacle.radius + self.radius:
                if largest_distance > distance:
                    threat = obstacle
                    largest_distance = distance
        if threat:
            force = ahead - threat.pos
            magnitude = (np.linalg.norm(force))
            if magnitude < 0:
                force[1] = 0
            return force/magnitude
        else:
            return np.array([0.0,0.0])


    def obstacle_intersect(self, obstacle):
        return np.linalg.norm(self.pos - obstacle.pos) < self.radius + obstacle.radius

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
        if len(n) > 0:
            force = np.array([0.0,0.0])
            for b in n:
                diff = self.pos - b.pos
                distance = np.linalg.norm(diff)
                force = force + (diff/distance) #The force contribution of each individual boids is inverse of the distance
            force = force/len(n)
            return force
        return np.array([0.0,0.0])

    def calc_cohesion_force(self, n):
        if len(n) > 0:
            avg_pos = np.array([0.0,0.0])
            for b in n:
                avg_pos = avg_pos + b.pos
            avg_pos = avg_pos/len(n)
            force = (avg_pos - self.pos)/100 #Moves boid towards percieved center by 1% each time
            return force
        return np.array([0.0,0.0])

    def calc_alignment_force(self, n):
        if len(n) > 0:
            avg_velocity = np.array([0.0,0.0])
            for b in n:
                avg_velocity = avg_velocity + b.prev_velocity
            avg_velocity = avg_velocity/len(n)
            return (avg_velocity - self.prev_velocity)/8 #TODO: is the 8 needed??
        return np.array([0.0,0.0])


    def calc_obstacle_force(self, obstacles):
        avoid = self.world.get_avoidance_weight() * self.calc_repel_force(obstacles)
        return avoid



class Boid(Creature):
    def __init__(self, world, y, x, radius, velocity, sight):
        #Move velocity up to creature, and get_orientation
        self.color = RED
        super(Boid, self).__init__(world, y, x, velocity, radius, sight)
        self.draw_creature()

    def update(self, time_passed):
        self.set_new_pos((self.pos + (self.velocity*time_passed)) % np.array(self.world.get_size()))
        self.draw_creature()

    def update_velocity(self):
        self.prev_velocity = self.velocity
        neighbors = self.prune(self.world.get_close_neighbors(self, True), MAX_NEIGHBORS)
        obstacles = self.world.get_close_obstacles(self)
        predators = self.world.get_close_predators(self)
        self.velocity = self.velocity + self.calc_flocking_force(neighbors) + self.calc_obstacle_force(obstacles) + self.calc_seperation_force(predators)
        self.velocity = self.velocity/np.linalg.norm(self.velocity) * MAX_BOID_VELOCITY

    def calc_flocking_force(self, neighbors):
        sep = self.world.get_seperation_weight() * self.calc_seperation_force(neighbors)
        coh = self.world.get_cohesion_weight() * self.calc_cohesion_force(neighbors)
        align = self.world.get_alignment_weight() * self.calc_alignment_force(neighbors)
        return sep + align + coh



class Predator(Creature):

    SEP_WEIGHT = 0.05
    COH_WEIGHT = 0.50
    ALIGN_WEIGHT = 0.3

    def __init__(self, world, y, x, radius, velocity, sight):
        #Move velocity up to creature, and get_orientation
        self.color = GREEN
        super(Predator, self).__init__(world, y, x, velocity, radius, sight)
        self.draw_creature()

    def update(self, time_passed):
        self.set_new_pos((self.pos + self.velocity*time_passed) % np.array(self.world.get_size()))
        self.draw_creature()

    def calc_chase_force(self, neighbors):
        sep = self.SEP_WEIGHT * self.calc_seperation_force(neighbors)

        coh = self.COH_WEIGHT * self.calc_cohesion_force(neighbors)

        align = self.ALIGN_WEIGHT * self.calc_alignment_force(neighbors)
        return sep + align + coh

    def update_velocity(self):
        self.prev_velocity = self.velocity
        neighbors = self.prune(self.world.get_close_neighbors(self, False), MAX_NEIGHBORS)
        obstacles = self.world.get_close_obstacles(self)
        self.velocity = self.velocity +  self.calc_chase_force(neighbors) + self.calc_obstacle_force(obstacles)
        self.velocity = self.velocity/np.linalg.norm(self.velocity) * MAX_PRED_VELOCITY