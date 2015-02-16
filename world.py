import pygame, random
import numpy as np
from creature import Boid, Obstacle, Predator
import math


OBSTACLE_RADIUS = 25
PREDATOR_RADIUS = 10.0
BOID_RADIUS = 8.0

GRID_WIDTH = 10

BOID_SIGHT = 80.0
PREDATOR_SIGHT = 100.0

class World(object):



    def __init__(self, screen, coh, align, sep, avoid):
        '''
        The world of all objects has some properties, like dimension
        and weights for seperation alignment and cohesion. Boids
        will use these variables to calculate their new positions.
        All boids use the same weights
        '''
        self.screen = screen
        width, height = self.get_size()
        self.grid =Grid(width, height)
        self.seperation = sep
        self.seperation.value = 0
        self.alignment = align
        self.alignment.value = 0
        self.cohesion = coh
        self.cohesion.value = 0
        self.avoid = avoid
        self.avoid.value = 10
        self.all_things = pygame.sprite.Group()
        self.boids = pygame.sprite.Group()
        self.predators = pygame.sprite.Group()
        self.obstacles = pygame.sprite.Group()

    def populate(self, nr_of_boids, boid_radius = BOID_RADIUS, boid_sight = BOID_SIGHT):
        width, height = self.get_size()
        for i in range(nr_of_boids):
            x = random.randrange(width)
            y = random.randrange(height)
            v = np.array([random.uniform(-15, 15), random.uniform(-15, 15)])
            boid = Boid(self, y, x, boid_radius, v, boid_sight)
            self.grid.add_element(y,x, boid)
            self.boids.add(boid)
            self.all_things.add(boid)

    def add_obstacle(self, pos):
        obstacle = Obstacle(self, pos[1]-OBSTACLE_RADIUS, pos[0]-OBSTACLE_RADIUS, OBSTACLE_RADIUS)
        self.all_things.add(obstacle)
        self.obstacles.add(obstacle)

    def remove_obstacle(self, pos):
        for obstacle in self.obstacles:
            if obstacle.rect.collidepoint(pos):
                obstacle.image.fill((0,0,0))
                self.obstacles.remove(obstacle)

    def add_predator(self, pos):
        v = np.array([random.uniform(-15, 15), random.uniform(-15, 15)])
        predator = Predator(self, pos[1], pos[0], PREDATOR_RADIUS, v, PREDATOR_SIGHT)
        self.all_things.add(predator)
        self.predators.add(predator)
        print("PREDATOR")
        print(predator.pos)

    def get_cohesion_weight(self):
        #print(self.cohesion.value)
        return self.cohesion.value/100

    def get_alignment_weight(self):
        #print(self.cohesion.value)
        return self.alignment.value/100

    def get_seperation_weight(self):
        #print(self.cohesion.value)
        return self.seperation.value/100
        return self.seperation.value/100

    def get_avoidance_weight(self):
        return self.avoid.value/100

    def get_size(self):
        '''
        Neccesary for updating position of all moving objects, since the world
        wraps around.
        '''
        return self.screen.get_size()

    def get_close_neighbors(self, creature, remove_self):
        '''
        Returns all neighboring boids close to the creature. The neighborhood
        will determine the next position of the creature
        '''
        neighborhood = []
        #for b in self.boids:
        #    d = (b.rect.centerx-creature.rect.centerx)**2 + (b.rect.centery - creature.rect.centery)**2
        #    if d < creature.sight**2:
        #        neighborhood.append(b)
        neighborhood = self.grid.get_all_elements(creature.pos[1], creature.pos[0], creature.sight)
        if remove_self:
            neighborhood.remove(creature)
        return neighborhood

    def get_close_predators(self, creature):
        '''
        The world return all predators detected by the creature's line of sight.
        If a predator is close by, the flight path of creature will be influenced
        '''
        close_predators = []
        for predator in self.predators:
            distance = np.linalg.norm(predator.pos-creature.pos)
            if distance < creature.sight:
                close_predators.append(predator)
        return close_predators

    def get_close_obstacles(self, boid):
        neighborhood = []
        for b in self.obstacles:
            d = (b.rect.centerx-boid.rect.centerx)**2 + (b.rect.centery - boid.rect.centery)**2
            if d < boid.sight**2:
                neighborhood.append(b)
        return neighborhood

    def set_new_element_position(self, old_pos, new_pos, element):
        self.grid.move_element(old_pos[1], old_pos[0], new_pos[1], new_pos[0], element)


class Grid(object):

    def __init__(self, width, height, square_dim=40):
        self.grid = []
        self.square_dim = square_dim
        self.nr_of_rows = (int)(height/square_dim)
        self.nr_of_cols = (int)(width/square_dim)
        for i in range(self.nr_of_rows):
            row = []
            for j in range(self.nr_of_cols):
                row.append({})
            self.grid.append(row)

    def add_element(self, y, x, element):
        grid_y = (int)(y/self.square_dim)
        grid_x = (int)(x/self.square_dim)
        self.grid[grid_y][grid_x][element.thing_id] = element

    def remove_element(self, y, x, element):
        grid_y = (int)(y/self.square_dim)
        grid_x = (int)(x/self.square_dim)
        self.grid[grid_y][grid_x][element.thing_id] = None
        del  self.grid[grid_y][grid_x][element.thing_id]

    def move_element(self, oy, ox, ny, nx, element):
        self.remove_element(oy, ox, element)
        self.add_element(ny, nx, element)

    def get_all_elements(self, y,x, radius):
        grid_y = (int)(y/self.square_dim)
        grid_x = (int)(x/self.square_dim)
        r = math.ceil(radius/self.square_dim)
        elements = []
        for i in range(-r, r):
            for j in range(-r, r):
                tx = (grid_x + j)
                ty = grid_y + i
                if ty >= 0 and ty < self.nr_of_rows and tx >= 0 and tx < self.nr_of_cols and r**2 >= i**2 + j**2:
                    elements.extend(self.grid[ty][tx].values())
        return elements

