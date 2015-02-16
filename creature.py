from pygame.sprite import Sprite
import pygame, random
import numpy as np

#Constants
MAX_PRED_VELOCITY = 0.15
MAX_BOID_VELOCITY = 0.1
ZERO_ARRAY = np.array([0.0, 0.0])
MAX_NEIGHBORS = 15
RED = (255,0,0)
GREEN = (0,255,0)

#Lookup dictionary for distances to reduce cost of calc seperation force
distance_lookup = {}

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
        '''
        Draw method for the obstacle. A filled circle
        '''
        self.image.fill((0,0,0))
        pygame.draw.ellipse(self.image, GREEN, [0, 0, self.radius*2-8, self.radius*2-8])



class Creature(Thing):
    def __init__(self,world, y, x, velocity, radius, sight):
        self.radius = radius
        self.velocity = velocity
        self.prev_velocity = velocity

        self.sight = sight
        super(Creature, self).__init__(world, y, x, radius*2, radius*2)

    def draw_creature(self):
        '''
        Update the Sprite image, by drawing a filled circle and a line showing the direction the creature is heading.
        '''
        self.image.fill((0,0,0))
        pygame.draw.ellipse(self.image, self.color, [0, 0, self.radius*2, self.radius*2])
        pygame.draw.aaline( self.image, (1,1,1), (self.radius,self.radius), self.get_orientation(), 2)

    def set_new_pos(self, new_pos):
        '''
        By using the creatures velocity a new position on the map is calculated. In case the creature is a boid
        '''
        old_pos = self.pos
        self.pos = new_pos
        self.rect.x = self.pos[0]-self.radius
        self.rect.y = self.pos[1]-self.radius
        self.notify_of_position_change(old_pos, new_pos)

    def notify_of_position_change(self, old_pos, new_pos):
        pass

    def calc_repel_force(self, obstacles):
        '''
        Repel force creates a force designed to avoid obstacles in the boids field of view. To ensure that the motion is
        smooth a look ahead vector is used to check intersecting obstacles. If there is a threat in front, the same vector
        and the obstacles position is used to create a new vector scaled by the force's manitude. If the threat is close
        the force will have a bigger influence on the creature's velocity. A create will respond only to the closest threat
        at each velocity update.
        '''
        norm_base = self.prev_velocity/np.linalg.norm(self.prev_velocity)
        ahead = self.pos + norm_base * self.sight
        ahead2 = (norm_base * self.sight *0.5) + self.pos
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
            return ZERO_ARRAY

    def line_intersect_sphere(self, v1, v2, o1):
        return np.linalg.norm(v1-o1.pos) <= o1.radius or np.linalg.norm(v2-o1.pos) <= o1.radius

    def update(self, time_passed):
        pass

    def get_orientation(self):
        '''
        Returns a orientation vector for drawing purposes. A normalized velocity vector is scaled by the creature's radius
        and offset by the center of creature.
        '''
        if not np.all(self.velocity==0):
            orientation = self.velocity/np.linalg.norm(self.velocity)*self.radius
            center = self.radius
            return orientation + center
        else:
            return np.array([self.radius, self.radius])

    def get_distance(self, diff):
        '''
        A small enhancement of seperation force calculations. The np.linalg.norm operation is quite expensive, and since
        there are a finite amount of integer distance vectors, all scalar distance calculations are put in a dictionary.
        '''
        aprox_diff = str(int(diff[0])) + " " + str(int(diff[1]))
        if aprox_diff in distance_lookup:
            return distance_lookup[aprox_diff]
        else:
            distance = np.linalg.norm(diff)
            distance_lookup[aprox_diff] = distance
            return distance

    def calc_seperation_force(self, n):
        '''
        A seperation or repel force is calculated based on creatures in the creature's neighborhood. One of the simple
        steering behaviors described by Craig Reynolds. The diff vector between the current creature and it's neighbors
        are inversely scaled by the distance. The sum of these vectors are then divided by the number of creatures in the neighborhood
        resulting in a force that encourage more seperation from the other creatures in the neighborhood
        '''
        if len(n) > 0:
            force = ZERO_ARRAY
            for b in n:
                diff = self.pos - b.pos
                distance = self.get_distance(diff)
                force = force + (diff*(1/distance)) #The force contribution of each individual boids is inverse of the distance
            force = force/len(n)
            return force
        return ZERO_ARRAY

    def calc_cohesion_force(self, n):
        '''
        The calculated cohesion force encourage closeness between the creature and it's neighbors. THe average position
        of the neighborhood is found and a vector pointing towards the center is returned by the method. The force can
        be added to the creature's velocity and will move the creature towards the percieved center by 1% each time
        the velocity is updated.
        '''
        if len(n) > 0:
            avg_pos = ZERO_ARRAY
            for b in n:
                avg_pos = avg_pos + b.pos
            avg_pos = avg_pos/len(n)
            force = (avg_pos - self.pos)/100 #Moves boid towards percieved center by 1% each time
            return force
        return ZERO_ARRAY

    def calc_alignment_force(self, n):
        '''
        The alignment force finds the average velocity in the creature's neighborhood and return a force that can move
        the velocity of the creature towards the average velocity. This force will steer the creature towards the average
        heading of it's neighbors.
        '''
        if len(n) > 0:
            avg_velocity = ZERO_ARRAY
            for b in n:
                avg_velocity = avg_velocity + b.prev_velocity
            avg_velocity = avg_velocity/len(n)
            return (avg_velocity - self.prev_velocity)/8
        return ZERO_ARRAY


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
        '''
        On each update a new position is calculated based on time_passed since last update, the velocity and the
        position of the boid. To ensure wrap around modulo is used. Then the creature is redrawn. This is necessary
        since the direction of the boid might have changed, and need to be updated.
        '''
        self.set_new_pos((self.pos + (self.velocity*time_passed)) % np.array(self.world.get_size()))
        self.draw_creature()

    def update_velocity(self):
        '''
        A updated velocity is created by using three simple flocking behavior rules, obstacle and predator avoidance.
        The velocity change is only based on local information readily available by the boids neigborhood. The velocity
        is also normalized and scaled by a max velocity constant to ensure that the velocity is bounded.
        The prune method is used to reduce flocking calculation time. A random sample is used if the neigborhood of the boid
        is to big.
        '''
        self.prev_velocity = self.velocity
        neighbors = self.prune(self.world.get_close_neighbors(self, True), MAX_NEIGHBORS)
        obstacles = self.world.get_close_obstacles(self)
        predators = self.world.get_close_predators(self)
        self.velocity = self.velocity + self.calc_flocking_force(neighbors) + self.calc_obstacle_force(obstacles) + self.calc_seperation_force(predators)
        self.velocity = self.velocity/np.linalg.norm(self.velocity) * MAX_BOID_VELOCITY

    def calc_flocking_force(self, neighbors):
        '''
        Basic flocking behavior is decided by three rules weighted together to create a force vector. Based on Craig
        Reynolds three steering behaviors, seperation, cohesion and alignment.
        '''
        sep = self.world.get_seperation_weight() * self.calc_seperation_force(neighbors)
        coh = self.world.get_cohesion_weight() * self.calc_cohesion_force(neighbors)
        align = self.world.get_alignment_weight() * self.calc_alignment_force(neighbors)
        return sep + align + coh

    def notify_of_position_change(self, old_pos, new_pos):
        '''
        Method overridden from the Creature class. The world needs to do some bookkeeping on the where each boid
        is positioned in the world, to more efficiently calculate the neigborhoods required by each boid.
        '''
        self.world.set_new_element_position(old_pos-self.radius, new_pos-self.radius, self)

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
        '''
        At each update a new position for the predator is created by the last position, time passed in between updates
        and the current velocity of the predator. The predator also has to be redrawn since the direction it's heading
        might have changed, and the direction line must be updated.
        '''
        self.set_new_pos((self.pos + self.velocity*time_passed) % np.array(self.world.get_size()))
        self.draw_creature()

    def calc_chase_force(self, neighbors):
        '''
        The predators chasing behavior is very similar to the boids flocking behavior method. The share the same rules,
        except the rules cant by dynamically changed. The result of these three rules is that the predator is attracted
        to big clusters of boids, and therefore chase boids. The boids on the other hand try to create seperation between
        the predator and itself.
        '''
        sep = self.SEP_WEIGHT * self.calc_seperation_force(neighbors)
        coh = self.COH_WEIGHT * self.calc_cohesion_force(neighbors)
        align = self.ALIGN_WEIGHT * self.calc_alignment_force(neighbors)
        return sep + align + coh

    def update_velocity(self):
        '''
        At each update the velocity of the predator is updated. Force vectors that encourage chasing and obstacle avoidance
        is summed together with the previous velocity to create a new velocity vector. This velocity vector is also bounded
        by the MAX_PRED_VELOCITY, by scaling the normalized velocity vector by this constant.
        '''
        self.prev_velocity = self.velocity
        neighbors = self.prune(self.world.get_close_neighbors(self, False), MAX_NEIGHBORS)
        obstacles = self.world.get_close_obstacles(self)
        self.velocity = self.velocity +  self.calc_chase_force(neighbors) + self.calc_obstacle_force(obstacles)
        self.velocity = self.velocity/np.linalg.norm(self.velocity) * MAX_PRED_VELOCITY