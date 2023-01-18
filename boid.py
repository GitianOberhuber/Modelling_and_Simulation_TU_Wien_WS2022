import numpy as np


class Boid():
    # static class variables can be read and set using Boid.variablename
    #experimentation
    vmax = 0.03
    do, dc = 0.2, 0.1
    l0, l1, l2, l3, l4 = 1, 0.001, 1, 1, 0.01

    #from exercise description
    #vmax = 0.03
    #do, dc = 0.02, 0.01
    #l0, l1, l2, l3, l4 = 0.31, 0.001, 1.2, 2, 0.01

    def __init__(self, pos, velocity, idx):

        self.pos = np.array(pos)
        self.velocity = velocity
        self.id = idx

    # tendency towards average point of neighbors
    def vector_avgPoint(self, neighborhood_boids):

        neighborhood_positions = np.array([boid.pos for boid in neighborhood_boids])
        return np.average(neighborhood_positions, axis=0) - self.pos

    # tendency towards average velocity of neighbors
    def vector_avgVelocity(self, neighborhood_boids):

        neighborhood_velocities = np.array([boid.velocity for boid in neighborhood_boids])
        return np.average(neighborhood_velocities, axis=0)

    # tendency away from (colission-radius)-neighbours
    def vector_avoidCollision(self, neighborhood_boids):

        neighborhood_positions = np.array([boid.pos for boid in neighborhood_boids])
        return np.average(neighborhood_positions - self.pos, axis=0)

    # tendency to stay in cube
    def vector_stayInCube(self):
        if (np.max(np.abs(self.pos)) <= 1):
            return 0
        else:
            return np.negative(self.pos)

    def step(self, neighborhood_function, viz, nr):
        neighbors_detection, neighbors_collision = neighborhood_function(self, Boid.do, Boid.dc)

        self.velocity = self.velocity * Boid.l0 + \
                        self.vector_avgPoint(neighbors_detection) * Boid.l1 + \
                        self.vector_avgVelocity(neighbors_detection) * Boid.l2 - \
                        self.vector_avoidCollision(neighbors_collision) * Boid.l3 + \
                        self.vector_stayInCube() * Boid.l4

        if (np.linalg.norm(self.velocity) > Boid.vmax):
            self.velocity = Boid.vmax * (self.velocity / np.linalg.norm(self.velocity))

            # In case the velocity gets 0
            self.velocity = np.nan_to_num(self.velocity)

        self.pos = self.pos + self.velocity
        
        # update vizualisation matrix
        if (not viz is None):
            viz[0:3,nr] = self.pos
            viz[3:6,nr] = self.velocity
