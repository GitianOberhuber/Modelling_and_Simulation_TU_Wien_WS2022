import numpy as np

class BoidContainerList:
    def __init__(self):
        self.container = list()

    #adds a boid to the datastructure
    def add(self, boid):
        self.container.append(boid)

    #returns neighborhoods for detection- as well as collission-radii (collision-radius <= detection-radius)
    def getNeighborhood(self, boid_i, radius_detection, radius_collision):
        res_detection = list()
        res_collision = list()
        for boid_j in self.container:
            if (np.linalg.norm(boid_i.pos - boid_j.pos) <= radius_detection):
                res_detection.append(boid_j)
                if (np.linalg.norm(boid_i.pos - boid_j.pos) <= radius_collision):
                    res_collision.append(boid_j)

        return res_detection, res_collision

    #calls step of all boids in the container
    def step(self):
        for boid in self.container:
            boid.step(self.getNeighborhood)



