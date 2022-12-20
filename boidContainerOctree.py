import open3d as o3d
import numpy as np
from data_structures.octree import Octree, OctNode

class BoidContainerOctree:
    def __init__(self, worldSize = 2.1, origin=(0.0, 0.0, 0.0), max_type="nodes", max_value=10):
        self.container = Octree(worldSize, origin, max_type, max_value)
        self.blist = []

    def add(self, boid):
        self.container.insertNode(boid.pos, boid)
        self.blist.append(boid)

    def getNeighborhood(self, boid_i, radius_detection, radius_collision):
        return self.container.findNeighborhoodBoids(boid_i.pos, radius_detection), \
               self.container.findNeighborhoodBoids(boid_i.pos, radius_collision)

    #Rebuilds the tree after each step for now, will be updated to doing only the neccessary
    #re-structuring
    def step(self, viz):
        for nr, boid in enumerate(self.blist):
            boid.step(self.getNeighborhood, viz, nr)
        self.container= Octree(2.1, (0.0, 0.0, 0.0), "nodes", 10)
        for boid in self.blist:
            self.container.insertNode(boid.pos, boid)




    # ----------------- additional methods mainly for debugging -----------------

    def getNeighborhoodNode(self, boid_i, radius_detection, radius_collision):
        return self.container.findNeighborhoodNode(boid_i.pos, radius_detection, radius_collision)

    def iterate(self):
        for i, x in enumerate(self.container.iterateDepthFirst()):
            print(i, ":", x)

    def find(self, boid):
        return self.container.findPosition(boid.pos)

    def getRoot(self):
        return self.container.root


