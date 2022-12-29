import open3d as o3d
import numpy as np
import time
from data_structures.octree import Octree, OctNode


class BoidContainerOctree:
    def __init__(self, worldSize=4.0, origin=(0.0, 0.0, 0.0), max_type="nodes", max_value=50, rebuild = False):
        self.container = Octree(worldSize, origin, max_type, max_value)
        self.max_value=max_value
        self.max_type=max_type
        self.blist = []
        self.worldSize = worldSize
        if rebuild:
            self.step = self.__step_rebuild
        else:
            self.step = self.__step_update

    def add(self, boid):
        self.container.insertNode(boid.pos, boid)
        self.blist.append(boid)

    def getNeighborhood(self, boid_i, radius_detection, radius_collision):
        return self.container.findNeighborhoodBoids(boid_i.pos, radius_detection), \
            self.container.findNeighborhoodBoids(boid_i.pos, radius_collision)

    # Rebuilds the tree after each step for now, will be updated to doing only the neccessary
    # re-structuring
    def __step_rebuild(self, viz):
        for nr, boid in enumerate(self.blist):
            boid.step(self.getNeighborhood, viz, nr)
        self.container = Octree(self.worldSize, (0.0, 0.0, 0.0), self.max_type, self.max_value)
        for boid in self.blist:
            self.container.insertNode(boid.pos, boid)

    def __step_update(self, viz):
        boids_to_move = []
        nodes_of_removed = set()

        # check which boids are outside their original node
        for idx, node in enumerate(self.container.iterateDepthFirst()):
            if node.isLeafNode:
                for bid, boid in enumerate(node.data):
                    boid.step(self.getNeighborhood, viz, boid.id)
                    if not Octree.pointWithinCube(boid.pos, node.pos, node.size):
                        boids_to_move.append((node, bid, boid))

        # remove boids from their old node
        #print("Before removal: " + str(self.container.root.count_children()))

        for node, bid, boid in boids_to_move:
            node.data.remove(boid)
            nodes_of_removed.add(node)
        #print("After removal: " + str(self.container.root.count_children()))
        # add boids to new node (and create more nodes if required)
        for node, bid, boid in boids_to_move:
            stop = False
            while not stop:
                inserted = self.container.insertNode(boid.pos, boid, node.parent)
                if inserted is None:
                    node = node.parent
                    if node == self.container.root:
                        print("ALARM")
                else:
                    stop = True
        self.container.cutTree(list(nodes_of_removed))

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
