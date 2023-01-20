from data_structures.octree import Octree, OctNode


class BoidContainerOctree:
    def __init__(self, worldSize=4.0, origin=(0.0, 0.0, 0.0), max_type="nodes", max_value=5, rebuild=False):
        self.container = Octree(worldSize, origin, max_type, max_value)
        self.max_value = max_value
        self.max_type = max_type
        self.blist = []
        self.worldSize = worldSize
        self.origin = origin
        self.rebuild = rebuild

    def add(self, boid):
        self.container.insertNode(boid.pos, boid)
        self.blist.append(boid)

    def getNeighborhood(self, boid_i, radius_detection, radius_collision, node):
        if radius_detection > radius_collision or radius_detection == radius_collision:
            collision_boids, detection_boids =\
                self.container.findNeighborhoodBoids(boid_i.pos, [radius_collision, radius_detection], node)
        else:
            detection_boids, collision_boids = \
                self.container.findNeighborhoodBoids(boid_i.pos, [radius_detection, radius_collision], node)
        return detection_boids, collision_boids

    def step(self, viz):
        boids_to_move = []
        nodes_of_removed = set()

        # check which boids are outside their original node
        for idx, node in enumerate(self.container.iterateDepthFirst()):
            if node.isLeafNode:
                for bid, boid in enumerate(node.data):
                    boid.step(lambda a, b, c: self.getNeighborhood(a, b, c, node), viz, boid.id)
                    if not Octree.pointWithinCube(boid.pos, node.pos, node.size):
                        boids_to_move.append((node, bid, boid))
        if self.rebuild:
            self.container = Octree(self.worldSize, self.origin, self.max_type, self.max_value)
            for b in self.blist:
                self.container.insertNode(b.pos, b)
        else:
            for node, bid, boid in boids_to_move:
                node.data.remove(boid)
                nodes_of_removed.add(node)
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
