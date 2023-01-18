import numpy as np


# based on https://github.com/jcummings2/pyoctree/blob/master/octree.py and adapted for boid-simulation

class OctNode(object):
    def __init__(self, pos, size, depth, data, parent=None):
        """
        OctNode Cubes have a position and size
        position is related to, but not the same as the objects the node contains.
        Branches (or children) follow a predictable pattern to make accesses simple.
        Here, - means less than 'origin' in that dimension, + means greater than.
        branch: 0 1 2 3 4 5 6 7
        x:      - - - - + + + +
        y:      - - + + - - + +
        z:      - + - + - + - +
        """
        self.pos = pos
        self.size = size
        self.depth = depth

        ## All OctNodes will be leaf nodes at first
        ## Then subdivided later as more objects get added
        self.isLeafNode = True

        ## store our object, typically this will be one, but maybe more
        self.data = data

        ## might as well give it some emtpy branches while we are here.
        self.branches = [None, None, None, None, None, None, None, None]

        ## set parent for bottom-up-navigation
        self.parent = parent

        half = size / 2

        ## The cube's bounding coordinates
        self.lower = (pos[0] - half, pos[1] - half, pos[2] - half)
        self.upper = (pos[0] + half, pos[1] + half, pos[2] + half)

    def __str__(self):
        if (self.data == None):
            return u"position: {0}, size: {1}, depth: {2} leaf: {3}".format(
                self.pos, self.size, self.depth, self.isLeafNode)

        else:
            data_str = u", ".join((str(x) for x in self.data))
            return u"position: {0}, size: {1}, depth: {2} leaf: {3}, data: {4}".format(
                self.pos, self.size, self.depth, self.isLeafNode, data_str)

    def count_children(self):
        if self.isLeafNode:
            return len(self.data)
        else:
            children_sums = [branch.count_children() for branch in self.branches if branch]
            return sum(children_sums)


class Octree(object):
    """
    The octree itself, which is capable of adding and searching for nodes.
    """

    def __init__(self, worldSize, origin, max_type, max_value):
        """
        Init the world bounding root cube
        all world geometry is inside this
        it will first be created as a leaf node (ie, without branches)
        this is because it has no objects, which is less than MAX_OBJECTS_PER_CUBE
        if we insert more objects into it than MAX_OBJECTS_PER_CUBE, then it will subdivide itself.
        """
        self.root = OctNode(origin, worldSize, 0, [])
        self.worldSize = worldSize
        self.limit_nodes = (max_type == "nodes")
        self.limit = max_value
        self.num_branches = 0

    def insertNode(self, position, objData=None, node=None):
        """
        Add the given object to the octree if possible
        Parameters
        ----------
        position : array_like with 3 elements
            The spatial location for the object
        objData : optional
            The data to store at this position. By default stores the position.
            If the object does not have a position attribute, the object
            itself is assumed to be the position.
        Returns
        -------
        node : OctNode or None
            The node in which the data is stored or None if outside the
            octree's boundary volume.
        """
        if node is None:
            node = self.root
        if np.any(position < node.lower):
            return None
        if np.any(position > node.upper):
            return None
        if objData is None:
            objData = position

        return self.__insertNode(node, node.size, node.parent, position, objData)

    def cutTree(self, nodes):
        # print(self.root.count_children())
        while len(nodes) > 0 and nodes[0] != self.root:
            nodes.sort(key=lambda x: x.depth, reverse=True)
            node = nodes.pop(0)
            if not node.isLeafNode and node.count_children() < self.limit:
                node.isLeafNode = True
                data_lists = [branch.data for branch in node.branches if branch and branch.data]
                node.data = [j for i in data_lists for j in i]
                node.branches = [None, None, None, None, None, None, None, None]
                nodes.append(node.parent)

    def __insertNode(self, root, size, parent, position, objData):
        """Private version of insertNode() that is called recursively"""
        if root is None:
            # we're inserting a single object, so if we reach an empty node, insert it here
            # Our new node will be a leaf with one object, our object
            # More may be added later, or the node maybe subdivided if too many are added
            # Find the Real Geometric centre point of our new node:
            # Found from the position of the parent node supplied in the arguments
            pos = parent.pos

            ## offset is halfway across the size allocated for this node
            offset = size / 2

            ## find out which direction we're heading in
            branch = self.__findBranch(parent, position)

            ## new center = parent position + (branch direction * offset)
            newCenter = (0, 0, 0)

            if branch == 0:
                newCenter = (pos[0] - offset, pos[1] - offset, pos[2] - offset)
            elif branch == 1:
                newCenter = (pos[0] - offset, pos[1] - offset, pos[2] + offset)
            elif branch == 2:
                newCenter = (pos[0] - offset, pos[1] + offset, pos[2] - offset)
            elif branch == 3:
                newCenter = (pos[0] - offset, pos[1] + offset, pos[2] + offset)
            elif branch == 4:
                newCenter = (pos[0] + offset, pos[1] - offset, pos[2] - offset)
            elif branch == 5:
                newCenter = (pos[0] + offset, pos[1] - offset, pos[2] + offset)
            elif branch == 6:
                newCenter = (pos[0] + offset, pos[1] + offset, pos[2] - offset)
            elif branch == 7:
                newCenter = (pos[0] + offset, pos[1] + offset, pos[2] + offset)

            # Now we know the centre point of the new node
            # we already know the size as supplied by the parent node
            # So create a new node at this position in the tree
            # print "Adding Node of size: " + str(size / 2) + " at " + str(newCenter)
            return OctNode(newCenter, size, parent.depth + 1, [objData], parent)

        # else: are we not at our position, but not at a leaf node either
        elif (not root.isLeafNode and
              (
                      (np and np.any(root.pos != position))
                      or
                      (root.pos != position)
              )
        ):

            # we're in an octNode still, we need to traverse further
            branch = self.__findBranch(root, position)
            # Find the new scale we working with
            newSize = root.size / 2
            # Perform the same operation on the appropriate branch recursively
            root.branches[branch] = self.__insertNode(root.branches[branch], newSize, root, position, objData)

        # else, is this node a leaf node with objects already in it?
        elif root.isLeafNode:
            # We've reached a leaf node. This has no branches yet, but does hold
            # some objects, at the moment, this has to be less objects than MAX_OBJECTS_PER_CUBE
            # otherwise this would not be a leafNode (elementary my dear watson).
            # if we add the node to this branch will we be over the limit?
            if (
                    (self.limit_nodes and len(root.data) < self.limit)
                    or
                    (not self.limit_nodes and root.depth >= self.limit)
            ):
                # No? then Add to the Node's list of objects and we're done
                root.data.append(objData)
                # return root
            else:
                # Adding this object to this leaf takes us over the limit
                # So we have to subdivide the leaf and redistribute the objects
                # on the new children.
                # Add the new object to pre-existing list
                root.data.append(objData)
                # copy the list
                objList = root.data
                # Clear this node's data
                root.data = None
                # It is not a leaf node anymore
                root.isLeafNode = False
                # Calculate the size of the new children
                newSize = root.size / 2
                # distribute the objects on the new tree
                # print "Subdividing Node sized at: " + str(root.size) + " at " + str(root.pos)
                for ob in objList:
                    # Use the position attribute of the object if possible
                    if hasattr(ob, "pos"):
                        pos = ob.pos
                    else:
                        pos = ob
                    branch = self.__findBranch(root, pos)
                    root.branches[branch] = self.__insertNode(root.branches[branch], newSize, root, pos, ob)
        return root

    def findPosition(self, position):
        """
        Basic lookup that finds the leaf node containing the specified position
        Returns the child objects of the leaf, or None if the leaf is empty or none
        """
        if np.any(position < self.root.lower):
            return None
        if np.any(position > self.root.upper):
            return None
        return self.__findPosition(self.root, position)

    @staticmethod
    def __findPosition(node, position, count=0, branch=0):
        """Private version of findPosition """
        if node.isLeafNode:
            # print("The position is", position, " data is", node.data)
            # return node.data
            return node
        branch = Octree.__findBranch(node, position)
        child = node.branches[branch]
        if child is None:
            return None
        return Octree.__findPosition(child, position, count + 1, branch)

    def findNeighborhoodNode(self, node, position, radius_list):
        if np.any(position < node.lower):
            return None
        if np.any(position > node.upper):
            return None

        min_r_res = self.__findNeighborhoodNode(node, position, radius_list[0])
        if radius_list[0] == radius_list[1]:
            max_r_res = min_r_res
        else:
            max_r_res = self.__findNeighborhoodNode(min_r_res, position, radius_list[1])

        return min_r_res, max_r_res

    def __findNeighborhoodNode(self, node, position, radius, count=0, branch=0):
        corners = Octree.getCubeCorners(position, radius)

        boidRadiusFullyContained = all([Octree.pointWithinCube(corner, node.pos, node.size) for corner in corners])
        if boidRadiusFullyContained:
            return node
        else:
            return self.__findNeighborhoodNode(node.parent, position, radius)

    def findNeighborhoodBoids(self, position, radius_list, node):
        if np.any(position < node.lower):
            return None
        if np.any(position > node.upper):
            return None
        res = self.__findNeighborhoodBoids(node, position, radius_list)
        return res

    def __findNeighborhoodBoids(self, node, position, radius_list, count=0, branch=0):
        resNodes = self.findNeighborhoodNode(node, position, radius_list)
        resBoids = []
        for idx, radius in enumerate(radius_list):
            resNode = resNodes[idx]
            if resNode is None:
                print("alarm")
            boids = []

            if resNode.isLeafNode:
                nodes_to_check = [resNode]
            else:
                nodes_to_check = []
                nodes_to_check.extend(resNode.branches)
            while len(nodes_to_check) > 0:
                n = nodes_to_check.pop()

                if n is None:
                    continue
                sphere_corners = Octree.getCubeCorners(position, radius)
                if Octree.cubeWithinRadius(n.pos, n.size, position, radius):
                    boids.extend(Octree.__getAllChildren(n))
                elif any([Octree.pointWithinCube(corner, n.pos, n.size) for corner in sphere_corners]):
                    if n.isLeafNode:
                        candidates = n.data
                        for candidate in candidates:
                            if np.linalg.norm(position - candidate.pos) <= radius:
                                boids.append(candidate)
                    else:
                        nodes_to_check.extend(n.branches)
            resBoids.append(boids)
        return resBoids

    @staticmethod
    def __getAllChildren(node):
        if (node.isLeafNode):
            return [node.data]
        else:
            res = []
            for child in node.branches:
                if (not child is None):
                    res.extend(Octree.__getAllChildren(child))
            return res

    @staticmethod
    def __findBranch(root, position):
        """
        helper function
        returns an index corresponding to a branch
        pointing in the direction we want to go
        """
        index = 0
        if (position[0] >= root.pos[0]):
            index |= 4
        if (position[1] >= root.pos[1]):
            index |= 2
        if (position[2] >= root.pos[2]):
            index |= 1
        return index

    def iterateDepthFirst(self):
        """Iterate through the octree depth-first"""
        gen = self.__iterateDepthFirst(self.root)
        for n in gen:
            yield n

    @staticmethod
    def __iterateDepthFirst(root):
        """Private (static) version of iterateDepthFirst"""

        for branch in root.branches:
            if branch is None:
                continue
            for n in Octree.__iterateDepthFirst(branch):
                yield n
            if branch.isLeafNode:
                yield branch

    def getRoot(self):
        return self.root

    # def __nodeWithinRadius__(self, boid, radius, node):
    # for corner in self.__getCubeCorners__(boid.pos, radius):

    @staticmethod
    def pointWithinRadius(point, sphereCenter, radius):
        return np.sum(np.power(point - sphereCenter, 2)) < np.power(radius, 2)

    @staticmethod
    def cubeWithinRadius(cubeCenter, size, sphereCenter, radius):
        corners = Octree.getCubeCorners(cubeCenter, size)
        return all([Octree.pointWithinRadius(corner, sphereCenter, radius) for corner in corners])

    @staticmethod
    def pointWithinCube(point, center, l):
        return (point[0] <= center[0] + l / 2 and point[0] >= center[0] - l / 2) and \
            (point[1] <= center[1] + l / 2 and point[1] >= center[1] - l / 2) and \
            (point[2] <= center[2] + l / 2 and point[2] >= center[2] - l / 2)

    @staticmethod
    def getCubeCorners(center, l):
        c1 = center + np.array([-l / 2, -l / 2, -l / 2])
        c2 = center + np.array([l / 2, -l / 2, -l / 2])
        c3 = center + np.array([-l / 2, l / 2, -l / 2])
        c4 = center + np.array([-l / 2, -l / 2, l / 2])
        c5 = center + np.array([l / 2, l / 2, -l / 2])
        c6 = center + np.array([l / 2, -l / 2, l / 2])
        c7 = center + np.array([-l / 2, l / 2, l / 2])
        c8 = center + np.array([l / 2, l / 2, l / 2])
        return c1, c2, c3, c4, c5, c6, c7, c8
