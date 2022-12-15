import numpy as np


class spatialHashtable:

    def __init__(self, N, cells_per_dim=32):

        # cells_per_dimension
        self.cpd = 45
        # number_grid = cells_per_dim ** 3

        # object index
        # object_index = np.array((N, 2))

        # hash table
        # hash_table = np.array((N,1))

        self.hashtable = {}
        self.blist = []

        # pivot table
        # pivot_table = np.array((number_grid, 4))

    # adds a boid to the datastructure
    def add(self, boid):

        index = self.getCell(boid.pos[0], boid.pos[1], boid.pos[2])
        self.add_values_in_dict(index, boid)
        self.blist.append(boid)

    def addToHashOnly(self, boid):

        index = self.getCell(boid.pos[0], boid.pos[1], boid.pos[2])
        self.add_values_in_dict(index, boid)

    def step(self):

        # updated every boid
        for boid in self.blist:
            boid.step(self.getNeighborhood)

        # reset hashtable
        self.hashtable = {}

        # build new hashtable
        for boid in self.blist:
            self.addToHashOnly(boid)

    def add_values_in_dict(self, key, boid):
        ''' Append multiple values to a key in
            the given dictionary '''

        if key not in self.hashtable:
            self.hashtable[key] = []

        self.hashtable[key].append(boid)

    def getNeighborhood(self, boid, radius_detection, radius_collision):

        return self.getNeighborhoodForOne(boid, radius_detection), self.getNeighborhoodForOne(boid, radius_collision)

    def getNeighborhoodForOne(self, boid, radius):

        boidlist = self.getNeighborhoodCells(boid, radius)

        boids = []

        for i in boidlist:

            boid_neighbors = self.hashtable.get(i)

            if boid_neighbors is not None:

                for x in boid_neighbors:

                    if np.linalg.norm(boid.pos - x.pos) <= radius:
                        boids.append(x)

        return boids

    def getNeighborhoodCells(self, boid, radius):

        x, y, z = boid.pos[0], boid.pos[1], boid.pos[2]

        rad = radius

        # get the cells:
        x_minus = self.getCell(x - rad, y, z)
        x_plus = self.getCell(x + rad, y, z)

        y_minus = self.getCell(x, y - rad, z)
        y_plus = self.getCell(x, y + rad, z)

        z_minus = self.getCell(x, y, z - rad)
        z_plus = self.getCell(x, y, z + rad)

        x_cell_steps = int(x_plus - x_minus)
        y_cell_steps = int((y_plus - y_minus) / self.cpd)
        z_cell_steps = int((z_plus - z_minus) / self.cpd ** 2)

        # get origin
        xyz_minus = self.getCell(x - rad, y - rad, z - rad)

        cell_list = list()

        for i in range(0, x_cell_steps + 1):

            # add on x scale
            current_cell = xyz_minus + i

            cell_list.append(current_cell)

            tmp = current_cell

            # add cells in z direction
            for k in range(0, z_cell_steps):
                tmp = tmp + self.cpd ** 2
                cell_list.append(tmp)

            if (y_cell_steps != 0):

                # add cells in y direction
                for j in range(0, y_cell_steps):
                    current_cell = current_cell + self.cpd
                    cell_list.append(current_cell)

                    current_cell_z_axis = current_cell
                    # add cells in z direction
                    for k in range(0, z_cell_steps):
                        current_cell_z_axis = current_cell_z_axis + self.cpd ** 2
                        cell_list.append(current_cell_z_axis)

        return cell_list

    # def step(self):

    # this is the hash function - a cell is computed for the coordinates of a boid
    # the index of the cell is returned
    def getCell(self, x, y, z):

        x, y, z = self.transform_coord(x, y, z)

        # computing a fitting modulo operator
        mod_op = 2 / self.cpd

        x_cell = x // mod_op

        if x_cell >= self.cpd:
            x_cell = self.cpd - 1

        if x_cell <= 0:
            x_cell = 0

        y_cell = y // mod_op

        if y_cell >= self.cpd:
            y_cell = self.cpd - 1

        if y_cell <= 0:
            y_cell = 0

        z_cell = z // mod_op

        if z_cell >= self.cpd:
            z_cell = self.cpd - 1

        if z_cell <= 0:
            z_cell = 0

        final_cell = x_cell + y_cell * self.cpd + z_cell * self.cpd ** 2

        return int(final_cell)

    def transform_coord(self, x, y, z):

        x = x + 1
        y = y + 1
        z = z + 1

        return x, y, z
