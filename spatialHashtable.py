import numpy as np

class spatialHashtable:

    def __init__(self, N, cells_per_dim = 32):

        # cells_per_dimension
        self.cpd = 32
        number_grid = cells_per_dim**3

        # object index
        object_index = np.array((N,2))

        # hash table
        #hash_table = np.array((N,1))

        self.hashtable = {}

        # pivot table
        pivot_table = np.array((number_grid, 4))


    # adds a boid to the datastructure
    def add(self, boid):

        index = self.getCell(boid.pos[0], boid.pos[1], boid.pos[2])
        self.add_values_in_dict(index, boid)


    def add_values_in_dict(self, key, boid):
        ''' Append multiple values to a key in
            the given dictionary '''

        if key not in self.hashtable:
            self.hashtable[key] = list()

        self.hashtable[key].append(boid)



    def getNeighborhood(self, boid, radius):

        # TODO - transform to coordinate system with 0,0,0 origin

        rad = radius

        #get the cells:
        c1 = self.getCell(boid)

        x_minus = self.getCell(boid.pos[0] - rad, boid.pos[1], boid.pos[2])
        x_plus = self.getCell(boid.pos[0] + rad, boid.pos[1], boid.pos[2])

        y_minus = self.getCell(boid.pos[0], boid.pos[1] - rad, boid.pos[2])
        y_plus =  self.getCell(boid.pos[0], boid.pos[1] + rad, boid.pos[2])

        z_minus = self.getCell(boid.pos[0], boid.pos[1], boid.pos[2] - rad)
        z_plus = self.getCell(boid.pos[0], boid.pos[1], boid.pos[2] + rad)

        x_cell_steps = x_plus - x_minus
        y_cell_steps = (y_plus - y_minus)/self.cpd
        z_cell_steps = (z_plus - z_minus)/self.cpd**2

        # get origin
        xyz_minus = self.getCell(boid.pos[0] - rad, boid.pos[1] - rad, boid.pos[2] - rad)

        cell_list = list()

        for i in range(0,x_cell_steps + 1):

            # add on x scale
            current_cell = xyz_minus + i

            cell_list.append(current_cell)

            #for the case there are no steps in y direction #and that base

            for k in range(0, z_cell_steps):

                current_cell = current_cell + self.cpd**2
                cell_list.append(current_cell)

            for j in range(0, y_cell_steps):

                current_cell = current_cell + self.cpd
                cell_list.append(current_cell)

                for k in range(0, z_cell_steps):

                    current_cell = current_cell + self.cpd**2
                    cell_list.append(current_cell)

        return cell_list


    #def step(self):



    # this is the hash function - a cell is computed for the coordinates of a boid
    # the index of the cell is returned
    def getCell(self, x, y, z):

        x,y,z = self.transform_coord(x,y,z)

        # computing a fitting modulo operator
        mod_op = 2 / self.cpd

        x_cell = x // mod_op

        # randfall
        if x_cell >= self.cpd:
            x_cell = self.cpd - 1


        y_cell = y // mod_op

        if y_cell >= self.cpd:
            y_cell = self.cpd - 1


        z_cell = z // mod_op

        if z_cell >= self.cpd:
            z_cell = self.cpd - 1


        final_cell = x_cell + y_cell*self.cpd + z_cell*self.cpd**2

        return final_cell


    def transform_coord(self, x,y,z):

        x = x + 1
        y = y + 1
        z = z + 1

        return x, y, z









