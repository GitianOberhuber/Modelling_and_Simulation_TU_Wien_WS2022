import numpy as np
from boid import Boid
from spatialHashtable import spatialHashtable


if __name__ == '__main__':
    N = 10

    table = spatialHashtable(10, 32)


    # Set static values of the boids like this: Boid.l1 = value (set to values from Ex. description per default)

    for i in range(1):


        pos = np.array([-1, -1, -1])
        velocity = np.array([0, 0, 0])
        boid = Boid(pos, velocity)

        table.add(boid)
        table.getNeighborhood(boid, 0.1)

        pos = np.array([0, -1, -1])
        velocity = np.array([0, 0, 0])
        boid = Boid(pos, velocity)

        table.add(boid)
        table.getNeighborhood(boid, 0.1)

        pos = np.array([1, -1, -1])
        velocity = np.array([0, 0, 0])
        boid = Boid(pos, velocity)


        table.add(boid)
        table.getNeighborhood(boid, 0.1)

        pos = np.array([-0.5, -1, -1])
        velocity = np.array([0, 0, 0])
        boid = Boid(pos, velocity)

        table.add(boid)
        table.getNeighborhood(boid, 0.1)

        pos = np.array([0.5, -1, -1])
        velocity = np.array([0, 0, 0])
        boid = Boid(pos, velocity)

        table.add(boid)
        table.getNeighborhood(boid, 0.1)

        pos = np.array([-1, 0, -1])
        velocity = np.array([0, 0, 0])
        boid = Boid(pos, velocity)

        table.add(boid)
        table.getNeighborhood(boid, 0.1)
        


        pos = np.array([-1, 0.3, -1])
        velocity = np.array([0, 0, 0])
        boid = Boid(pos, velocity)

        table.add(boid)
        table.getNeighborhood(boid, 0.1)

        pos = np.array([1, 1, -1])
        velocity = np.array([0, 0, 0])
        boid = Boid(pos, velocity)

        table.add(boid)
        table.getNeighborhood(boid, 0.1)

        pos = np.array([1, 1, 1])
        velocity = np.array([0, 0, 0])
        boid = Boid(pos, velocity)

        table.add(boid)
        table.getNeighborhood(boid, 0.1)

        pos = np.array([1, 1, 1])
        velocity = np.array([0, 0, 0])
        boid = Boid(pos, velocity)

        table.add(boid)
        table.getNeighborhood(boid, 0.1)

    #while(True):
        #boidContainer.step()
        #visualize



