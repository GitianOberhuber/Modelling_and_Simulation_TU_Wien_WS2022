import numpy as np
from boidContainerList import BoidContainerList
from boid import Boid


if __name__ == '__main__':
    N = 500
    boidContainer = BoidContainerList()
    # Set static values of the boids like this: Boid.l1 = value (set to values from Ex. description per default)

    for i in range(N):
        pos = np.random.uniform(-1, 1, 3)
        velocity = np.array([0,0,0])
        boid = Boid(pos, velocity)
        boidContainer.add(boid)

    #while(True):
        #boidContainer.step()
        #visualize



