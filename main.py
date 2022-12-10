import numpy as np
from boidContainerList import BoidContainerList
from boidContainerOctree import BoidContainerOctree
from boid import Boid
import time


if __name__ == '__main__':
    N = 500
    boidContainerList = BoidContainerList()
    boidContainerOctree = BoidContainerOctree()
    # Set static values of the boids like this: Boid.l1 = value (set to values from Ex. description per default)

    for i in range(N):
        pos = np.random.uniform(-1, 1, 3)
        velocity = np.array([0,0,0])
        boid = Boid(pos, velocity)
        boidContainerList.add(boid)
        boidContainerOctree.add(boid)

    start = time.time()
    for i in range(10):
        boidContainerList.step()
    end = time.time()
    print("list: "  + str(end - start))

    start = time.time()
    for i in range(10):
        boidContainerOctree.step()
    end = time.time()
    print("octree: "  + str(end - start))


    #while(True):
        #boidContainer.step()
        #visualize



