import numpy as np
from boid import Boid
from spatialHashtable import spatialHashtable


if __name__ == '__main__':
    N = 100

    table = spatialHashtable(10, 32)

    for i in range(N):

        pos = np.random.uniform(-1, 1, 3)
        velocity = np.array([0, 0, 0])
        boid = Boid(pos, velocity)
        table.add(boid)


    i = 0


    while(i < 200):
        table.step()

        i = i + 1




