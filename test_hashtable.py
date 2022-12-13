import numpy as np
from boid import Boid
from spatialHashtable import spatialHashtable

if __name__ == '__main__':
    N = 500

    table = spatialHashtable(10, 32)

    for i in range(N):
        pos = np.random.uniform(-1, 1, 3)
        velocity = np.random.uniform(0.1, 0.5, 3)
        boid = Boid(pos, velocity)
        table.add(boid)

    i = 0

    import time

    start_time = time.time()

    while i < 30:
        table.step()

        i = i + 1

    print("--- %s seconds ---" % (time.time() - start_time))
