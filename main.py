import numpy as np
from mayavi import mlab
from boidContainerList import BoidContainerList
from boidContainerOctree import BoidContainerOctree
from boid import Boid
import time
from spatialHashtable import spatialHashtable

        
if __name__ == '__main__':
    N = 300
    #boidContainerList = BoidContainerList()
    table = spatialHashtable(10, 32)
    # boidContainerOctree = BoidContainerOctree()
    # Set static values of the boids like this: Boid.l1 = value (set to values from Ex. description per default)
    # Define visualization matrix
    viz = np.zeros((6,N))

    for i in range(N):
        position = np.random.uniform(-1, 1, 3)
        #velocity = np.array([0,0,0])
        velocity = np.random.uniform(-0.015,0.015,3)   # Test case
        """
        # An Error occurs when running this starting configuration, coming from the vector_avoidColission function
        if (N%2 == 0):
            velocity = np.array([0,0.001,0])
        else:
            velocity = np.array([0,0,0.001])
        """
        boid = Boid(position, velocity)
        table.add(boid)
        #boidContainerOctree.add(boid)
        
        # initial update of vectorization matrix
        viz[0:3,0] = position
        viz[3:6,0] = velocity
    """
    # timings
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
    """
    
    # Set the animation framework and starting frame    
    fig = mlab.figure(size=(1600,1600))
    s = mlab.quiver3d(viz[0,:], viz[1,:], viz[2,:], viz[3,:], viz[4,:], viz[5,:],line_width=6.0,scale_factor = 4, scale_mode = 'vector', \
                      colormap='plasma',mode='2darrow',figure=fig, scalars = viz[2,:])
    s.glyph.color_mode = 'color_by_scalar'
    mlab.axes(figure=fig, ranges = [-1,1,-1,1,-1,1])
    delayer = 10        # Miliseconds delay between animation runs, hard lower limit is 10ms
    its = 400            # Iterations of the Simulation

    @mlab.animate(delay = delayer)
    def animate_loop():
        for it in range(its):
            table.step(viz)
            print("Step nr:",it+1)      # Debugging purpose
            # Reset data, avoids redrawing canvas
            s.mlab_source.reset(x=viz[0,:], y=viz[1,:], z=viz[2,:], u=viz[3,:], v=viz[4,:], w=viz[5,:], scalars = viz[2,:])
            yield
    animate_loop()
    mlab.show()


    
    
    

    