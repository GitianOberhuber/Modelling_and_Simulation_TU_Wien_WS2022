import numpy as np
from mayavi import mlab
from boidContainerList import BoidContainerList
from boidContainerOctree import BoidContainerOctree
from boid import Boid
import time


def get_vis_data(boidContainer, N):
    pos = np.zeros((3,N))
    vel = np.zeros((3,N))
    for i in range(N):
        pos[:,i] = boidContainer.container[i].pos
        vel[:,i] = boidContainer.container[i].velocity
    return pos, vel
        
if __name__ == '__main__':
    N = 500
    boidContainerList = BoidContainerList()
    boidContainerOctree = BoidContainerOctree()
    # Set static values of the boids like this: Boid.l1 = value (set to values from Ex. description per default)

    for i in range(N):
        pos = np.random.uniform(-1, 1, 3)
        velocity = np.array([0,0,0])
        #velocity = np.random.uniform(-0.015,0.015,3)   # Test case
        """
        # An Error occurs when running this starting configuration, coming from the vector_avoidColission function
        if (N%2 == 0):
            velocity = np.array([0,0.001,0])
        else:
            velocity = np.array([0,0,0.001])
        """
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



    # Set the animation framework and starting frame
    pos, vel = get_vis_data(boidContainer, N)
    fig = mlab.figure(size=(1600,1600))
    s = mlab.quiver3d(pos[0,:], pos[1,:], pos[2,:], vel[0,:], vel[1,:], vel[2,:],line_width=4.0,scale_factor = 2, scale_mode = 'vector', \
                      colormap='coolwarm',mode='2darrow',figure=fig, vmin=0, vmax=0.03)
    mlab.axes(figure=fig, ranges = [-1,1,-1,1,-1,1])
    ms = s.mlab_source
    delayer = 20        # Miliseconds delay between animation runs, hard lower limit is 10ms
    its = 100            # Iterations of the Simulation

    @mlab.animate(delay = delayer)
    def animate_loop():
        for it in range(its):
            boidContainer.step()
            print("Step nr:",it+1)      # Debugging purpose
            pos, vel = get_vis_data(boidContainer, N)
            ms.reset(x=pos[0,:], y=pos[1,:], z=pos[2,:], u=vel[0,:], v=vel[1,:], w=vel[2,:])    # Reset data, avoids redrawing canvas
            yield
    animate_loop()
    mlab.show()

    
    
    

    