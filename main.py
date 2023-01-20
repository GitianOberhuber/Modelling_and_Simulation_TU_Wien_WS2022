import numpy as np
import os
from mayavi import mlab
import imageio
from boidContainerList import BoidContainerList
from boidContainerOctree import BoidContainerOctree
from boid import Boid
import time
from spatialHashtable import spatialHashtable


if __name__ == '__main__':
    N = 300         # nr of boids
    save_animation = True          # if True saves animation as gif in location specified in outpath
    outpath = os.getcwd() + '/anim.gif'   # maybe specify this manually, depends on system if it works
    delayer = 10        # Miliseconds delay between animation runs, hard lower limit is 10ms
    its = 700            # Iterations of the Simulation
    if Boid.vmax == 0:
        raise ValueError('Maximum boid velocity can not be zero.')
    sf = 0.08 / Boid.vmax           # assures the relative size of the arrows is independant of maximum speed (set fixed value for comparison)
    implementation = 'hashtable'   # select from 'hashtable', 'octree' or 'list'
    
    if (implementation == 'hashtable'):
        struct = spatialHashtable(10, 32)
    elif (implementation == 'octree'):
        struct = BoidContainerOctree(rebuild=True)
    elif (implementation == 'octree+'):
        struct = BoidContainerOctree()
    elif (implementation == 'list'):
        struct = BoidContainerList()
    else:
        raise ValueError('Implementation type must be implemented, please check for correct spelling.')
    
    # Define visualization matrix
    viz = np.zeros((6,N))

    # initialize starting parameters
    np.random.seed(42)
    for i in range(N):
        position = np.random.uniform(-1, 1, 3)
        velocity = np.array([0.001,0.001,0.001])
        #velocity = np.random.uniform(-0.015,0.015,3)   # Test case

        boid = Boid(position, velocity, i)
        struct.add(boid)
        
        # initial update of visualization matrix
        viz[0:3,0] = position
        viz[3:6,0] = velocity

    # animate loop without saving as gif
    @mlab.animate(delay = delayer)
    def animate_loop():
        for it in range(its):
            struct.step(viz)
            print('Step nr:',it+1)
            # Reset data, avoids redrawing canvas
            s.mlab_source.reset(x=viz[0,:], y=viz[1,:], z=viz[2,:], u=viz[3,:], v=viz[4,:], w=viz[5,:], scalars =  viz[0,:] + viz[1,:] + viz[2,:] + 0.000001)
            yield

    # animation loop with saving output as gif
    @mlab.animate(delay=delayer, ui=False)
    def animate_loop_gif(writer, s):
        for it in range(its):
            struct.step(viz)
            print('Step nr:',it+1)
            s.mlab_source.reset(x=viz[0,:], y=viz[1,:], z=viz[2,:], u=viz[3,:], v=viz[4,:], w=viz[5,:], scalars = viz[0,:] + viz[1,:] + viz[2,:] + 0.000001)
            image = mlab.screenshot()
            writer.append_data(image)
            yield

    fig = mlab.figure(size=(1600,1600), bgcolor=(1,1,1), fgcolor=(0.,0.,0.))         # make larger for higher quality
    s = mlab.quiver3d(viz[0,:], viz[1,:], viz[2,:], viz[3,:], viz[4,:], viz[5,:],line_width=6.0,scale_factor = sf, scale_mode = 'vector', \
                      colormap='Greys',mode='2darrow',figure=fig, scalars = viz[0,:] + viz[1,:] + viz[2,:] + 0.000001)
    s.glyph.color_mode = 'color_by_scalar'
    mlab.axes(figure=fig, ranges = [-1,1,-1,1,-1,1], extent = [-1,1,-1,1,-1,1])
    mlab.view(focalpoint = [0,0,0], distance = 8)


    if (save_animation == True):
        with imageio.get_writer(outpath, mode='I') as writer:
            a = animate_loop_gif(writer, s)
            mlab.show()
        
    else:
        a = animate_loop()
        mlab.show()
        
    """
    # Timings
    boidContainerOctree = BoidContainerOctree(rebuild=True)
    boidContainerOctreePlus = BoidContainerOctree()

    for i in range(N):
        position = np.random.uniform(-1, 1, 3)
        velocity = np.random.uniform(-0.015,0.015,3)   # Test case
        boid1 = Boid(position, velocity, i)
        boid2 = Boid(position, velocity, i)
        boidContainerOctree.add(boid1)
        boidContainerOctreePlus.add(boid2)

    start = time.time()
    for i in range(20):
        boidContainerOctree.step(viz)
    end = time.time()
    print("octree: "  + str(end - start))

    start = time.time()
    for i in range(20):
        boidContainerOctreePlus.step(viz)
    end = time.time()
    print("octree+: "  + str(end - start))
   """
    
    

    