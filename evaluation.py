import matplotlib as plt
import os
from boid import Boid
import numpy as np
from mayavi import mlab
import copy
import time
from boidContainerList import BoidContainerList
from boidContainerOctree import BoidContainerOctree
from spatialHashtable import spatialHashtable


def evaluateStrategies(Ns, iterations, containers_dict, seed = 42):
    #outpath = os.getcwd() + '/anim.gif'  # maybe specify this manually, depends on system if it works
    @mlab.animate(delay=10 )
    def animate_loop_eval(vizu, its, struct, s, writer = None):
        for it in range(its):
            struct.step(vizu)
            # Reset data, avoids redrawing canvas
            s.mlab_source.reset(x=vizu[0, :], y=vizu[1, :], z=vizu[2, :], u=vizu[3, :], v=vizu[4, :], w=vizu[5, :],
                                scalars=vizu[0, :] + vizu[1, :] + vizu[2, :] + 0.000001)
            yield
        mlab.close(all=True)

        return

    if Boid.vmax == 0:
        raise ValueError('Maximum boid velocity can not be zero.')
    sf = 0.08 / Boid.vmax  # assures the relative size of the arrows is independant of maximum speed (set fixed value for comparison)



    noVizTimes_dict = {}
    vizTimes_dict = {}
    for N in Ns:
        print("------------------ Run with " + str(N) + " boids ------------------")

        viz = np.zeros((6, N))
        np.random.seed(seed)
        boid_population = []
        for i in range(N):
            position = np.random.uniform(-1, 1, 3)
            velocity = np.array([0.001, 0.001, 0.001])
            boid = Boid(position, velocity, i)
            boid_population.append(boid)

            viz[0:3, 0] = position
            viz[3:6, 0] = velocity

        noVizTimes_dict[N] = {}
        vizTimes_dict[N] = {}
        for name, container in containers_dict.items():
            print("# Evaluating " + name)
            structNoVizEval = copy.deepcopy(container)
            structVizEval = copy.deepcopy(container)
            for boid in boid_population:
                structNoVizEval.add(copy.copy(boid))
                structVizEval.add(copy.copy(boid))

            #No visualization runtime evaluation
            t1 = time.time()
            for i in range(iterations):
                structNoVizEval.step(None)
            t = time.time() - t1
            print("Without visualization using " + name + " took " + str(round(t, 3)) + " seconds.")
            noVizTimes_dict[N][name] = t

            #Visualization runtime evaluation
            cur_viz = copy.deepcopy(viz)
            fig = mlab.figure(size=(1600, 1600), bgcolor=(1, 1, 1), fgcolor=(0., 0., 0.))
            s = mlab.quiver3d(cur_viz[0, :], cur_viz[1, :], cur_viz[2, :], cur_viz[3, :], cur_viz[4, :], cur_viz[5, :], line_width=6.0,
                              scale_factor=sf,
                              scale_mode='vector', \
                              colormap='Greys', mode='2darrow', figure=fig,
                              scalars=cur_viz[0, :] + cur_viz[1, :] + cur_viz[2, :] + 0.000001)

            s.glyph.color_mode = 'color_by_scalar'
            mlab.axes(figure=fig, ranges=[-1, 1, -1, 1, -1, 1], extent=[-1, 1, -1, 1, -1, 1])
            mlab.view(focalpoint=[0, 0, 0], distance=8)


            t1 = time.time()
            a = animate_loop_eval(cur_viz, iterations, structVizEval, s)
            a.close()
            mlab.show()
            t = time.time() - t1
            print("With visualization using " + name + " took " + str(round(t, 3)) + " seconds.")
            noVizTimes_dict[N][name] = t
    return noVizTimes_dict, vizTimes_dict

if __name__ == '__main__':
    noVizTimes_dict, vizTimes_dict = evaluateStrategies([100, 200, 300, 400, 500], 500,
                                                        {"list": BoidContainerList(), "octree": BoidContainerOctree(),
                                                         "hashtable": spatialHashtable(10, 32)})

