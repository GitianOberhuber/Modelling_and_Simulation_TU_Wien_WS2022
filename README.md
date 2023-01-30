# Modelling_and_Simulation_TU_Wien_WS2022

* **boid.py** contains code for simulating a boid (variables for position and velocity, functions for calculating new velocity/position and static parameters to control behaviour)
* **boidContainerList.py** manages all existing boids in a list (initial approach) and provides functions for adding boids `add`, getting the neighborhood of a boid `getNeighborhood` and calling the step-function on all contained boids `step`. Other implementations of more efficient datastructure should provide the same interface.
* **boidContainerOctree.py** uses an Octree to manage boids and efficiently calculate neighborhoods. 
* **data_structures/octree.py** contains an Octree implementation that was copied from  https://github.com/jcummings2/pyoctree/blob/master/octree.py and adapted for the boid-swarm scenario (search for neighborhood based on boid and it's radius)
* **boidContainerHashtable** contains a spatial Hashtable implementation for efficient neighborhood calculation

* **main.py** can be used to start the simulation and set parameters for the visualization
* **evaluation.py** calls the code in main.py for varying number of boids and environment-implementations and generates a plot that compares performances