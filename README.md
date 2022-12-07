# Modelling_and_Simulation_TU_Wien_WS2022

* **boid.py** contains code for simulating a boid (variables for position and velocity, functions for calculating new velocity/position and static parameters to control behaviour)

* **boidContainerList.py** manages all existing boids in a list (initial approach) and provides functions for adding boids `add`, getting the neighborhood of a boid `getNeighborhood` and calling the step-function on all contained boids `step`. Other implementations of more efficient datastructure should provide the same interface.
