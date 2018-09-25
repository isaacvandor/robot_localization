# robot_localization
This is the base repo for the Olin Computational Robotics Robot Localization project
Gretchen Rice & Isaac Vandor // Comprobo 2018

## Code Architecture Plan
### What classes will you create for your implementation (everyone should have a ParticleFilter class as given in the starter code, but more are probably better)?
Particle: Handles all particle updating, weighting, resampling related functions
ParticleFilter: handles all particle filtering related functions
RunRobot: handles all robot-related functions

### What functions will be implemented in these classes?
create_particle: Makes particles, assigns weight to them, and visualizes them
particle_filter: Does the actual particle filtering
robot_position: Calculates updated robot position
laser_callback: handles laser messages
particle_updater: Updates particles based on new robot position
particle_resampler: resamples based on new weights
publisher: Publishes resampled weights

### If you will be using the code in the helper classes, how will your code utilize them.
We will almost certainly use the code in the helper classes as a good way to define our new robot positions and convert the coordinates we are getting from the lidar into useful information for our particle filter.

### Define a timeline of how the code will interact with the sensor data coming from the robot and generate update estimates of the robot's location.
1. Take initial lidar readings to create a particle cloud
2. Update our particle cloud based on transformation from Neato last position to current position
3. Weight particles based on lidar measurements
4. Resample particles based on probability distribution of new weights
5. Repeat
