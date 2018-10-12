## What was the goal of your project?
The goal of this project was to create a robot localizer using particle clouds
and filtering. In order to do this we created a particle cloud containing
weighted particles. The weights corresponded to how likely it was the robot was
at that point based on the robots change in x, y, and theta position. We used
the robot's change in position to update the particles position which moved the
same x,y, and theta amount as the robot. 

## How did you solve the problem? 
*(Note: this doesn't have to be super-detailed, you should try to explain what you did at a high-level so that others in the class could reasonably understand what you did).*
To develop the localizer, three classes were created (along with the TFHelper class and the OccupancyField class): a Particle class, a ParticleFilter class, and a RobotLocalizer class. The particle class handles the creation of particles and converts them into a Pose message easily visualized in RVIZ. The ParticleFilter class initializes a particle cloud around the robot (plus some noise to add variation), normalizes the particles, and publishes them to a PoseArray. The RobotLocalizer class holds the main functionality which we call when we want to run our code. This class uses the particle are from the ParticleFilter class to estimate the current position of the robot, it updates the particle positions based on odom readings, uses particle weights to resample the particle cloud, and sets up the frame transforms so we can visualize in the map frame.Below, you can see a GIF of our final particle filter running on the provided test BAG file.

![GIF of our particle filter in action](https://github.com/isaacvandor/robot_localization/blob/master/media/filterclouds.gif)

## Describe a design decision you had to make when working on your project and what you ultimately did (and why)? 
*These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.*
One of the main design decisions we made in the localizer class was the math for handling robot pose updates. There are two good methods for this: computing the mean pose or computing the most likely pose (based on the mode of the points). We chose to use the average particle position to calculate mean pose because we thought this would have a better representation of where the robot actually was. With the mode option, it is likely that the estimate could get thrown off because of a weighting error or miscalculation. By using mean,

## What if any challenges did you face along the way?
Rather than starting with the particle cloud, we built a lot of logic to deal with the localization aspect. This logic was to say that if the robot hadn't moved a certain amount, we should not update the particle cloud. However, our thresholds seemed to be too high becease we never got into the loop. We realized that although it would require a bit more computational power, it would be okay not to look at thresholds, so we ended up moving to those updates as long as the particle cloud was filled. We also made a slight error with the placement of where we appended particles to our particle filter. We accidentally put it outside of a for loop that created particles, this resulted in only one partcle being added to the particle filter, and subsequentially, only one being visualized. This caused a lot of confusion as we knew the particles were being made, we just didn't know where they were diesappearing too. Below you can see a GIF of our visualizer when only one particle was being added and visualized, it is the thin red arrow.  
Prior to this challenge we had a lot of challenges getting our code to work because of a lot of minor fixes. After our first pass, we had to completely rework the code architecture and the location of different functions. After that, we had a lot of minor code problems, like with how we were calling different functions, where those functions were being called (primarily, whcih loop they were be called in), creating two maps in two different classes, and visualization.

![GIF of our particle cloud just not even running at all](https://github.com/isaacvandor/robot_localization/blob/master/media/solonelyohsolonely.gif)

## What would you do to improve your project if you had more time?
If we had more time, we would definitely focus on improving the particle filter. Currently, the particle filter uses occupancy fields and the Nearest Neighbors Algorithm to filter particles. Using occupancy fields works well for a basic implementation, but this method doesn't take into account obstacles that might be between the robot and the distance at which it is checking. Additionally, other methods like ray casting might be faster and ultimately more successful if we had the time to implement these methods. 
We would aslo want to improve where the particles converged to. Currently, our the particles are converging to a few best guess points. Ideally, they should converge to one point the is fairly close to where the robot actually is. We would want to look into this further and see how we could improve the location estimations. Below, you can see a GIF of the localizer running before we got it to converge at all, as you can see, the point estimates were not very accurate.

![GIF of our "localizer" running](https://github.com/isaacvandor/robot_localization/blob/master/media/pfcloudrunning.gif)

## Did you learn any interesting lessons for future robotic programming projects?
*These could relate to working on robotics projects in teams, working on more
open-ended (and longer term) problems, or any other relevant topic.*
The largest lesson we learned out of this project was to implement independently and then bring everything together once the individual components are working. While we broke down the localization into particles, filtering, and robot stuff pretty nicely, the crunch to have everything integrated and working for the original deadline meant that we didn't get to independently test as much as we would have liked. This led to issues down the road when we tried to test the complete implementation and ran into issues with our individual bits of code. 
