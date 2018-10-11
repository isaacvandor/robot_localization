## What was the goal of your project?
The goal of this project was to create a robot localizer using particle clouds
and filtering. In order to do this we created a particle cloud containing
weighted particles. The weights corresponded to how likely it was the robot was
at that point based on the robots change in x, y, and theta position. We used
the robots change in position to update the particles position which moved the
same x,y, and theta amount as the robot. 

## How did you solve the problem? 
*(Note: this doesn't have to be super-detailed, you should try to explain what you did at a high-level so that others in the class could reasonably understand what you did).*
To develop the localizer, three classes were created: a Particle class, a ParticleFilter class, and a Localizer class. The particle class handles the creation of particles and converts them into a Pose message easily visualized in RVIZ while the ParticleFilter class iniializes a particle cloud around the robot (plus some noise to add variation), normalized the particles, and then publishes them to a PoseArray.

## Describe a design decision you had to make when working on your project and what you ultimately did (and why)? 
*These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.*
One of the main design decisions we made in the localizer class was the math for handling robot pose updates. There are two good methods for this: computing the mean pose or computing the most likely pose. We chose to use the average particle position to calculate mean pose because we both took trig in high school and felt that calculating mean pose was the quickest implementation.

## What if any challenges did you face along the way?
Rather than starting with the particle cloud, we built a lot of logic to deal with the localization aspect. While this was great for getting localization working quickly, getting the particle cloud to show up was much harder. Also, @Isaac is pretty dumb and didn't realize he was appending particles outside of the for loop. Once we got this working, most of the work focused on tuning and improving the particle filter. 

## What would you do to improve your project if you had more time?
If we had more time, we would definitely focus on improving the particle filter. Currently, the particle filter uses occupancy fields and the Nearest Neighbors Algorithm to filter particles. Using occupancy fields works well for a basic implementation, but this method doesn't take into account obstacles that might be between the robot and the distance at which it is checking. Additionally, other methods like ray casting might be faster and ultimately more successful if we had the time to implement these methods. 

![GIF of our "localizer" running](https://github.com/isaacvandor/robot_localization/blob/master/media/solonelyohsolonely.gif)

## Did you learn any interesting lessons for future robotic programming projects?
*These could relate to working on robotics projects in teams, working on more
open-ended (and longer term) problems, or any other relevant topic.*
The largest lesson we learned out of this project was to implement independently and then bring everything together once the individual components are working. While we broke down the localization into particles, filtering, and robot stuff pretty nicely, the crunch to have everything integrated and working for the original deadline meant that we didn't get to independently test as much as we would have liked. This led to issues down the road when we tried to test the complete implementation and ran into issues with our individual bits of code. 
