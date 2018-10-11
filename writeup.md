## What was the goal of your project?

## How did you solve the problem? 
(Note: this doesn't have to be super-detailed, you should try to explain what you did at a high-level so that others in the class could reasonably understand what you did).

## Describe a design decision you had to make when working on your project and what
you ultimately did (and why)? 
These design decisions could be particular choices
for how you implemented some part of an algorithm or perhaps a decision regarding
which of two external packages to use in your project.

## What if any challenges did you face along the way?

## What would you do to improve your project if you had more time?

## Did you learn any interesting lessons for future robotic programming projects?
These could relate to working on robotics projects in teams, working on more
open-ended (and longer term) problems, or any other relevant topic.

The goal of this project was to create a robot localizer using particle clouds
and filtering. In order to do this we created a particle cloud containing
weighted particles. The weights corresponded to how likely it was the robot was
at that point based on the robots change in x, y, and theta position. We used
the robots change in position to update the particles position which moved the
same x,y, and theta amount as the robot. 
