#our_localization_bag1: uses mymap.pgm  
In this bag file, we were figuring out how to properly start the launch, move the robo, and start the bag file, so it takes a little bit before the initial robo pose guess is put in. After moving the robot around for a bit, we left it in one place to see our algorithm converge. As you can see, the cloud slowly converges to 2 spots on the map.

#our_localization_bag2: uses mymap.pgm  
In this bag file the initial pose is guessed immediately. After that, we move the robot around the map as the point cloud starts to move and guess the location. We stopped the robot in approximately the same place it started. Again, the cloud converged to 2 points. This time, one of the points looked to be almost exactly where the robot was, the other point was by a different wall, which I can ssume had similar estimated lidar readings as the real location.
