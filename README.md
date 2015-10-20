# Exercise 1

This package is a turtlebot being teleoperated in an environment, avoiding crashing into obstacles and returning back to it's home base.

##Stopper

To start a turtlebot in stage so that it will move forward until it is within a stopping distance of an obstacle run the following:

```
roslaunch exercise1 stopper.launch
```

Change stop distance:
```
rosparam set stop_dist
```


## Guarded Teleoperation
This node will allow you to drive a turtlebot around a simulated environment with your keyboard. The node will protect you from crashing into walls with the robot. You can change how close the node will allow you to get to walls by setting the rosparameter.
Change stop distance:
```
rosparam set stop_dist
```

To start a turtlebot in stage and drive it around the environment, run the following:

```
roslaunch exercise1 driver.launch
```

To return to home:

```
rosparam set RTH true
```
