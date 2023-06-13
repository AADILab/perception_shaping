# perception_shaping
Ros package for perception shaping with the turtlebot3

This code accompanies the "Evolving Supervisory Agents for Multiagent Perception Shaping" paper submitted to MRS 2023.

This is for a gazebo simulation of a turtlebot3 burger where the robot drives to a POI in gazebo. However, there is an obstacle in between the robot and POI that the robot cannot sense with its lidar. This is where the supervisor agent comes in. The supervisor can sense the obstacle, and must shape the robot's perception in order to guide the robot around the obstacle. In its current state, this package requires that you manually launch gazebo, setup the POI and obstacle, and then you can run the launch file to run the supervisor.
