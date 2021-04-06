## General info
This project is created for PA7 by Al Colon and Seho Kim. A robot races along a given track infinetely. 

## To start
roslaunch racetrack racetrack.launch

## Description
It spawns a robot. A pre-created wolrd is loaded in Gazebo where a track is drawn. The robot, using computer vision, detects track and follows it.
 
## Explanation
line_follower.py file reads robot's camera data. Then, it filters by color in order to find track. After detecting the line, using PID control, the robot races along the track

## Video
https://drive.google.com/file/d/1YEPTIM6pwnQczA5FP5yggnG6UxfEnx7S/view?usp=sharing