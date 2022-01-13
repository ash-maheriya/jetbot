# jetbot
Jetson TX1/TX2 based robot using ROS.

This repository contains the files that are necessary for a companion robot. I wanted to create something that felt less like a mechanical entity and more like a living being. The robot uses computer vision and machine learning to follow me around when I am holding a specific item and also responds to my hand gestures. 
The plan was to have the robot entirely controlled by a Jetson TX2, but after trying to make this work I ran into difficulties communicating between the motor controllers and the Jetson. I resolved this by using an Arduino as a middleman between the motor controllers and the Jetson. There were many such challenges where I had to find workarounds. 
Even though it is far from a synthetic lifeform, the centerpiece of my vision for the future, this is my first baby step.
