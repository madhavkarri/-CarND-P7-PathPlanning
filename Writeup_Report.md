# **Self-Driving Car Engineer**
# **Project: Highway Driving Path Planning**

## MK

Overview
---
Develop a path planner in c++ to safely navigate around a virtual 3 lane highway with other traffic that is driving +/-10 MPH of the 50 MPH speed limit. A successful path planner would keep inside its lane, avoid hitting other cars, and pass slower moving traffic using localization, sensor fusion, and map data.

The Project
---
The goals/steps for this project:
* Ego-car should maintain vehicle speed close to 50 mph, whenever possible
* Ego-car must avoid hitting other cars during all manuvers
* Ego-car must drive inside of the marked road lanes, except when changing lanes
* Ego-car must atleast complete one complete loop around the 6946 (m) length of highway
* Ego-car should take a little over 5 minutes to complete 1 loop
* Ego-car must maintain: (acceleration < 10 m/s^2) and (jerk: rate of change of acceleration < 10 m/s^3)


[//]: # (Image References)

[image1]: ./Writeup_IV/HDPP_FRS.png "HDPP_FRS"

## [Rubric](https://review.udacity.com/#!/rubrics/432/view) Points

### Consider the rubric points individually and describe how each point has been addressed.

---
### Files Submitted

#### 1. Files included with submission to run the simulator in autonomous mode

Project includes the following files:
* main.cpp contains all the relevant, helper (helpers.h), and header files necessary to execute the path planner to control ego-car within the simulator 
* helpers.h: functions "getXY" (transform from Frenet s,d coordinates to Cartesian x,y) and "distance" (Calculate distance between two points) are the two main functions defined within helpers.h
* spline.h: a function to construct a smooth spline (piece-wise polynomial) passing through a set of X and Y points. spline.h is used in developing a smooth trajectory generation
* Writeup_Report.md summarizes the results
* The first attempt resulted in a distance of 38.9 miles without incident. Final output video [Link](https://www.youtube.com/watch?v=G4B1sXR3a6I&t=10s)
![][image1]


#### 2. Project code (main.cpp)

The main.cpp file contains code for the following set of tasks:
* Read/load map data to acquire highway waypoint data [Link](./src/main.cpp#L33-L59)
* Connect to highway simulator using [micro-websocket](https://github.com/uNetworking/uWebSockets). Websocket handles two-way data transfer between planner and simulator
* Gather the below listed data from simulator [Link](./src/main.cpp#L80-L102):
  - Ego car localization data
  - 
