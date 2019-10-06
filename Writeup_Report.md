# **Self-Driving Car Engineer**
# **Project: Highway Driving Path Planning**

## MK

Overview
---
Develop a path planner in c++ to safely navigate around a virtual highway with other traffic that is driving +/-10 MPH of the 50 MPH speed limit.

The Project
---
The goals/steps for this project:
* Ego-car should maintain vehicle speed close to 50 mph, whenever possible
* Ego-car must avoid hitting other cars during all manuvers
* Ego-car must drive inside of the marked road lanes, except when changing lanes
* Ego-car must atleast complete one complete loop around the 6946 (m) length of highway
* Ego-car should take a little over 5 minutes to complete 1 loop
* Ego-car must maintain: (acceleration < 10 m/s^2) and (jerk (rate of change of acceleration) < 10 m/s^3)


[//]: # (Image References)

[image1]: ./Writeup_IV/HDPP_FRS.png "HDPP_FRS"

## [Rubric](https://review.udacity.com/#!/rubrics/432/view) Points

### Consider the rubric points individually and describe how each point has been addressed.

---
### Files Submitted & Code Quality

#### 1. Files included with submission to run the simulator in autonomous mode

Project includes the following files:
* main.cpp contains all the relevant, helper (helpers.h), and header files necessary to execute the path planner to control ego-car within the simulator 
* helpers.h: functions "getXY" (transform from Frenet s,d coordinates to Cartesian x,y) and "distance" (Calculate distance between two points) are the two main functions defined within helpers.h
* Writeup_Report.md summarizes the results
* The first attempt resulted in a distance of 38.9 miles without incident. Final output video [Link](https://www.youtube.com/watch?v=G4B1sXR3a6I&t=10s)
![][image1]


#### 2. Project code (main.cpp)

The main.cpp file contains the code for the following set of tasks:
* Load all images and steering angles from memory
* Data augumentation using vertical flip
* Data normilization and centering
* Image cropping to retain, only pixels that contain useful information
* Define and build convolution neural network model
* Training, validation, and saving the model

### Model Architecture and Training Strategy

#### 1. Model architecture 

Model consists of a convolution neural network with the following set of features 
![][image1]
![][image2]

#### 2. Model parameter tuning

The model used an adam optimizer, so the learning rate was not tuned manually

#### 3. Appropriate training data

Training data was collected to keep the vehicle driving on the road. Used only center lane driving. For details about how I created the training data, see the next section. 

### Model Architecture and Training Strategy

The CNN model used in this project has been replicated and based on the model developed by nvidia for similar work [Link](https://devblogs.nvidia.com/deep-learning-self-driving-cars/).

In order to gauge how well the model was working, image and steering angle data was split into a training and validation sets with a split ratio of 80% and 20% respectively. During the frist attempt itself, model had a low mean squared error on both the training and validation sets. Further, the validation error was lower than the training error. 

Final step was to run the simulator to see how well the car was driving around track one. There was only a single loctaion where the vehicle fell off the track. To improve driving behavior for this specific case, additional data was collected in the vicnity of this specific location.

At the end of the process, the vehicle is able to drive autonomously around the track without leaving the road.


#### Creation of the Training Set & Training Process

To capture good driving behavior the following steps were implemented for data collection:
* Recorded 8 laps on track one using center lane driving in forward direction
* Recorded 8 laps on track one using center lane driving in reverse direction. To minimize left-turn bias in the data.
* Further data augumentation was performed using "cv2" vertical flip on all of the images

Below are few random examples of center lane driving in forward direction

![][image3]


Below are few random examples of center lane driving in reverse direction

![][image4]

After collection and data augumentation process, a total of 34,660 number of data points represented the complete data set. Finally, the dataset was shuffled randomly and allocated 20% of the data into a validation set. 

Used this training data for training the model. The validation set helped determine if the model was over or under fitting. The ideal number of epochs was determined to be about 5. Used an adam optimizer so that manually training the learning rate wasn't necessary.
