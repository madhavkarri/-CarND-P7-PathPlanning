# **Self-Driving Car Engineer**
# **Project: Highway Driving Path Planning**

## MK

Overview
---
Develop a path planner to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.

The Project
---
The goals/steps for this project:
* The ego-car should maintain vehicle speed close to 50 mph, whenever possible
* Ego-car should atleast complete one complete loop around the 6946 (m) length of highway
* Ego-car should take a little over 5 minutes to complete 1 loop
* Ego-car should maintain: (acceleration < 10 m/s^2) and (jerk (rate of change of acceleration) < 10 m/s^3)
* 

, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

[//]: # (Image References)

[image1]: ./Writeup_IV/CNNModelSummary.png "CNNModelSummary"
[image2]: ./Writeup_IV/CNNArch.png "CNNArch"
[image3]: ./Writeup_IV/CLD_Fwd.png "CLD_Fwd"
[image4]: ./Writeup_IV/CLD_Rwd.png "CLD_Rwd"


## [Rubric](https://review.udacity.com/#!/rubrics/432/view) Points

### Consider the rubric points individually and describe how each point has been addressed.

---
### Files Submitted & Code Quality

#### 1. Files included with submission to run the simulator in autonomous mode

Project includes the following files:
* model.py containing the script to create and train the model
* drive.py for driving the car in autonomous mode
* model.h5 containing a trained convolution neural network 
* MadhavKarri_CarNDP4_Writeup.md summarizes the results
* Final output video [Link](./Writeup_IV/video.mp4)

#### 2. Functional code
Using the Udacity provided simulator and drive.py file, the car can be driven autonomously around the track by executing 
```sh
python drive.py model.h5
```

#### 3. Project code (model.py)

The model.py file contains the code for the following set of tasks.
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
