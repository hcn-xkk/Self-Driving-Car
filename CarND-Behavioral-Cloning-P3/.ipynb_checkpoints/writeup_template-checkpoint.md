# **Behavioral Cloning** 


---

**Behavioral Cloning Project**

The goals / steps of this project are the following:
* Use the simulator to collect data of good driving behavior
* Build, a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./examples/Model.PNG "Model Visualization"
[image2]: ./examples/center_2020_07_07_13_27_44_826.jpg "Center Lane"
[image3]: ./examples/center_2020_07_05_21_41_10_969.jpg "Recovery Image1"
[image4]: ./examples/center_2020_07_05_21_41_11_172.jpg "Recovery Image2"
[image5]: ./examples/center_2020_07_05_21_41_11_377.jpg "Recovery Image3"
[image6]: ./examples/flip.PNG "Flipped Image"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/432/view) individually and describe how I addressed each point in my implementation.  

---
### Files Submitted & Code Quality

#### 1. Submission includes all required files and can be used to run the simulator in autonomous mode

My project includes the following files:
* model.ipynb containing the script to create and train the model
* drive.py for driving the car in autonomous mode
* modelNvidiaNoDrop_008_0.022486_0.015296.h5 containing a trained convolution neural network 
* writeup_report.md summarizing the results
* run008.mp4 A video recording of the vehicle driving autonomously at least one lap around the track.

#### 2. Submission includes functional code
Using the Udacity provided simulator and my drive.py file, the car can be driven autonomously around the track by executing 
```sh
python drive.py model.h5
```

#### 3. Submission code is usable and readable

The model.ipynb file contains the code for training and saving the convolution neural network. The file shows the pipeline I used for training and validating the model, and it contains comments to explain how the code works.

### Model Architecture and Training Strategy

#### 1. An appropriate model architecture has been employed

My model consists of a convolution neural network with 5x5 and 3x3 filter sizes and depths between 24 and 64 (`Part 2: Build CNN network for predicting steering angle`).

The model includes RELU layers to introduce nonlinearity, and the data is normalized in the model using a Keras lambda layer. 

I first tried the LeNet architecture. It would drive the vehicle for a part of the trial, but cannot successfully finish. So to use a more powerful model, I changed to the Nvidia CNN model architecture which includes a normalization layer, convolution layers with 5x5 and 3x3 filter sizes, as well as 3 fully connected layers. I used RELU as the activation function. The model is implemented in `Part 2.2: Apply Nvidia architecture` in model.ipynb.

#### 2. Attempts to reduce overfitting in the model

The model was trained and validated on different data sets to ensure that the model was not overfitting, this is implemented in `Prepare train_generator and validation_generator` in model.ipynb. Another approach to reduce overfitting is to include regularization in convolutional layers and fully connected layers. The final model is chosen by validation loss. Also, the training process is stopped early to reduce overfitting. The model was tested by running it through the simulator and ensuring that the vehicle could stay on the track.

#### 3. Model parameter tuning

The model used an adam optimizer, so the learning rate was not tuned manually.

#### 4. Appropriate training data

Training and validation data both come from this dataset. Dataset includes the original data provided with the project, and data collected on local laptop. When perform data collection, I used scenarios includes center lane driving, recovering from the left and right sides of the road. Image data from center, left and right camera are all used, and data augmentation is performed by flipping all images to get more data. When using left and right camera data, I applied a correction steering angle of 0.2 and -0.2, separately. 

For details about how I created the training data, see the next section. 

### Model Architecture and Training Strategy

#### 1. Solution Design Approach

The overall strategy for deriving a model architecture was to first use convolutional layers to extract features from images, and then use fully connected layers to predict steering angle. 

My first step was to use a convolution neural network model similar to the LeNet architecture. I thought this model might be appropriate because it performed well on classifying digits, which is mostly shapes and curves, similar to the lane edges in the current project.

In order to gauge how well the model was working, I split my image and steering angle data into a training and validation set. I found that my first model had a low mean squared error on the training set but a high mean squared error on the validation set. This implied that the model was overfitting. 

To combat the overfitting, I selected the model parameterization that gives low training loss and also low validation loss. 

The final step was to run the simulator to see how well the car was driving around track one. There were a few spots where the vehicle fell off the track. To improve the driving behavior in these cases, I collected more data when the vehicle recovers from the road side to the center lane. 

Then I went back to training the model using more data. After several times, I found the LeNet architecture does not serve as a very good model. To improve performance, I changed to using the CNN architecture used by Nvidia in their paper: End to End Learning for Self-Driving Cars. This is implemented in `Part 2.2: Apply Nvidia architecture`. Same as before, the model chosen has low training loss and low validation loss. Regularization is also included in the network to reduce overfitting. 

At the end of the process, the vehicle is able to drive autonomously around the track without leaving the road. The final code only has Nvidia model trained. 

#### 2. Final Model Architecture

The final model architecture is selected to be the CNN architecture first proposed by Nvidia. It consisted of a convolution neural network with the following layers and layer sizes:
1. a cropping layer to get rid of the sky and the front cover of the vehicle
2. a normalization layer
3. a convolutional layer with 5x5 kernel size, and a max pooling layer with 2x2 kernel size
4. a convolutional layer with 5x5 kernel size, and a max pooling layer with 2x2 kernel size
5. a convolutional layer with 5x5 kernel size, and a max pooling layer with 2x2 kernel size
6. a convolutional layer with 3x3 kernel size
7. a convolutional layer with 3x3 kernel size
8. fully connected layer with 100 hidden layers
9. fully connected layer with 50 hidden layers
10. fully connected layer with 10 hidden layers

Here is a visualization of the architecture:

![alt text][image1]

#### 3. Creation of the Training Set & Training Process

To capture good driving behavior, I first recorded two laps on track one using center lane driving. Here is an example image of center lane driving:

![alt text][image2]

I then recorded several laps when the vehicle recovering from the left side and right sides of the road back to center so that the vehicle would learn to recover center lane driving when it moves to the road sides. These images show what a recovery looks like starting from locating at the left side of road, to recovering to the lane center :

![alt text][image3]
![alt text][image4]
![alt text][image5]

Then I repeated this process on the track in order to get more data points.

To augment the data sat, I also flipped images and angles thinking that this would make the learned model adapt to more general scenario. For example, here is an image that has then been flipped:

![alt text][image6]


After the collection process, I had 54381 training data, and 11493 validation data, including center, left and right camera images before including the flipped images. 


I used this training data for training the model. The validation set helped determine if the model was over or under fitting. The ideal number of epochs was 8 as before this point, validation loss was decreasing, and after this point, validation loss starts to wander. I used an adam optimizer so that manually training the learning rate wasn't necessary.

This model is tested in the simulator and drives the car pretty well. 


#### 4. Testing in simulator and record video

* run.mp4 is the video recording of the vehicle driving autonomously at least one lap around the track. The vehicle is driven mostly smoothly, but there is one part in the trial where the vehicle gets away from the lane center, but it could recover from edges of the lane. 