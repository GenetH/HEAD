# Simple Visual System Needs

This repository contains the code for detecting room luminance in general. Currently, it includes the following features:

* General Room Luminance Detection and Classification.
* Hand/Object Block Recognition

Moreover, the repository will have the following features by the end of development.
* Room Silence
* Room Occupation
* and other simple visual system needs

## Install, Build and Run w/ HEAD
* Refer [HEAD] (https://github.com/hansonrobotics/HEAD#install-build-and-run)
## Run Room Luminance Only
* navigate to your workspace
* `roslaunch roomluminance room_luminance.launch`

## Topics and Published Messages Type
### Topics
* `/opencog/room_luminance`
### Custom Messages
* Message File: `Luminance.msg`
  * `string brightness`: string that holds the luminance(DARK, NOMINAl, and BRIGHT) of a given ROI. 
  * `float32 coverage`: This is the percent of screen covered by close enough objects. Very  close objects by themselves may cover the camera and affect it to have very small luminance. On the contrary, the robot camera could be covered without affecting the amount of light applied on the camera. In this scenario, the robot must know that it is covered by some object/hand.

