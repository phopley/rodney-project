# Mission 1, Design Goal 2
To accomplish this design goal we will need to

* Add facial expression
* Add a voice
## Facial Expression
The hardware making up the robot head includes the 7" Touchscreen Display which is ideal for giving the robot a face, but utilising it for facial expressions seems like a big task. However, I have explained that there are many ROS packages available from the ROS community leaving us free to concentrate on the robot application and that's exactly what we are going to make use of here.

We will make use of the __homer_robot_face__ package from the University of Koblenz. This package includes two different selectable faces but it is also possible to model your own character. This package also includes speech synthesis using the Mary TTS (Text to Speech) generator. As this consumes a lot of memory, it is not suitable for single board computers, but we will write our own TTS node suitable for the Raspberry Pi later in this article.

This video from the University of Koblenz shows the range of facial expressions available with the package.
[![YouTube](http://img.youtube.com/vi/jgcztp_jAQE/0.jpg)](http://www.youtube.com/watch?v=jgcztp_jAQE "Homer Face")

To install the robot face package for ROS Kinetic, run the following command in a terminal.
```
$ sudo apt-get install ros-kinetic-homer-robot-face
```
The configuration of the face is done by editing the *config.cfg file*. It would have been more user friendly if you could pass the path of a configuration file to the node, but the location of the file appears to be hard code. It is therefore necessary to edit the file within the package folder. The folder */opt/ros/kinetic/share/homer_robot_face/config* contains the *config.cfg* file and a number of example files. The package comes with two sets of mesh files, 'Lisa' represents a female face and 'GiGo' a male face. For the Rodney project, I edited the config.cfg file to contain the following:
```
Mesh Filename : GiGo
Head Color : 1.0, 1.0, 1.0
Iris Color : 0.0, 1.0, 1.0
Outline Color : 0.0, 0.0, 0.0
Voice : male
Window Width : 600
Window Height : 600
Window Rotation : 0
```
Since we will be using our own speech synthesis node, the __Voice__ parameter is not actually used.

If you wish to get creative and design your own character, there are some notes on modeling a face at http://wiki.ros.org/robot_face.

We can test the installation and the configuration with the following.

In a terminal, start a ROS master node with the following command:

```
$ roscore
```
In a second terminal, start the robot face node with the following command:
```
$ rosrun homer_robot_face RobotFace
```
Running on my Linux PC, I got the following neutral facial expression.
