# ROS and the Raspberry Pi
In the document I’ll explain how I have utilised ROS on the project and describe how I have used some of the tools available in ROS to test my code. It is not a tutorial on ROS, there are plenty of tutorials available online that do a much better job than I ever could. From time to time I’ll include links to relevant tutorials on the ROS Wiki but for now, to aid a first read through of the document or the casual reader, here are some ROS terms that you may find useful.
* It’s a distributed system and the robot code can run on multiple machines communicating over a network
* A __node__ is a single purpose executable
* Nodes are organised into __packages__ which is a term used for a collection of folders and files
* Nodes can be written using many languages. In this project we will use C++ and Python
* Nodes communicate between each other using __Topics__ which are one way streams
* Topics are instances of __Messages__. Messages are data structures
* Standard messages are available from ROS and you can also create user defined messages
* Nodes can also communicate with each other using __Services__, a server/client blocking protocol
* Nodes can also communicate using __Actions__, a non-blocking goal orientated task protocol
* There is a master node, roscore, which all the other nodes register with. Only one master node exists, even when using a distributed system
* Uses a catkin build system
* Individual nodes can be run using the rosrun command or you can use the launch tool to start many nodes from the same command terminal
* It includes a __parameter server__. Nodes can store and retrieve parameters during runtime
* It includes various tools for examining the system and can even simulate robot hardware

There is a nice overview of ROS in [this Code Project article](https://www.codeproject.com/Articles/1229906/Build-an-Autonomous-Mobile-Robot-with-the-Intel-Re "this Code Project article").

So having made the decision to use a Raspberry Pi 3 as the main processor and to use ROS, the first stage is to install ROS on a Pi.

Instructions for downloading and installing ROS are [available here](http://wiki.ros.org/ROS/Installation "available here"), but to make life easier I’m going to use an Ubuntu image for the Raspberry Pi which includes ROS. You can down the image for free from the [Ubiquity Robotics](https://ubiquityrobotics.com/ "Ubiquity Robotics") website. The ROS version included in this image is the Kinetic version. The image also includes some useful ROS packages like the one for accessing the Raspberry Pi camera, [raspicam_node](https://github.com/ubiquityRobotics/raspicam_node "raspicam_node"). If you prefer to use a different image for your Raspberry Pi and install ROS yourself, you can still make use of packages from Ubiquity by downloading the code from [their GitHub site](https://github.com/UbiquityRobotics "their GitHub site").

Other Raspberry Pi peripherals that I intend to use on the Rodney project are:
- 7" Touchscreen Display
- Camera Module V2

The plan is to use the display for passing status information, web content to the user and also for displaying an animated robot face. The camera will be the eyes of the robot initially being used for facial recognition.

The following images show the 7" display with the Raspberry Pi and camera mounted on the rear of the screen. The camera is mounted using a 3D printed bracket. The stl file for the bracket is available [here]( https://github.com/phopley/rodney-project/blob/master/hardware/3D%20Prints/camera%20bracketV2.stl "Camera bracket")

<img src="https://github.com/phopley/rodney-project/blob/master/docs/images/Optimized-IMG_0380.JPG" width="427" height="284" title="7inch screen"> <img src="https://github.com/phopley/rodney-project/blob/master/docs/images/Optimized-IMG_0381.JPG" width="427" height="284" title="7inch screen and camera">

As the ROS system can run across a distributed network, I have also installed ROS on an Ubuntu desktop. This desktop PC will be used to develop the nodes for the system, to run some of the ROS tools available to test the code and to run simulations.
