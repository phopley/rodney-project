# Mission 1, Design Goal 1
To accomplish this design goal we will need to

* Control the head/camera using RC servos for pan/tilt movement
* Access images from the Raspberry Pi Camera
* Detect and recognize faces
* Control the order of these actions
## Pan and Tilt
To control the head/camera we need a pan and tilt device which will require two RC servos. I expect that the project will also in the future require a second pan/tilt device for a LIDAR (Light Detection and Ranging) sensor. We therefore straight away require four PWM outputs to control the servos, not to mention any required for motors in the future. The Raspberry Pi only has one hardware PWM and although we could make use of software PWMs, I'm goind to avoid that overhead by passing control of the servos off to a second board.

We could use a purpose built board like the one available from [PiBorg, the UltraBorg](https://www.piborg.org/sensors-1136/ultraborg "PiBorg, the UltraBorg"). Using this board you can connect up to four servos and four HC-SR04 ultrasonic devices to the Raspberry Pi using an I2C bus. However, since I have a number of Arduino Nano's available from a previous project, I'm going to make use of one of those.

This is also going to be our first of many examples in taking advantage of work already carried out by the ROS community, allowing us to concentrate on the robot application. To attach to the ROS like node which will be running on the Arduino, we are going to use a package that includes a node for communicating with the Arduino over the serial port and an Arduino library for use in the Arduino sketch. This package documentation is available on the ROS Wiki website [rosserial_arduino](http://wiki.ros.org/rosserial_arduino "[rosserial_arduino").

To make use of this package we need to install it on the ROS target and install the library in the Arduino IDE environment. We will also need to rebuild the Arduino library if we use any user defined ROS messages (which we will). How to do this and much more is covered on [rosserial arduino tutorials](http://wiki.ros.org/rosserial_arduino/Tutorials "rosserial arduino tutorials").

To control the position of each servo making up the pan/tilt devices we will write a ROS package whose node will take pan/tilt demand messages and turn them into individual position messages that will be sent to the Arduino. The first message will identify which pan/tilt device is to be moved and the required position for the pan servo and the tilt servo. The second message, sent to the Arduino, will contain an index value, indicating which of the four servos is to be moved, and a value for the angle to move that servo to. Splitting this functionality down means that the Arduino sketch only needs to understand about servos and not pan/tilt device, it therefore could be reused for other servo applications. BTW in the world of Arduino programming the code running on the Arduino is known as a sketch and I’ll continue to use that term here. If you are unfamiliar with Arduino programming there are plenty of articles about Arduinos on the Code Project site.

We could include the definition of our user defined messages in the pan tilt package but again in the interest of reuse we will create a separate package for the message definitions.

So to complete the pan/tilt functionality we are going to write two ROS packages and a ROS style Arduino sketch.

We will call the first of these packages *servo_msgs* and it will define our messages. When built it will produce .h files for use by C++ code and will automatically create Python scripts. We will also recompile the Arduino library to produce .h files that will be used by our sketch.

The files that make up this first package are available in the [servo_msgs repository](https://github.com/phopley/servo_msgs "servo_msgs repository"). The root of this folder contains a readme file documenting the package and two files that are required to always be present in a ROS package. These are the CmakeList.txt and the package.xml files, information about these files can be found in the tutorial on [creating ROS packages](http://wiki.ros.org/ROS/Tutorials/CreatingPackage "creating ROS packages").

The msg folder within the package contains the definition files for our messages. The first of these is *servo_array.msg*
``` Python
# index references the servo that the angle is for, e.g. 0, 1, 2 or 3
# angle is the angle to set the servo to
uint8 index
uint16 angle
```
You can think of this as a C like structure. This is the message which will be sent as a ROS topic to the Arduino. The message contains two elements, __index__ indicates which of the servos is to be moved and __angle__ is the angle to move the servo to in degrees.

The second message type will be sent as two topics to the pan and tilt node and is the *pan_tilt.msg*. One topic will relate to the head/camera pan tilt device and the other will be used later for possibly the LIDAR pan tilt device.
``` Python
int16 pan   # the angle for the pan servo
int16 tilt  # the angle for the tilt servo
```
Here the __pan__ element is the angle in degrees for the pan servo and the __tilt__ element is the angle in degrees for the tilt servo.

That completes our first simple ROS package, our second package is the *pan_tilt* package. This package is available in the [pan_tilt repository](https://github.com/phopley/pan_tilt "pan_tilt repository") and contains executable code which will form the *pan_tilt_node*.

The root folder of this package again includes a documentation file and the CmakeList.txt and package.xml files. This package includes a number of sub folders which I'll briefly describe. The *config* folder contains the file *config.yaml*. This file will be used by the launch file (see below) to set the given parameters in the parameter server. This will allow us to configure the system without having to recompile the code.
```
# Configuration for pan/tilt devices
# In Rodney index0 is for the head and index 1 is for the LIDAR
servo:
  index0:
    tilt_max: 100
    tilt_min: 0
    pan_servo: 0
    tilt_servo: 1
  index1:
    pan_servo: 2
    tilt_servo: 3
```
In this config file, __index0__ gives parameters for the head pan and tilt device and __index1__ for the possible 2nd pan and tilt device. The ___max__ and ___min__ allow us to restrict the travel of a servo and the ___servo__ parameters identify which servo (0-3) is attached to which pan/tilt device and in which position.

The *cfg* folder contains the file *pan_tilt.cfg*. This file is used by the dynamic reconfiguration server so that we can adjust the trim of the servos on the fly. As you can see the file is actually a Python script.
``` Python
#!/usr/bin/env python
PACKAGE = "pan_tilt"

from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()

gen.add("index0_pan_trim",  int_t, 0, "Index 0 - Pan Trim",  0,  -45, 45)
gen.add("index0_tilt_trim", int_t, 0, "Index 0 - Tilt Trim", 0,  -45, 45)
gen.add("index1_pan_trim",  int_t, 0, "Index 1 - Pan Trim",  0,  -45, 45)
gen.add("index1_tilt_trim", int_t, 0, "Index 1 - Tilt Trim", 0,  -45, 45)

exit(gen.generate(PACKAGE, "pan_tilt_node", "PanTilt"))
```
For a complete understanding of the dynamic reconfiguration server refer to the ROS Wiki [section dynamic reconfiguration](http://wiki.ros.org/dynamic_reconfigure "section dynamic reconfiguration"). For now in our file you can see that we add four parameters, one for each servo and that the default value of each parameter is zero with the minimum value set to -45 and the maximum value set to 45.

The *launch* folder contains launch files which enable us to not only to load configuration files but to start all the nodes that make up a system. In our folder we have a *pan_tilt_test.launch* file which is used for testing just the pan/tilt part of the Rodney system. As you can see below this is an xml formatted file.
``` XML
<?xml version="1.0" ?>
<launch>
  <rosparam command="load" file="$(find pan_tilt)/config/config.yaml" />
  <node pkg="pan_tilt" type="pan_tilt_node" name="pan_tilt_node" output="screen">
    <remap from="pan_tilt_node/index0_position" to="pan_tilt_node/head_position" />
  </node>
  <node pkg="rosserial_python" type="serial_node.py" name="serial_node" output="screen" args="/dev/ttyUSB0" />
</launch>
```
For a complete understanding of launch files refer to the ROS Wiki [section on launch files](http://wiki.ros.org/roslaunch/XML "section on launch files"). Our launch file first finds and loads our config file.
``` XML
<rosparam command="load" file="$(find pan_tilt)/config/config.yaml" />
```
The next set of tags will result in our *pan_tilt_node* being executed and remaps one of the topics so that we can easily see that it is the pan/tilt message for the head/camera. This is another little trick to help with reuse of the package. Notice also that with __output="screen"__ we will direct any logging messages to the terminal that we launched from.
``` XML
<node pkg="pan_tilt" type="pan_tilt_node" name="pan_tilt_node" output="screen">
 <remap from="pan_tilt_node/index0_position" to="pan_tilt_node/head_position" /> </node>
```
The last tag in the file will result in the running of the rosserial node which communicates with the Arduino. You can see the argument which selects the serial port connected to the Arduino, __args="/dev/ttyUSB0"__
``` XML
<node pkg="rosserial_python" type="serial_node.py" name="serial_node" output="screen" args="/dev/ttyUSB0" />
```
The remaining folders (*include* and *src*) contain the C++ code for the package. For this package we have one C++ class, __PanTiltNode__ and a main routine contained within the pan_tilt_node.cpp file.

The main routine informs ROS of our node, creates a instance of our class which contains the code for the node, passes a callback function to the dynamic reconfiguration server and hands control to ROS spin which will handle the incoming topics and the posting of outgoing topics. 
``` C++
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pan_tilt_node");    
    
    PanTiltNode *pan_tiltnode = new PanTiltNode();
    
    dynamic_reconfigure::Server<pan_tilt::PanTiltConfig> server;
    dynamic_reconfigure::Server<pan_tilt::PanTiltConfig>::CallbackType f;
      
      f = boost::bind(&PanTiltNode::reconfCallback, pan_tiltnode, _1, _2);
    server.setCallback(f);
        
    std::string node_name = ros::this_node::getName();
    ROS_INFO("%s started", node_name.c_str());
    ros::spin();
    return 0;
}
```
The constructor for our class sets some parameter defaults which will be used if we don't load the parameter server with our configuration file.
``` C++
// Constructor 
PanTiltNode::PanTiltNode()
{
    // Set default values
    pan_servo_[0] = 0;
    tilt_servo_[0] = 1;
    pan_servo_[1] = 2;
    tilt_servo_[1] = 3;
    
    pan_max_[0] = 180;
    pan_min_[0] = 0;
    tilt_max_[0] = 180;
    tilt_min_[0] = 0;
    pan_max_[1] = 180;
    pan_min_[1] = 0;
    tilt_max_[1] = 180;
    tilt_min_[1] = 0;

    // Get any parameters from server, will not change after startup 
    n_.param("/servo/index0/pan_servo", pan_servo_[0], pan_servo_[0]);
    n_.param("/servo/index0/tilt_servo", tilt_servo_[0], tilt_servo_[0]);
    n_.param("/servo/index1/pan_servo", pan_servo_[1], pan_servo_[1]);
    n_.param("/servo/index1/tilt_servo", tilt_servo_[1], tilt_servo_[1]);
    n_.param("/servo/index0/pan_max", pan_max_[0], pan_max_[0]);
    n_.param("/servo/index0/pan_min", pan_min_[0], pan_min_[0]);
    n_.param("/servo/index0/tilt_max", tilt_max_[0], tilt_max_[0]);
    n_.param("/servo/index0/tilt_min", tilt_min_[0], tilt_min_[0]);
    n_.param("/servo/index1/pan_max", pan_max_[1], pan_max_[1]);
    n_.param("/servo/index1/pan_min", pan_min_[1], pan_min_[1]);
    n_.param("/servo/index1/tilt_max", tilt_max_[1], tilt_max_[1]);
    n_.param("/servo/index1/tilt_min", tilt_min_[1], tilt_min_[1]);

    pan_tilt_sub_[0] = n_.subscribe("pan_tilt_node/index0_position", 10, &PanTiltNode::panTilt0CB, this);
    pan_tilt_sub_[1] = n_.subscribe("pan_tilt_node/index1_position", 10, &PanTiltNode::panTilt1CB, this);

    servo_array_pub_ = n_.advertise<servo_msgs::servo_array>("servo", 10, true);
}
```
The calls to __param__ will read the parameter from the server if it is available, otherwise the default value will be used.
``` C++
n_.param("/servo/index0/pan_servo", pan_servo_[0], pan_servo_[0]);
```
The last three lines of the constructor subscribe to the topics and advertises which topics our node will be publishing. The subscribe calls are passed the callback functions to be called when the respective topic arrives.

We have two similar callback functions for the topics we subscribe to. The function stores the request position value and makes a call to the helper function __movePanTilt__ passing the requested position and the latest trim values.
``` C++
// Callback to move the pan tilt device indexed 0
void PanTiltNode::panTilt0CB(const servo_msgs::pan_tilt& pan_tilt)
{
    // Store lastes value in case we change thte trim
    index0_pan_tilt_ = pan_tilt;
    
    movePanTilt(pan_tilt, index0_pan_trim_, index0_tilt_trim_, 0);
}
```
The callback then calls the function __movePanTilt__. This function adds in the trim offset for the relevant pan, trim servos, checks if the range should be limited and then publish the two messages with the servo index and position. The two messages published are of the same type, one is for the relevant pan servo and the second is for the relevant tilt servo.
``` C++
void PanTiltNode::movePanTilt(const servo_msgs::pan_tilt& pan_tilt, int pan_trim, int tilt_trim, int index)
{
    int pan;
    int tilt;
    servo_msgs::servo_array servo;

    pan = pan_trim + pan_tilt.pan;
    tilt = tilt_trim + pan_tilt.tilt;

    pan = checkMaxMin(pan, pan_max_[index], pan_min_[index]);
    tilt = checkMaxMin(tilt, tilt_max_[index], tilt_min_[index]);

    // Send message for pan position
    servo.index = (unsigned int)pan_servo_[index];
    servo.angle = (unsigned int)pan;
    servo_array_pub_.publish(servo);

    // Send message for tilt position
    servo.index = (unsigned int)tilt_servo_[index];
    servo.angle = (unsigned int)tilt;
    servo_array_pub_.publish(servo);    
}
```
There is another helper function which is used to check for the max/min range.
``` C++
int PanTiltNode::checkMaxMin(int current_value, int max, int min)
{
    int value = current_value;

    if (value > max)
    {
        value = max;
    }

    if (value < min)
    {
        value = min;
    }

    return (value);
}
```
The dynamic parameter server callback, stores each of the trim parameters and then makes two calls to __movePanTilt__, one for each pan/tilt device, with the last position value and the latest trim values.
``` C++
// This callback is for when the dynamic configuration parameters change
void PanTiltNode::reconfCallback(pan_tilt::PanTiltConfig &config, uint32_t level)
{
    index0_pan_trim_ = config.index0_pan_trim;
    index0_tilt_trim_ = config.index0_tilt_trim;
    index1_pan_trim_ = config.index1_pan_trim;
    index1_tilt_trim_ = config.index1_tilt_trim;

    // Send new messages with new trim values
    movePanTilt(index0_pan_tilt_, index0_pan_trim_, index0_tilt_trim_, 0);
    movePanTilt(index1_pan_tilt_, index1_pan_trim_, index1_tilt_trim_, 1);
}
```
The pan_tilt_node.h file contains the definitions for our __PanTiltNode__ class.

Having completed the pan tilt package the last coding task we need to do is write the Arduino sketch. The sketch contains many of the elements used in the pan/tilt node. Our sketch is based on the servo tutorial for rosserial but we have to modified it to access more than one servo, which also includes subscribing to our user defind message.

Each Arduino sketch includes a setup and loop procedure. Our setup procedure sets pin 13 to an output. On the Arduino Nano pin 13 is connected to an onboard LED and we will toggle the state of the LED each time a servo message arrives. We then initialise the node and subscribe to the servo topic. The remainder of the setup procedure attaches the pins 9, 6, 5 and 3 to the four instances of Servo.

The loop procedure simply calls spinOnce and then delays for 1ms. The call to spinOnce will handle the receipt  of the topic.

Attached to the receipt of the servo topic is the callback function servo_cb. This function will be called each time the servo topic message is received, it then simply adjusts the PWM output for the indexed servo.

The sketch is in the *rodney/rondey_control* folder.

``` C++
/*
 * Based on the rosserial Servo Control Example
 * This version controls upto four RC Servos
 * The node subscribes to the servo topic and acts on a rodney_msgs::servo_array message.
 * This message contains two elements, index and angle. Index references the servos 0-3 and 
 * angle is the angle to set the servo to 0-180.
 *
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <servo_msgs/servo_array.h>

/* Define the PWM pins that the servos are connected to */
#define SERVO_0 9
#define SERVO_1 6
#define SERVO_2 5
#define SERVO_3 3

ros::NodeHandle  nh;

Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;

void servo_cb( const servo_msgs::servo_array& cmd_msg){
  
  /* Which servo to drive */
  switch(cmd_msg.index)
  {
    case 0:
      nh.logdebug("Servo 0 ");
      servo0.write(cmd_msg.angle); //set servo 0 angle, should be from 0-180
      break;

    case 1:
      nh.logdebug("Servo 1 ");
      servo1.write(cmd_msg.angle); //set servo 1 angle, should be from 0-180
      break;

    case 2:
      nh.logdebug("Servo 2 ");
      servo2.write(cmd_msg.angle); //set servo 2 angle, should be from 0-180
      break;

    case 3:
      nh.logdebug("Servo 3 ");
      servo3.write(cmd_msg.angle); //set servo 3 angle, should be from 0-180
      break;
      
    default:
      nh.logdebug("No Servo");
      break;
  }
    
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

ros::Subscriber<servo_msgs::servo_array> sub("servo", servo_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
  
  servo0.attach(SERVO_0); //attach it to the pin
  servo1.attach(SERVO_1);
  servo2.attach(SERVO_2);
  servo3.attach(SERVO_3);
  
  // Defaults
  servo0.write(90);
  servo1.write(40);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
```
## Using the code
Before we can compile the sketch and program the Arduino we first need to build our ROS packages and recompile the ROS Arduino library. We need to do this so that our user defined message, servo_array, is available in the Arduino IDE.
### Bulding the ROS packages
As I'll use the Linux workstation to run the Arduino IDE I’m going to build our packages on both the workstation and the Raspberry Pi. Since at this stage we are not using any dedicated Raspberry Pi hardware you could just run the nodes entirely on a workstation. I'm going to run the nodes on the Raspberry Pi and run the test tools on the workstaion.

ROS uses the catkin build system so first we will create a catkin workspace and initialise the workspace. In a command terminal enter the following commands:
```
$ mkdir -p ~/rodney_ws/src
$ cd ~/rodney_ws/
$ catkin_make
```
Copy the two package folders *pan_tilt* and *servo_msgs* into the *~/rodney_ws/src* folder and then build the code with the following commands.
```
$ cd ~/rodney_ws/ 
$ catkin_make
```
Check that the build completes without any errors.
### Build the Arduino ROS library
We could actually run the code now on the worksation but before we do that let's rebuild the Arduino library, compile the sketch and program the Arduino Nano.

I have the Arduino IDE installed on the workstation and at the time of the installation it created an Arduino folder in my home directory. Use the following commands to build the ros_lib library.
```
$ cd ~/Arduino/libraries
$ rosrun rosserial_arduino make_libraries.py .
```
 Check the build completes without any errors and check that the servo_array.h file was created in the ~/Arduino/libraries/ros_lib/servo_msgs folder.
### Building the servo sketch and programming the Arduino
Copy the rodney_control folder to the ~/Arduino/Projects folder. Start the Arduino IDE and open the rodney_control.ino file. From the Tools->Board menu select the Arduino board you are using. In my case it's the Nano. From the Tools->Processor menu select the processor. In my case it's the ATmega328P.

Build the sketch and check there are no errors.

To program the Arduino connect the device to a workstation USB port. In the IDE from the Tools->Port menu select the serial port that the Arduino is connected to. In my case it's /dev/ttyUSB0. 

Next upload the sketch to the Arduino and check there are no errors reported.
### Arduino circuit
When we construct Rodney we will need to give some thought about power. For now I'm going to power the Arduino using the USB port of the Raspberry Pi, the servos will be powered from 4xAA rechargeable batteries. Below is a circuit which shows the servo connections and the power to the servos.
<img src="https://github.com/phopley/rodney/blob/master/docs/images/Optimized-Nano%20prototpe_schem.png" title="Test Circuit">
For now to test the software I'm going to build the circuit on a bread board and only connecting the servos for the head pan and tilt device.
<img src="https://github.com/phopley/rodney/blob/master/docs/images/Optimized-IMG_0387.JPG" title="Test Circuit bread board">
### Running the code
Now we are ready to run our code.

The package versions used in this test were:
* pan_tilt V0.1.4 [pan_tilt repository](https://github.com/phopley/pan_tilt "pan_tilt repository")
* servo_msgs V0.1.2 [servo_msgs repository](https://github.com/phopley/servo_msgs "servo_msgs repository")

With the Arduino connected to a USB port use the launch file to start the nodes with the following commands. If no master node is running in a system the launch command will also launch the master node, roscore.
```
$ cd ~/rodney_ws/
$ source devel/setup.bash
$ roslaunch pan_tilt pan_tilt_test.launch
```
In the terminal you should see:

* a list of parameters now in the parameter server
* a list of the nodes which should show pan_tilt_node and serial_node
* the address of the master
* the starting of the two nodes
* log information from our code
We can now use some of the ROS tools to examine, interact and test the system.

To test the expected nodes are running and connected using the topics open a second command terminal and type the following command:
```
$ cd ~/rodney_ws
$ source devel/setup.bash
```
If you launched the nodes on one device, for example the Raspberry Pi and want to run the tools on a second device you need to tell the second device where to find the master. In the same terminal type:
```
$ export ROS_MASTER_URI=http://ubiquityrobot:11311
```
Now in the same terminal start the graph tool
```
$ rqt_graph
```
<img src="https://github.com/phopley/rodney/blob/master/docs/images/rosgraph_pantiltv2.png" title="Pan Tilt Graph">
From the graph you can see the two nodes are running and are connected by the /servo topic. You can also see that the topic /pan_tilt_node/index0_position has been remapped to /pan_tilt_node/head_position.

We will now open a third terminal and send a message to move the pan/tilt device using rostopic. In a new terminal enter the following commands, don't forget to give the location of the master if running on a different device to that you launch the nodes on.
```
$ cd ~/rodney_ws
$ source devel/setup.bash
$ export ROS_MASTER_URI=http://ubiquityrobot:11311
$ rostopic pub -1 /pan_tilt_node/head_position servo_msgs/pan_tilt {0,45}
```
The last command will result in rostopic publishing one instance of the /pan_tilt_node/head_position topic of message type servo_msgs/pan_tilt with the pan position 0 degrees and the tilt position 45 degrees. The order of the parameters is given in our *pan_tilt.msg* file. If all worked fine the servos will move to the position given.

Due to the mechanical fittings of the pan/tilt device it may be off centre by a number of degrees. You can trim the servos with the following procedure:

Set the position of both servos to the mid positions, say 90 degrees for the pan and 45 degrees for the tilt.
```
$ rostopic pub -1 /pan_tilt_node/head_position servo_msgs/pan_tilt {90,45}
```
In a new terminal start *rqt_reconfigure* with the following commands, don't forget to give the location of the master if running on a different device.
```
$ cd ~/rodney_ws 
$ source devel/setup.bash 
$ export ROS_MASTER_URI=http://ubiquityrobot:11311 
$ rosrun rqt_reconfigure rqt_reconfigure
```
This will bring up a user interface like the one shown below. Trim parameters can be dynamically adjusted via the interface.
<img src="https://github.com/phopley/rodney/blob/master/docs/images/pan_tilt_trim.png" title="Pan Tilt Trim">
Once you are happy with the trim values you can edit the pan_tilt.cfg to include the new trim values as the defaults. Then the next time the nodes are started these trim values will be used.

To terminate the nodes simply hit Ctrl-c in the terminal.
### Head pan tilt device
For the Pan/Tilt device I’m using two Futaba S3003 servos. You can purchase pan/tilt devices for various size servos, however I chose to 3D print my own version and the stl files for the parts are available in the zip file. I had some concern about the combined weight of the display and Raspberry Pi exerting sideways torque on the pan servo shaft, so I have used a load bearing servo block to alleviate this problem. This unit acts as a servo exoskeleton which enhances the mechanical loads the servo can withstand. It add expense to the robot so an alternative would be to mount just the camera on a smaller pan/tilt device and have the screen fixed in position. The servos I’m using have plastic gears so another alternative could be to use ones with metal gears to withstand this sideways torque, that said the load bearing block is excellent and appears to work well. The following images show the load bearing servo block and the pan/tilt arrangement.

<img src="https://github.com/phopley/rodney/blob/master/docs/images/servoblock.jpg" width="300" height="317" title="Load bearing block"> <img src="https://github.com/phopley/rodney/blob/master/docs/images/Opt-IMG_0388.JPG" width="320" height="213" title="Pan Tilt 1">
<img src="https://github.com/phopley/rodney/blob/master/docs/images/Opt-IMG_0389.JPG" width="320" height="213" title="Pan Tilt 2"> <img src="https://github.com/phopley/rodney/blob/master/docs/images/Opt-IMG_0391.JPG" width="320" height="213" title="Pan Tilt 3">
