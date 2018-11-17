# Mission 1, Design Goal 3
To accomplish this design goal we will need to

* Add teleop control
* Add locomotion by means of electric motors

In this part we will also add a state machine that will control the missions and add a 2nd mission which makes use of Design Goal 1 and 2.

## A complex plan
### smach
When we are finally ready to bring all these Design Goals together it's going to require a complex system to order and control all the various parts of the system. To do this we are going to use a ROS Python library called smach. The package documentation is available on the ROS Wiki website [smach](http://wiki.ros.org/smach "smach").

With smach we can develop a hierarchical state machine where we add a lower level state machine for each mission we add.

### Gluing Design Goal 1 and 2 together
Although our overall aim is what we have defined as Mission 1, it would be nice to start working on this control mechanism. What we can do is combine Design Goal 1 and 2 into another smaller mission (Mission 2) which is required to search for recognised faces within the head movement range and speak a greeting to anyone that the robot recognises. The processes used for Mission 2 will also form part of Mission 1 when it is complete.

To complete Mission 2 in this article we are going to write two nodes. The first node, *rodney_missions* will contain the code for the state machine making up the missions and jobs. The second node *rodney*, will be used to control when various missions and jobs are started. We will also take the opportunity to add some functionality for reading the keyboard and a game controller which will be used in Design Goal 3.

Now I'm fully aware that I have introduced a new term there alongside "missions" and that is "jobs". A job will be a task that the robot needs to carry out but is not as complex as a full mission. The node running the state machines is the best place for these "jobs" as they may require the same resources as the more complex missions. For example the mission 1 state machine is required to request the movement of the head/camera but we may also want to be able to move the head/camera manually. Although it's fine to have two nodes subscribing to a topic it's considered bad practice to have two nodes publishing the same topic. So we will avoid this by having one node to action the "missions" and "jobs".

Up to now I have kept the node names generic and not named them after this particular robot. This was so that the nodes could be used in other projects, however these two nodes will be particular to this robot so they are named after it.

### State machine
We will start with the package and node which contains the state machine that controls the different missions and jobs that the robot is capable of. As stated above smach is a Python library so our package will be written in Python.

Our ROS package for the node is called *rodney_missions* and is available in the [rodney_missions repository](https://github.com/phopley/rodney_missions "rodney_missions repository"). The *src* folder contains the *rodney_missions_node.py* file, which contains the main code for the node. The *src* folder also contains a sub folder called *missions_lib*, each of the robot missions we add to Rodney will result in a Python class which will be contained in this folder. Here we are going to work on Mission 2 and the code for that is in the *greet_all.py* file.

The *rodney_missions_node.py* file will contain the code to register the node and will also contain the high level state machine which accepts each mission and job. The *greet_all.py* file will contain part of the sub state machine for mission 2. Each time we add a new mission to the robot we will add a sub state machine for that mission.

The diagram below shows our state machine.
<img src="https://github.com/phopley/rodney-project/blob/master/docs/images/Opti-statemachine1.png" title="State Machine">

The WAITING state is a special type of state called a __MonitorState__ and simply monitors the */missions/mission_request* topic. When a message is received on this topic it will extract the request and any parameters that go with the request and then transit to the PREPARE state passing on the request data.

The PREPARE state will carry out any 'Job' requests and then transit back to the waiting state. If the request was to carryout Mission 2 then it will transit to the sub state machine MISSION2. 

The SCANNING state is another special state called a __SimpleActionState__. If you look back to [Design Goal 1 Part 3 Head Control](https://github.com/phopley/rodney-project/blob/master/docs/006%20Mission%201%20Design%20Goal%201%20Part%203.md "Head Control"), we wrote an action server in the *head_control* node. This action server was responsible for coordinating the head movement and when to attempt to do the face recognition of the captured image. At the time we wrote an action client in a piece of test code so that we could test the functionality. This state will replace the action client for this action. As we develop the robot and want to move the head for other missions, we might remove the *head_control* node and move the functionality into the *rodney_missions* node. For now I'm leaving it as it is as an example of how to use the __SimpleActionState__.

If the action is completed successfully the state machine will transit to the GREETING state where a spoken greeting for all the individuals recognised will be generated. The state machine will then transit to the REPORT state.

The REPORT state simply sends the mission complete message on the */missions/mission_complete* topic and transits back to the WAITING state.

Before I explain the code it is worth stating what is contained in the */missions/mission_request* topic. It is of type std_msgs/String and contains an ID for the Mission or the Job and depending on the ID zero or more parameters separated by the '^' character.

Currently the IDs and parameters are as follows

* "M2" This ID is a request to conduct Mission 2 and there are no parameters associated with it.
* "J1" Is a request to conduct Job 1. This job is to playback the supplied wav file and to pass matching text to the robot face to animate the lips. The first parameter following the ID is the wav file name and the second parameter is the matching text.
* "J2" Is a request to conduct Job 2. This Job is to speak the supplied text and to pass the matching text to the robot face to animate the lips. The first parameter is the text to speak and the second parameter is the matching text. Remember these are separate as the text for the robot face may contain smileys for the robot face expression.
* "J3" Is a request to conduct Job 3. This Job is to move the position of the head/camera. The first parameter will contain the letter 'u' if the camera is to be moved up, 'd' if the camera is to be moved down, or '-' if the camera is not to be moved up or down. The second parameter contains 'l' if the camera is to be moved left, 'r' if the camera is to be moved right, or '-' if the camera is not to be moved left to right.

I'll now briefly describe the code starting with the *rodney_missions_node.py* file.

The __main__ function registers our node with ROS and creates an instance of the __RodneyMissionsNode__ class.
``` Python
def main(args):
    rospy.init_node('rodney_missions_node', anonymous=False)
    rospy.loginfo("Rodney missions node started")
    rmn = RodneyMissionsNode()        

if __name__ == '__main__':
    main(sys.argv)
```
The class constructor for __RodneyMissionsNode__ sets up to call __ShutdownCallback__ if the node is shutdown, and subscribes to the */missions/mission_cancel* topic. It then creates each state and adds the states to the state machine. This includes creating the two special type of states the __MonitorState__ and the __SimpleActionState__.

We then create and start an introspective server. This is not required for the robot to operate but allows you to run a tool called [smach_viewer](http://wiki.ros.org/smach_viewer "smach_viewer"). This tool can help to debug any problems with your state machine and was used to produce the state diagram above.

The constructor then starts the execution of the state machine and hands control over to ROS.

There are three other functions in the __RodneyMissionsNode__ class.

__MissionsRequestCB__ is the function called by the MonitorState WAITING when a message is received on the */missions/mission_request* topic. This extracts the data from the message and copies it to __userdata__ which is a process for passing data between states. It then returns __False__ so that the state machine will transit to the PREPARE state.

__CancelCallback__ is the callback function called if a message is received on the */missions/mission_cancel* topic. This will result in SCANNING state transiting back to WAITING should the state machine be in that state at the time.

__ShutdownCallback__ is the callback function called if the node receives a command from ROS to shutdown. It again will cancel the action associated with the SCANNING state.
``` Python
# Top level state machine. The work for each mission is another state machine in the 'mission' states        
class RodneyMissionsNode:

    def __init__(self):
        rospy.on_shutdown(self.ShutdownCallback)
        
        # Subscribe to message to cancel missions        
        self.__cancel_sub = rospy.Subscriber('/missions/mission_cancel', Empty, self.CancelCallback)
        
        # Create top level state machine
        self.__sm = StateMachine(['preempted'])
        with self.__sm:
            # Add the first state which monitors for a mission to run
            StateMachine.add('WAITING',
                             MonitorState('/missions/mission_request',
                             String,
                             self.MissionRequestCB,
                             output_keys = ['mission']),
                             transitions={'valid':'WAITING', 'invalid':'PREPARE', 'preempted':'preempted'}) 
            # Add state to prepare the mission
            StateMachine.add('PREPARE',
                             Prepare(),
                             transitions={'mission2':'MISSION2','done_task':'WAITING'})
            # Add the reporting state
            StateMachine.add('REPORT',
                             Report(),
                             transitions={'success':'WAITING'})
                             
            # Create a sub state machine for mission 2 - greeting
            self.__sm_mission2 = StateMachine(['success', 'aborted', 'preempted'])
            
            with self.__sm_mission2:
                goal_scan = scan_for_facesGoal()                
                StateMachine.add('SCANNING',
                                 SimpleActionState('head_control_node',
                                                   scan_for_facesAction,
                                                   goal=goal_scan,
                                                   result_slots=['detected']),                                 
                                 transitions={'succeeded':'GREETING', 'aborted':'aborted', 'preempted':'preempted'})
                StateMachine.add('GREETING',
                                 missions_lib.Greeting(),                                 
                                 transitions={'success':'success'})
                                 
            # Now add the sub state machine (for mission 2) to the top level one
            StateMachine.add('MISSION2', 
                             self.__sm_mission2, 
                             transitions={'success':'REPORT', 'aborted':'WAITING', 'preempted':'WAITING'}) 
        
        # Create and start the introspective server so that we can use smach_viewer
        sis = IntrospectionServer('server_name', self.__sm, '/SM_ROOT')
        sis.start()
                             
        self.__sm.execute()
        
        # Wait for ctrl-c to stop application
        rospy.spin()
        sis.stop()
        
    
    # Monitor State takes /missions/mission_request topic and passes the mission in user_data to the PREPARE state
    def MissionRequestCB(self, userdata, msg):                
        # Take the message data and send it to the next state in the userdata
        userdata.mission = msg.data;       
                        
        # Returning False means the state transition will follow the invalid line
        return False
        
    # Callback for cancel mission message
    def CancelCallback(self, data):
        # List all sub state machines which can be preempted
        self.__sm_mission2.request_preempt()
        
    def ShutdownCallback(self):        
        self.__sm.request_preempt()
        # Although we have requested to shutdown the state machine 
        # it will not happen if we are in WAITING until a message arrives
```
The *rodney_missions_node.py* file also contains classes that make up the PREPARE and REPORT states.

The class __Prepare__ contains a constructor which declares which state follows PREPARE, what data is passed to it and advertises that it will publish messages on the topics */speech/to_speak*, */robot_face/text_out* and */head_control_node/manual*.

The class also contains an __execute__ function which is run when the state is entered. This function examines the request message, carries out any Jobs it can and then transits to the WAITING state or transits to the SCANNING state if Mission 2 is requested.
``` Python
# The PREPARE state
class Prepare(State):
    def __init__(self):
        State.__init__(self, outcomes=['mission2','done_task'], input_keys=['mission'])
        self.__speech_pub_ = rospy.Publisher('/speech/to_speak', voice, queue_size=5)
        self.__text_out_pub = rospy.Publisher('/robot_face/text_out', String, queue_size=5)
        self.__man_head = rospy.Publisher('/head_control_node/manual', String, queue_size=1)
    
    def execute(self, userdata):        
        # Based on the userdata either change state to the required mission or carry out single job
        # userdata.mission contains the mission or single job and a number of parameters seperated by '^'
        retVal = 'done_task';
        
        # Split into parameters using '^' as the delimiter
        parameters = userdata.mission.split("^")
        
        if parameters[0] == 'M2':
            # Mission 2 is scan for faces and greet those known, there are no other parameters with this mission request
            retVal = 'mission2'
        elif parameters[0] == 'J1':
            # Simple Job 1 is play a supplied wav file and move the face lips
            voice_msg = voice()
            voice_msg.text = ""
            voice_msg.wav = parameters[1]            
            # Publish topic for speech wav and robot face animation
            self.__speech_pub_.publish(voice_msg)
            self.__text_out_pub.publish(parameters[2])
        elif parameters[0] == 'J2':
            # Simple Job 2 is to speak the supplied text and move the face lips
            voice_msg = voice()
            voice_msg.text = parameters[1]
            voice_msg.wav = ""
            # Publish topic for speech and robot face animation
            self.__speech_pub_.publish(voice_msg)
            self.__text_out_pub.publish(parameters[2])
        elif parameters[0] == 'J3':
            # Simple Job 3 is to move the head/camera. This command will only be sent in manual mode. The resultant
            # published message will only be sent once per received command.
            # parameters[1] will either be 'u', 'd', 'c' or '-'
            # parameters[2] will either be 'l', 'r' or '-'             
            self.__man_head.publish(parameters[1]+parameters[2])
        return retVal
```
The class __Report__ contains a constructor which declares which state follows REPORT and advertises that it will publish a message on the */missions/mission_complete* topic. 

The class also contains an __execute__ function which is run when the state is entered. This function simply publishes the message for the */missions/mission_complete* topic.
``` Python
# The REPORT state
class Report(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.__pub = rospy.Publisher('/missions/mission_complete', String, queue_size=5)
    
    def execute(self, userdata):        
        # Publishes message that mission completed
        self.__pub.publish("Mission Complete")
        return 'success'  
```
The only state we now need to write code for is the GREETING state.

This is in the *greet_all.py* file and contains the __Greeting__ class. The constructor declares the state to follow it, what data is passed to the state and that it will publish on the topics */speech/to_speak* and */robot_face/text_out*.

The class also contains an __execute__ function which is run when the state is entered. This function constructs and publishes the two topics based on the data passed to it.
``` Python
# Greeting State
class Greeting(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'],
                       input_keys=['detected'])
        self.__speech_pub_ = rospy.Publisher('/speech/to_speak', voice, queue_size=5)
        self.__text_out_pub = rospy.Publisher('/robot_face/text_out', String, queue_size=5)        
    
    def execute(self, userdata):        
        # userdata.detected.ids_detected is the IDs of those detected
        # userdata.detected.names_detected is the name of thise detected
        
        # Construct greeting
        greeting = ''
        if len(userdata.detected.names_detected) == 0:
            greeting = 'No one recognised'
        else:
            greeting = 'Hello '
            for n in userdata.detected.names_detected:
                greeting += n + ' '
                
            greeting += 'how are you '
            
            if len(userdata.detected.names_detected) == 1:
                greeting += 'today'
            elif len(userdata.detected.names_detected) == 2:
                greeting += 'both'
            else:
                greeting += 'all'
            
        rospy.loginfo(greeting)
        
        voice_msg = voice()
        voice_msg.text = greeting
        voice_msg.wav = ""
        
        # Publish topic for speech and robot face animation
        self.__speech_pub_.publish(voice_msg)
        self.__text_out_pub.publish(greeting + ":)")
        
        return 'success'
```
### Top level control
The *rodney* node will be responsible for the top level control of the robot.

Our ROS package for the node is called *rodney* and is available in the [rodney repository](https://github.com/phopley/rodney "rodney repository"). The package contains all the usual ROS files and folders plus a few extra.

The *config* folder contains a *config.yaml* file which can be used to override some of the default configuration values. You can configure:

* The game controller axis which is used for moving the robot forward and backward in manual locomotion mode
* The game controller axis which is used for moving the robot clockwise and anti-clockwise in manual locomotion mode
* The game controller axis which will be used for moving the head/camera up and down in manual locomotion mode
* The game controller axis which will be used for moving the head/camera left and right in manual locomotion mode
* The game controller button which will be used for selecting manual locomotion mode
* The game controller button which will be used for moving the head/camera back to the default position
* The game controller axes dead zone value
* The linear velocity which is requested when the controller axis is at its maximum range
* The angular velocity which is requested when the controller axis is at its maximum range
* The ramp rate used to increase or decrease the linear velocity
* The ramp rate used to increase or decrease the angular velocity
* The battery voltage level that a low battery warning will be issued at
* Enable/disable the wav file playback functionality when the robot is inactive
* A list of wav filenames to play from when the robot is inactive
* A list of speeches to use when playing the wav files names

The *launch* folder contains two launch files, *rodney.launch* and *rviz.launch*. The *rodney.launch* file is used to load all the configuration files, covered in the first articles, into the parameter server and to start all the nodes that make up the robot project. It is similar to the launch files used so far in the project except it now includes the *rodney_node* and the *rodney_missions_node*. rviz is a 3D visualization tool for ROS which can be used to visualise data including the robot position and pose. Documentation for [rviz is available on the ROS Wiki website](http://wiki.ros.org/rviz "rviz"). The *rviz.launch* file along with the *meshes*, *rviz* and *urdf* folders can be used for visualising Rodney. We will use the urdf model of Rodney to do some testing on a simulated Rodney robot.

The image below shows a visualisation of Rodney in rviz.
<img src="https://github.com/phopley/rodney-project/blob/master/docs/images/Opti-rviz_2wd.png" title="rviz">

The *rodney_control* folder is just a convenient place to store the Ardunio file that was discussed earlier in these articles.

The *sounds* folder is used to hold any wav files that the system is required to play. How to play these files and at the same time animate the robot face was covered earlier in these articles.

The *include/rodney* and *src* folders contain the C++ code for the package. For this package we have one C++ class, __RodneyNode__, and a __main__ routine contained within the *rodney_node.cpp* file.

The __main__ routine informs ROS of our node, creates an instance of the node class and passes it the node handle.

Again we are going to do some processing of our own in a loop so instead of passing control to ROS with a call to __ros::spin__ we are going to call __ros::spinOnce__ to handle the transmitting and receiving of the topics. The loop will be maintained at a rate of 20Hz, this is setup by the call to __ros::rate__ and the timing is maintained by the call to __r.sleep__ within the loop.  

Our loop will continue while the call to __ros::ok__ returns true, it will return false when the node has finished shutting down e.g. when you press Ctrl-c on the keyboard.

In the loop we will call __sendTwist__ and __checkTimers__ which are described later in the article.

``` C++
int main(int argc, char **argv)
{   
    ros::init(argc, argv, "rodney");
    ros::NodeHandle n;    
    RodneyNode rodney_node(n);   
    std::string node_name = ros::this_node::getName();
	ROS_INFO("%s started", node_name.c_str());
	
	ros::Rate r(20); // 20Hz	    
    
    while(ros::ok())
    {
        rodney_node.sendTwist();
        rodney_node.checkTimers();
        
        ros::spinOnce();
        r.sleep();
    }
    
return 0;    
}
```
The constructor for our class starts by setting default values for the class parameters. For each of the parameters which are configurable using the ROS parameter server, a call is made to either __param__ or __getParam__. The difference between these two calls is that with param the default value passed to the call is used if the parameter is not available in the parameter server.

We next subscribe to the topics that the node is interested in.

* *keyboard/keydown* to obtain key presses from a keyboard. These key presses are generated from a remote PC to control the robot in manual mode
* *joy* to obtain joystick/game pad controller input, again to control the robot from a remote PC
* *missions/mission_complete* so that the node is informed when the current robot mission is completed
* *main_battery_status* this will be used later in the project to receive the state of the robots main battery
* *demand_vel* this will be used later in the project to receive autonomous velocity demands

Next in the constructor is the advertisement of the topics which this node will publish.

* */robot_face/expected_input* this topic was discussed in part 3 of these articles and is used to display a status below the robot face. We will use it to show the status of the main battery
* */missions/mission_request* this will be used to pass requested missions and jobs on to the state machine node 
* */missions/mission_cancel* this can be used to cancel the current ongoing mission
* */cmd_vel* this will be used later in the project to send velocity commands to the node responsible for driving the electric motors. The requested velocities will either be from the autonomous subsystem or as a result of keyboard/joystick requests when in manual mode

Finally the constructor sets a random generator seed and obtains the current time. The use of the random number generator and the time is discussed in the section on the __checkTimers__ method.
``` C++
// Constructor 
RodneyNode::RodneyNode(ros::NodeHandle n)
{
    nh_ = n;
    
    linear_mission_demand_ = 0.0f;
    angular_mission_demand_ = 0.0f;
    
    manual_locomotion_mode_ = false;
    linear_set_speed_ = 0.5f;
    angular_set_speed_ = 1.0f;
    
    linear_speed_index_ = 0;
    angular_speed_index_ = 1;
    manual_mode_select_ = 0;
    
    camera_x_index_ = 2;
    camera_y_index_ = 3;
    default_camera_pos_select_ = 1;
    
    max_linear_speed_ = 3;
    max_angular_speed_ = 3;
    
    dead_zone_ = 2000;
    
    ramp_for_linear_ = 5.0f;
    ramp_for_angular_ = 5.0f;
    
    voltage_level_warning_ = 9.5f; 
    
    wav_play_enabled_ = false;  
    
    // Obtain any configuration values from the parameter server. If they don't exist use the defaults above
    nh_.param("/controller/axes/linear_speed_index", linear_speed_index_, linear_speed_index_);
    nh_.param("/controller/axes/angular_speed_index", angular_speed_index_, angular_speed_index_);
    nh_.param("/controller/axes/camera_x_index", camera_x_index_, camera_x_index_);
    nh_.param("/controller/axes/camera_y_index", camera_y_index_, camera_y_index_);
    nh_.param("/controller/buttons/manual_mode_select", manual_mode_select_, manual_mode_select_);
    nh_.param("/controller/buttons/default_camera_pos_select", default_camera_pos_select_, default_camera_pos_select_);
    nh_.param("/controller/dead_zone", dead_zone_, dead_zone_);
    nh_.param("/teleop/max_linear_speed", max_linear_speed_, max_linear_speed_);
    nh_.param("/teleop/max_angular_speed", max_angular_speed_, max_angular_speed_);
    nh_.param("/motor/ramp/linear", ramp_for_linear_, ramp_for_linear_);
    nh_.param("/motor/ramp/angular", ramp_for_angular_, ramp_for_angular_);
    nh_.param("/battery/warning_level", voltage_level_warning_, voltage_level_warning_);    
    nh_.param("/sounds/enabled", wav_play_enabled_, wav_play_enabled_);
    
    // Obtain the filename and text for the wav files that can be played    
    nh_.getParam("/sounds/filenames", wav_file_names_);
    nh_.getParam("/sounds/text", wav_file_texts_);
     
    // Subscribe to receive keyboard input, joystick input, mission complete and battery state
    key_sub_ = nh_.subscribe("keyboard/keydown", 5, &RodneyNode::keyboardCallBack, this);
    joy_sub_ = nh_.subscribe("joy", 1, &RodneyNode::joystickCallback, this);
    mission_sub_ = nh_.subscribe("/missions/mission_complete", 5, &RodneyNode::completeCallBack, this);
    battery_status_sub_ = nh_.subscribe("main_battery_status", 1, &RodneyNode::batteryCallback, this);
    
    // The cmd_vel topic below is the command velocity message to the motor driver.
    // This can be created from either keyboard or game pad input when in manual mode or from the thi subscribed
    // topic when in autonomous mode.
    demmand_sub_ = nh_.subscribe("demand_vel", 5, &RodneyNode::motorDemandCallBack, this);

    // Advertise the topics we publish
    face_status_pub_ = nh_.advertise<std_msgs::String>("/robot_face/expected_input", 5);
    mission_pub_ = nh_.advertise<std_msgs::String>("/missions/mission_request", 10);
    cancel_pub_ = nh_.advertise<std_msgs::Empty>("/missions/mission_cancel", 5);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    
    // Seed the random number generator
    srand((unsigned)time(0));
    
    last_interaction_time_ = ros::Time::now();
}
```
I'll now briefly describe the functions that make up the class.

The __joystickCallback__ is called when a message is received on the *Joy* topic. The data from the joystick/game pad controller can we used to move the robot around and to move the head/camera when in manual mode.

Data from the joystick is in two arrays, one contains the current position of each axes and the other the current state of the buttons. Which axis and which button are used is configurable by setting the index value in the parameter server.

The function first reads the axes that control the angular and linear speed of the robot. These values are compared to a dead zone value which dictates how much the axes must be moved before the value is used to control the robot. The values from the controller are then converted to values that can be used for linear and velocity demands. This will mean that the maximum possible value received from the controller will result in a demand of the robots top speed. These values are stored and will be used in the __sendTwist__ method.

Next the axes used for controlling the movement of the head/camera in manual mode are read, again a dead zone is applied to the value. If the robot is in manual locomotion mode the values are sent as a "J3" job to the *rondey_mission_node*. 

Next the button values are checked. Again the index of the button used for each function can be configured. One button is used to put the robot in manual locomotion mode, which if a robot mission is currently running results in a request to cancel the mission. The second button is used as a quick way of returning the head/camera to the default position.
``` C++
void RodneyNode::joystickCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    float joystick_x_axes;
    float joystick_y_axes;
            
    // manual locomotion mode can use the joystick/game pad
    joystick_x_axes = msg->axes[angular_speed_index_];
    joystick_y_axes = msg->axes[linear_speed_index_];
        
    // Check dead zone values   
    if(abs(joystick_x_axes) < dead_zone_)
    {
        joystick_x_axes = 0;
    }
    
    if(abs(joystick_y_axes) < dead_zone_)
    {
        joystick_y_axes = 0;
    }    
    
    // Check for manual movement
    if(joystick_y_axes != 0)
    {      
        joystick_linear_speed_ = -(joystick_y_axes*(max_linear_speed_/(float)MAX_AXES_VALUE_));
        last_interaction_time_ = ros::Time::now();
    }
    else
    {
        joystick_linear_speed_ = 0;
    }
    
    if(joystick_x_axes != 0)
    {
        joystick_angular_speed_ = -(joystick_x_axes*(max_angular_speed_/(float)MAX_AXES_VALUE_));
        last_interaction_time_ = ros::Time::now();
    }
    else
    {
        joystick_angular_speed_ = 0;
    }
    
    // Now check the joystick/game pad for manual camera movement               
    joystick_x_axes = msg->axes[camera_x_index_];
    joystick_y_axes = msg->axes[camera_y_index_];
    
    // Check dead zone values   
    if(abs(joystick_x_axes) < dead_zone_)
    {
        joystick_x_axes = 0;
    }
    
    if(abs(joystick_y_axes) < dead_zone_)
    {
        joystick_y_axes = 0;
    }  
    
    if(manual_locomotion_mode_ == true)
    {
        if((joystick_x_axes != 0) || (joystick_y_axes != 0))
        {
            std_msgs::String mission_msg;   
            mission_msg.data = "J3^";
        
            if(joystick_y_axes == 0)
            {
                mission_msg.data += "-^";
            }
            else if (joystick_y_axes > 0)
            {
                mission_msg.data += "u^";
            }
            else
            {
                mission_msg.data += "d^";        
            }
        
            if(joystick_x_axes == 0)
            {
                mission_msg.data += "-";
            }
            else if (joystick_x_axes > 0)
            {
                mission_msg.data += "r";
            }
            else
            {
                mission_msg.data += "l";        
            }
        
            mission_pub_.publish(mission_msg);
            
            last_interaction_time_ = ros::Time::now();
        }
    }
    
    // Button on controller selects manual locomotion mode
    if(msg->buttons[manual_mode_select_] == 1)
    {
        if(mission_running_ == true)
        {
            // Cancel the ongoing mission
            std_msgs::Empty empty_msg;
            cancel_pub_.publish(empty_msg);                        
        }
        
        // Reset speeds to zero           
        keyboard_linear_speed_ = 0.0f; 
        keyboard_angular_speed_ = 0.0f;
        
        manual_locomotion_mode_ = true;
        
        last_interaction_time_ = ros::Time::now(); 
    }
    
    // Button on controller selects central camera position   
    if((manual_locomotion_mode_ == true) && (msg->buttons[default_camera_pos_select_] == 1))
    {            
        std_msgs::String mission_msg;
        mission_msg.data = "J3^c^-";
        mission_pub_.publish(mission_msg);
        
        last_interaction_time_ = ros::Time::now();
    }
}
```
The __keyboardCallBack__ is called when a message is received on the *keyboard/keydown* topic. The key presses can be used to move the robot around and to move the head/camera when in manual mode.

The data in the message is checked to see if it corresponds to a key that we are interested in.

The number keys are used to select robot missions. Currently we are interested in mission 2, so if the '2' key is pressed the code publishes the request on the */missions/mission_request* topic with the "M2" ID.

The 'C' key is used to request that the current mission be cancelled, this is done by sending a message on the */missions/mission_cancel* topic.

The 'D' key is used to move the camera/head back to the default position if the robot is in manual locomotion mode.

The 'M' key is used to put the robot in manual locomotion mode. If a mission is currently in progress a request to cancel the mission is also sent.

The keyboard numeric keypad is used to control movement of the robot when in manual locomotion mode. For example key '1' will result in linear velocity in the reverse direction plus angular velocity in the ant-clockwise direction. The amount of velocity is set by the current values in __linear_set_speed___ and __angular_set_speed___ variables. The speed of the robot can be increased or decreased by the use of the '+', '-', '\*' and '/' keys on the numeric keypad. The '+' key will increase the robot linear velocity by 10% whilst the '-' key will decrease the linear velocity by 10%. The '\*' increases the angular velocity by 10% and the '/' key decreases the angular velocity by 10%.

The space key will stop the robot moving.

The concept of the linear and angular velocity will be discussed when the Twist message is described. But basically the robot does not contain steerable wheels so a change in direction will be achieved by requesting different speeds and or direction of the two motors. The amount of steering required will be set by the angular velocity.

The up/down/left and right keys are used to move the head/camera when in manual mode.
``` C++
void RodneyNode::keyboardCallBack(const keyboard::Key::ConstPtr& msg)
{
    // Check for any keys we are interested in 
    // Current keys are:
    //      'Space' - Stop the robot from moving if in manual locomotion mode
    //      'Key pad 1 and Num Lock off' - Move robot forwards and counter-clockwise if in manual locomotion mode
    //      'Key pad 2 and Num Lock off' - Move robot backwards if in manual locomotion mode
    //      'Key pad 3 and Num Lock off' - Move robot backwards and clockwise if in manual locomotion mode 
    //      'Key pad 4 and Num Lock off' - Move robot counter-clockwise if in manual locomotion mode   
    //      'Key pad 6 and Num Lock off' - Move robot clockwise if in manual locomotion mode
    //      'Key pad 7 and Num Lock off' - Move robot forwards amd counter-clockwise if in manual locomotion mode    
    //      'Key pad 8 and Num Lock off' - Move robot foward if in manual locomotion mode
    //      'Key pad 9 and Num Lock off' - Move robot forwards amd clockwise if in manual locomotion mode
    //      'Up key' - Move head/camera down in manual mode
    //      'Down key' - Move head/camera up in manual mode
    //      'Right key' - Move head/camera right in manual mode
    //      'Left key' - Move head/camera left in manual mode 
    //      'Key pad +' - Increase linear speed by 10% (speed when using keyboard for teleop)
    //      'Key pad -' - Decrease linear speed by 10% (speed when using keyboard for teleop)
    //      'Key pad *' - Increase angular speed by 10% (speed when using keyboard for teleop)
    //      'Key pad /' - Decrease angular speed by 10% (speed when using keyboard for teleop)   
    //      '2' - Run mission 2    
    //      'c' or 'C' - Cancel current mission
    //      'd' or 'D' - Move head/camera to the default position in manual mode 
    //      'm' or 'M' - Set locomotion mode to manual        

    // Check for key 2 with no modifiers apart from num lock is allowed
    if((msg->code == keyboard::Key::KEY_2) && ((msg->modifiers & ~keyboard::Key::MODIFIER_NUM) == 0))
    {
        // '2', start a complete scan looking for faces (mission 2)
        std_msgs::String mission_msg;
        mission_msg.data = "M2";
        mission_pub_.publish(mission_msg);
                    
        mission_running_ = true; 
        manual_locomotion_mode_ = false;
        
        last_interaction_time_ = ros::Time::now();       
    }
    else if((msg->code == keyboard::Key::KEY_c) && ((msg->modifiers & ~RodneyNode::SHIFT_CAPS_NUM_LOCK_) == 0))
    {          
        // 'c' or 'C', cancel mission if one is running
        if(mission_running_ == true)
        {
            std_msgs::Empty empty_msg;
            cancel_pub_.publish(empty_msg);
        }
        
        last_interaction_time_ = ros::Time::now();        
    }
    else if((msg->code == keyboard::Key::KEY_d) && ((msg->modifiers & ~RodneyNode::SHIFT_CAPS_NUM_LOCK_) == 0))
    {          
        // 'd' or 'D', Move camera to default position
        if(manual_locomotion_mode_ == true)
        {            
            std_msgs::String mission_msg;
            mission_msg.data = "J3^c^-";
            mission_pub_.publish(mission_msg);
        }    
        
        last_interaction_time_ = ros::Time::now();   
    }       
    else if((msg->code == keyboard::Key::KEY_m) && ((msg->modifiers & ~RodneyNode::SHIFT_CAPS_NUM_LOCK_) == 0))
    {
        // 'm' or 'M', set locomotion mode to manual (any missions going to auto should set manual_locomotion_mode_ to false)
        // When in manual mode user can teleop Rodney with keyboard or joystick
        if(mission_running_ == true)
        {
            // Cancel the ongoing mission
            std_msgs::Empty empty_msg;
            cancel_pub_.publish(empty_msg);                        
        }
        
        // Reset speeds to zero           
        keyboard_linear_speed_ = 0.0f; 
        keyboard_angular_speed_ = 0.0f;
        
        manual_locomotion_mode_ = true;
        
        last_interaction_time_ = ros::Time::now();
    }             
    else if((msg->code == keyboard::Key::KEY_KP1) && ((msg->modifiers & keyboard::Key::MODIFIER_NUM) == 0))
    {
        // Key 1 on keypad without num lock
        // If in manual locomotion mode this is an indication to move backwards and counter-clockwise by the current set speeds
        if(manual_locomotion_mode_ == true)
        {
            keyboard_linear_speed_ = -linear_set_speed_;                        
            keyboard_angular_speed_ = -angular_set_speed_;        
        }
        
        last_interaction_time_ = ros::Time::now();
    }
    else if((msg->code == keyboard::Key::KEY_KP2) && ((msg->modifiers & keyboard::Key::MODIFIER_NUM) == 0))
    {
        // Key 2 on keypad without num lock
        // If in manual locomotion mode this is an indication to move backwards by the current linear set speed
        if(manual_locomotion_mode_ == true)
        {
            keyboard_linear_speed_ = -linear_set_speed_;        
            keyboard_angular_speed_ = 0.0f;            
        }
        
        last_interaction_time_ = ros::Time::now();
    }  
    else if((msg->code == keyboard::Key::KEY_KP3) && ((msg->modifiers & keyboard::Key::MODIFIER_NUM) == 0))
    {
        // Key 3 on keypad without num lock
        // If in manual locomotion mode this is an indication to move backwards and clockwise by the current set speeds
        if(manual_locomotion_mode_ == true)
        {
            keyboard_linear_speed_ = -linear_set_speed_;
            keyboard_angular_speed_ = angular_set_speed_;                    
        }
        
        last_interaction_time_ = ros::Time::now();
    }
    else if((msg->code == keyboard::Key::KEY_KP4) && ((msg->modifiers & keyboard::Key::MODIFIER_NUM) == 0))
    {
        // Key 4 on keypad without num lock
        // If in manual locomotion mode this is an indication to turn counter-clockwise (spin on spot) by the current angular set speed
        if(manual_locomotion_mode_ == true)
        {
            keyboard_linear_speed_ = 0.0f;
            keyboard_angular_speed_ = angular_set_speed_;                    
        }
        
        last_interaction_time_ = ros::Time::now();
    } 
    else if((msg->code == keyboard::Key::KEY_KP6) && ((msg->modifiers & keyboard::Key::MODIFIER_NUM) == 0))
    {
        // Key 6 on keypad without num lock
        // If in manual locomotion mode this is an indication to turn clockwise (spin on spot) by the current angular set speed
        if(manual_locomotion_mode_ == true)
        {
            keyboard_linear_speed_ = 0.0f;  
            keyboard_angular_speed_ = -angular_set_speed_;                  
        }
        
        last_interaction_time_ = ros::Time::now();
    }
    else if((msg->code == keyboard::Key::KEY_KP7) && ((msg->modifiers & keyboard::Key::MODIFIER_NUM) == 0))
    {
        // Key 7 on keypad without num lock
        // If in manual locomotion mode this is an indication to move forwards and counter-clockwise by the current set speeds
        if(manual_locomotion_mode_ == true)
        {
            keyboard_linear_speed_ = linear_set_speed_; 
            keyboard_angular_speed_ = angular_set_speed_;                   
        }
        
        last_interaction_time_ = ros::Time::now();
    }    
    else if((msg->code == keyboard::Key::KEY_KP8) && ((msg->modifiers & keyboard::Key::MODIFIER_NUM) == 0))
    {
        // Key 8 on keypad without num lock
        // If in manual locomotion mode this is an indication to move forward by the current linear set speed
        if(manual_locomotion_mode_ == true)
        {
            keyboard_linear_speed_ = linear_set_speed_; 
            keyboard_angular_speed_ = 0.0f;                   
        }
        
        last_interaction_time_ = ros::Time::now();
    }
    else if((msg->code == keyboard::Key::KEY_KP9) && ((msg->modifiers & keyboard::Key::MODIFIER_NUM) == 0))
    {
        // Key 9 on keypad without num lock
        // If in manual locomotion mode this is an indication to move forwards and clockwise by the current set speeds
        if(manual_locomotion_mode_ == true)
        {
            keyboard_linear_speed_ = linear_set_speed_; 
            keyboard_angular_speed_ = -angular_set_speed_;                   
        }
        
        last_interaction_time_ = ros::Time::now();
    }
    else if(msg->code == keyboard::Key::KEY_SPACE)
    {
        // Space key
        // If in manual locomotion stop the robot movment 
        if(manual_locomotion_mode_ == true)
        {
            keyboard_linear_speed_= 0.0f;     
            keyboard_angular_speed_ = 0.0f;               
        }
        
        last_interaction_time_ = ros::Time::now();
    }
    else if(msg->code == keyboard::Key::KEY_KP_PLUS)
    {
        // '+' key on num pad
        // If in manual locomotion increase linear speed by 10%
        if(manual_locomotion_mode_ == true)
        {
            linear_set_speed_ += ((10.0/100.0) * linear_set_speed_);
            ROS_INFO("Linear Speed now %f", linear_set_speed_);
        }  
        
        last_interaction_time_ = ros::Time::now();  
    }
    else if(msg->code == keyboard::Key::KEY_KP_MINUS)
    {
        // '-' key on num pad
        // If in manual locomotion decrease linear speed by 10%
        if(manual_locomotion_mode_ == true)
        {
            linear_set_speed_ -= ((10.0/100.0) * linear_set_speed_);
            ROS_INFO("Linear Speed now %f", linear_set_speed_);
        }  
        
        last_interaction_time_ = ros::Time::now();      
    }
    else if(msg->code == keyboard::Key::KEY_KP_MULTIPLY)
    {
        // '*' key on num pad
        // If in manual locomotion increase angular speed by 10%
        if(manual_locomotion_mode_ == true)
        {
            angular_set_speed_ += ((10.0/100.0) * angular_set_speed_);
            ROS_INFO("Angular Speed now %f", angular_set_speed_);
        }    
        
        last_interaction_time_ = ros::Time::now();
    }
    else if(msg->code == keyboard::Key::KEY_KP_DIVIDE)
    {
        // '/' key on num pad        
        // If in manual locomotion decrease angular speed by 10%
        if(manual_locomotion_mode_ == true)
        {
            angular_set_speed_ -= ((10.0/100.0) * angular_set_speed_);
            ROS_INFO("Angular Speed now %f", angular_set_speed_);
        }   
        
        last_interaction_time_ = ros::Time::now(); 
    }    
    else if(msg->code == keyboard::Key::KEY_UP)
    {
        // Up Key
        // This is a simple job not a mission - move the head/camera down
        if(manual_locomotion_mode_ == true)
        {            
            std_msgs::String mission_msg;
            mission_msg.data = "J3^d^-";
            mission_pub_.publish(mission_msg);
        }
        
        last_interaction_time_ = ros::Time::now();
    }
    else if(msg->code == keyboard::Key::KEY_DOWN)
    {
        // Down Key
        // This is a simple job not a mission - move the head/camera up
        if(manual_locomotion_mode_ == true)
        {
            std_msgs::String mission_msg;
            mission_msg.data = "J3^u^-";
            mission_pub_.publish(mission_msg);
        }
        
        last_interaction_time_ = ros::Time::now();
    }  
    else if(msg->code == keyboard::Key::KEY_LEFT)
    {
        // Left key
        // This is a simple job not a mission - move the head/camera left
        if(manual_locomotion_mode_ == true)
        {
            std_msgs::String mission_msg;
            mission_msg.data = "J3^-^l";
            mission_pub_.publish(mission_msg);
        }
        
        last_interaction_time_ = ros::Time::now();
    }       
    else if(msg->code == keyboard::Key::KEY_RIGHT)
    {
        // Right Key
        // This is a simple job not a mission - move the head/camera right
        if(manual_locomotion_mode_ == true)
        {
            std_msgs::String mission_msg;
            mission_msg.data = "J3^-^r";
            mission_pub_.publish(mission_msg);
        }
        
        last_interaction_time_ = ros::Time::now();
    }                             
    else
    {
        ;
    } 
}
```
The __batteryCallback__ function is called when a messaged is received on the *main_battery_status* topic. This topic is of message type [sensor_msgs/BatteryState](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/BatteryState.html "BatteryState") which contains numerous battery information. For now we are just interested in the battery voltage level.

The callback will publish a message which contains an indication of a good or bad level along with the battery voltage level. This is published on the */robot_face/expected_input* topic so will be displayed below the robot's animated face.

The level at which the battery is considered low is configurable by using the parameter server. If the voltage is below this value, as well as the warning below the animated face a request will be sent every 5 minutes requesting that the robot speaks a low battery warning. This request will be sent to the *rodney_mission_node* with an ID of "J2". The first parameter is the text to speak and the second parameter is the text that the animated face should use for its display. This includes the ":(" smiley so that the robot face looks sad.
``` C++
// Callback for main battery status
void RodneyNode::batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{ 
    // Convert float to string with two decimal places
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << msg->voltage;
    std::string voltage = ss.str();
    
    std_msgs::String status_msg;
    
    // Publish battery voltage to the robot face
    // However the '.' will be used by the face to change the expression to neutral so we will replace with ','
    replace(voltage.begin(), voltage.end(), '.', ',');
    
    if(msg->voltage > voltage_level_warning_)
    {
        status_msg.data = "Battery level OK ";
        battery_low_count_ = 0;
    }
    else
    {
        // If the battery level goes low we wait a number of messages to confirm it was not a dip as the motors started
        if(battery_low_count_ > 1)
        {
        
            status_msg.data = "Battery level LOW ";
        
            // Speak warning every 5 minutes        
            if((ros::Time::now() - last_battery_warn_).toSec() > (5.0*60.0))
            {
                last_battery_warn_ = ros::Time::now();
            
                std_msgs::String mission_msg;
                mission_msg.data = "J2^battery level low^Battery level low:(";
                mission_pub_.publish(mission_msg);
            }
        }
        else
        {
            battery_low_count_++;
        }
    }
    
    status_msg.data += voltage + "V";                                 
    face_status_pub_.publish(status_msg);
}
```
The __completeCallBack__ function is called when a messaged is received on the */missions/mission_complete* topic. An indication that the robot is no longer running a mission is set by setting __missions_running___ to false.
``` C++
void RodneyNode::completeCallBack(const std_msgs::String::ConstPtr& msg)
{
    mission_running_ = false;
    
    last_interaction_time_ = ros::Time::now();
}
```
The __motorDemandCallBack__ function is called when a message is received on the *demand_vel* topic.

The robot movements will be either manual or autonomous, this node is responsible for using either the demands created from the keyboard or joystick in manual mode, or from the autonomous subsystem. This callback simply stores the linear and angular demands from the autonomous subsystem.
``` C++
// Callback for when motor demands received in autonomous mode
void RodneyNode::motorDemandCallBack(const geometry_msgs::Twist::ConstPtr& msg)
{ 
    linear_mission_demand_ = msg->linear.x;
    angular_mission_demand_ = msg->angular.z;
}
```
The __sendTwist__ function is one of those called from main in our loop. It decides which input should be used for requesting the actual electric motor demands, either joystick, keyboard or the autonomous subsystem. The chosen demands are published in a message on the *cmd_vel* topic. Notice that a demand is always published as its normal practice for the system to keep up a constant rate of demands. If the demands are not sent then the part of the system controlling the motors can shut them down as a safety precaution. 

The message is of type [geometry_msgs/Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html "geometry_msgs/Twist") and contains two vectors, one for linear velocity (meters/second) and one for angular velocity (radians/second). Each vector gives the velocities in three dimensions, now for linear we will only use the x direction and for angular only the velocity around the z direction. This may seem like overkill but it does mean that we can make use of existing path planning and obstacle avoidance software later in the project. Publishing this topic also means that we can simulate our robot movements in [Gazebo](http://wiki.ros.org/gazebo "Gazebo"). Gazebo is a robot simulation tool which we will use later in this part of the article to test some of our code.

To ramp the velocities to the target demands the callback function make use of two helper functions __rampedTwist__ and __rampedVel__. We use these to ramp to the target velocities in order to stop skidding and shuddering which may occur if we attempted to move the robot in one big step change in velocity. The code in these two helper functions is based on Python code from the O'Reilly book "Programming Robots with ROS".
``` C++
void RodneyNode::sendTwist(void)
{
    geometry_msgs::Twist target_twist;
    
    // If in manual locomotion mode use keyboard or joystick data
    if(manual_locomotion_mode_ == true)
    {
        // Publish message based on keyboard or joystick speeds
        if((keyboard_linear_speed_ == 0) && (keyboard_angular_speed_ == 0))
        {
            // Use joystick values
            target_twist.linear.x = joystick_linear_speed_;
            target_twist.angular.z = joystick_angular_speed_;            
        }
        else
        {
            // use keyboard values
            target_twist.linear.x = keyboard_linear_speed_;
            target_twist.angular.z = keyboard_angular_speed_;                   
        }
    }
    else
    {
        // Use mission demands (autonomous)
        target_twist.linear.x = linear_mission_demand_;
        target_twist.angular.z = angular_mission_demand_;
    }
    
    ros::Time time_now = ros::Time::now();
        
    // Ramp towards are required twist velocities
    last_twist_ = rampedTwist(last_twist_, target_twist, last_twist_send_time_, time_now);
        
    last_twist_send_time_ = time_now;
        
    // Publish the Twist message
    twist_pub_.publish(last_twist_);
}
//---------------------------------------------------------------------------

geometry_msgs::Twist RodneyNode::rampedTwist(geometry_msgs::Twist prev, geometry_msgs::Twist target,
                                             ros::Time time_prev, ros::Time time_now)
{
    // Ramp the angular and linear values towards the tartget values
    geometry_msgs::Twist retVal;
    
    retVal.angular.z = rampedVel(prev.angular.z, target.angular.z, time_prev, time_now, ramp_for_angular_);
    retVal.linear.x = rampedVel(prev.linear.x, target.linear.x, time_prev, time_now, ramp_for_linear_);
    
    return retVal;
}
//---------------------------------------------------------------------------

float RodneyNode::rampedVel(float velocity_prev, float velocity_target, ros::Time time_prev, ros::Time time_now,
                            float ramp_rate)
{
    // Either move towards the velocity target or if difference is small jump to it
    float retVal;    
    float sign;
    float step = ramp_rate * (time_now - time_prev).toSec();
    
    if(velocity_target > velocity_prev)
    {
        sign = 1.0f;
    }
    else
    {
        sign = -1.0f;
    }
    
    float error = std::abs(velocity_target - velocity_prev);
    
    if(error < step)
    {
        // Can get to target in this within this time step
        retVal = velocity_target;
    }
    else
    {
        // Move towards our target
        retVal = velocity_prev + (sign * step);
    }        
    
    return retVal;
}
```
The last function __checkTimers__ is the other function called from main in our loop. Now the functionality here serves two purposes. The first is if the robot is inactive, that is that it has not been manually controlled or it finished the last mission more than 15 minutes ago, it will play a pre-existing wav file to remind you that it is still powered up. This functionality can be disabled by use of the */sounds/enabled* parameter in the parameter server.

Oh and the second purpose of the functionality I'm afraid is an indication of my sense of humour, all my pre-existing wav files are recordings of Sci-Fi robots. I figured if a robot got bored it may amuse its self by doing robot impressions! "Danger Will Robinson, danger". Anyway if you don't like this idea you can disable the functionality or just play something else to show it is still powered up and inactive.

There are a number of wav file names and text sentences to go with the wav files loaded into the parameter server. When it is time to play a wav file a random number is generated to select which wav file to play. The request is then sent using the ID "J1".
``` C++
void RodneyNode::checkTimers(void)
{
    /* Check time since last interaction */
    if((wav_play_enabled_ == true) && (mission_running_ == false) && ((ros::Time::now() - last_interaction_time_).toSec() > (15.0*60.0)))
    {
        last_interaction_time_ = ros::Time::now();
        
        // Use a random number to pick the wav file
        int random = (rand()%wav_file_names_.size())+1;                
         
        // This is a simple job not a mission
        std_msgs::String mission_msg;
        std::string path = ros::package::getPath("rodney");
        mission_msg.data = "J1^" + path + "/sounds/" + wav_file_names_[std::to_string(random)] + 
                           "^" + wav_file_texts_[std::to_string(random)];        
        mission_pub_.publish(mission_msg);         
    }    
}
```
### Changes to head_control node
Earlier in these articles we wrote the head_control package to synchronise the head movement and facial recognition functionality. In this article we want to be able to control the head manually as well. We therefore need to make some modifications to the *head_control_node.cp* file.

In the __HeadControlNode__ constructor add code to subscribe to the */head_control_node/manual* topic.
``` C++
// Subscribe to topic for manual head movement command
manual_sub_ = nh_.subscribe("/head_control_node/manual", 5,&HeadControlNode::manualMovementCallback, this);
```
Also add the following line to the end of the constructor.
``` C++
target_pan_tilt_ = current_pan_tilt_;
```
Add code for the __manualMovementCallback__ function which is called when a message is received on the */head_control_node/manual* topic. This function processes the request to move the head/camera up, down, left, right or to the default position.
``` C++
// This callback is used to process a command to manually move the head/camera
void HeadControlNode::manualMovementCallback(const std_msgs::String& msg)
{   
    if(msg.data.find('u') != std::string::npos)
    {
        target_pan_tilt_.tilt = current_pan_tilt_.tilt + tilt_view_step_;        
		
	    if(target_pan_tilt_.tilt > tilt_max_)
	    {          
	        // Moved out of range, put back on max                   
	        target_pan_tilt_.tilt = tilt_max_;
        }
    }
    
    if(msg.data.find('d') != std::string::npos)
    {
        target_pan_tilt_.tilt = current_pan_tilt_.tilt - tilt_view_step_;
        
        if(target_pan_tilt_.tilt < tilt_min_)
	    {          
	        // Moved out of range, put back on min                   
	        target_pan_tilt_.tilt = tilt_min_;
        }
    }
    
    if(msg.data.find('l') != std::string::npos)
    {
        target_pan_tilt_.pan = current_pan_tilt_.pan + pan_view_step_;
        
        if(target_pan_tilt_.pan > pan_max_)
	    {          
	        // Moved out of range, put back on max                   
	        target_pan_tilt_.pan = pan_max_;
        }
    }
    
    if(msg.data.find('r') != std::string::npos)
    {
        target_pan_tilt_.pan = current_pan_tilt_.pan - pan_view_step_;

        if(target_pan_tilt_.pan < pan_min_)
	    {          
	        // Moved out of range, put back on min                   
	        target_pan_tilt_.pan = pan_min_;
        }
    }
    
    if(msg.data.find('c') != std::string::npos)
    {
        // Move to default central position
        target_pan_tilt_ = default_position_;
    }
    
    // Assume if message received we will be moving the head/camera
    move_head_ = true;
    process_when_moved_ = nothing;        
}
```
### Joystick node
Now throughout this article we have added functionality for the robot to be moved manually by using a joystick/game pad controller. There is a joystick node available on the ROS Wiki website called [joy](http://wiki.ros.org/joy "joy").

However I tried this package on two different Linux PCs and found that I kept getting segmentation faults. Instead of doing any deep investigation to see what the problem was I wrote my own simple joystick node. It's simpler than the one on the ROS website as I don't bother with worrying about sticky buttons etc.

I would suggest that you try and use the package from the ROS website but if you have similar problems then you can use my ROS package which is available in the *joystick folder*. I have used it successfully with a Microsoft Xbox 360 Wired Controller and the *joystick_node.cpp* file is reproduced below.
``` C++
// Joystick Node. Takes input from a joystick/game pad and outputs current state in a sensor_msgs/joy topic.
// See https://www.kernel.org/doc/Documentation/input/joystick-api.txt
#include <joystick/joystick_node.h>

#include <fcntl.h>
#include <stdio.h>
#include <linux/joystick.h>

// Constructor 
Joystick::Joystick(ros::NodeHandle n, std::string device)
{
    nh_ = n;
    
    // Advertise the topics we publish
    joy_status_pub_ = nh_.advertise<sensor_msgs::Joy>("joy", 5);
        
    js_ = open(device.c_str(), O_RDONLY);
    
    if (js_ == -1)
    {
        ROS_ERROR("Problem opening joystick device");
    }
    else
    {
        int buttons = getButtonCount();
        int axes = getAxisCount();

        joyMsgs_.buttons.resize(buttons);
        joyMsgs_.axes.resize(axes);
        
        ROS_INFO("Joystick number of buttons %d, number of axis %d", buttons, axes);
    }
}

// Process the joystick input
void Joystick::process(void)
{
    js_event event;
        
    FD_ZERO(&set_);
    FD_SET(js_, &set_);
    
    tv_.tv_sec = 0;
    tv_.tv_usec = 250000;
    
    int selectResult = select(js_+1, &set_, NULL, NULL, &tv_);
    
    if(selectResult == -1)
    {
        ROS_ERROR("Error with select joystick call"); // Error
    }
    else if (selectResult)
    {
        // Data available
        if(read(js_, &event, sizeof(js_event)) == -1 && errno != EAGAIN)
        {
            // Joystick probably closed
            ;
        }
        else
        {
            switch (event.type)
            {
                case JS_EVENT_BUTTON:
                case JS_EVENT_BUTTON | JS_EVENT_INIT:
                    // Set the button value                    
                    joyMsgs_.buttons[event.number] = (event.value ? 1 : 0);
                    
                    time_last_msg_ = ros::Time::now();
                    
                    joyMsgs_.header.stamp = time_last_msg_;
                    
                    // We publish a button press right away so they are not missied
                    joy_status_pub_.publish(joyMsgs_);
                    break;
                    
                case JS_EVENT_AXIS:
                case JS_EVENT_AXIS | JS_EVENT_INIT:
                    // Set the axis value 
                    joyMsgs_.axes[event.number] = event.value;
                    
                    // Only publish if time since last regular message as expired
                    if((ros::Time::now() - time_last_msg_).toSec() > 0.1f)                    
                    {
                        time_last_msg_ = ros::Time::now();
                    
                        joyMsgs_.header.stamp = time_last_msg_;
                        
                        // Time to publish
                        joy_status_pub_.publish(joyMsgs_);                 
                    }

                default:                    
                    break;            
            }
        }  
    }
    else
    {
        // No data available, select time expired.
        // Publish message to keep anything alive that needs it
        
        time_last_msg_ = ros::Time::now();
        
        joyMsgs_.header.stamp = time_last_msg_;
        
        // Publish the message
        joy_status_pub_.publish(joyMsgs_);
    }
}

// Returns the number of buttons on the controller or 0 if there is an error.
int Joystick::getButtonCount(void)
{
    int buttons;
    
    if (ioctl(js_, JSIOCGBUTTONS, &buttons) == -1)
    {
        buttons = 0;
    }

    return buttons;
}

// Returns the number of axes on the controller or 0 if there is an error.
int Joystick::getAxisCount(void)
{
    int axes;

    if (ioctl(js_, JSIOCGAXES, &axes) == -1)
    {
        axes = 0;
    }

    return axes;
}

int main(int argc, char **argv)
{
    std::string device;
    
    ros::init(argc, argv, "joystick_node");

    // ros::init() parses argc and argv looking for the := operator.
    // It then modifies the argc and argv leaving any unrecognized command-line parameters for our code to parse.
    // Use command line parameter to set the device name of the joystick or use a default.        
    if (argc > 1)
    {
        device = argv[1];
    }
    else
    {
        device = "/dev/input/js0";
    }
    
    ros::NodeHandle n;    
    Joystick joystick_node(n, device);   
    std::string node_name = ros::this_node::getName();
	ROS_INFO("%s started", node_name.c_str());	
	
	// We are not going to use ros::Rate here, the class will use select and 
	// return when it's time to spin and send any messages on the topic
    
    while(ros::ok())
    {
        // Check the joystick for an input and process the data
        joystick_node.process();
        
        ros::spinOnce();
    }
    
    return 0;    
}
```
## Using the code
To test the code we have developed so far I'm going to run some tests on the actual robot hardware but we can also run some tests on the Gazebo robot simulator tool running on a Linux PC. In the folder *rodney/urdf* there is a file called *rodney.urdf* which models the Rodney Robot. How to write a URDF (Unified Robot Description Format) model would require many articles itself but as always there is information on the ROS Wiki website about [URDF](http://wiki.ros.org/urdf "URDF"). My model is nowhere near perfect and needs some work but we can use it here to test the robot locomotion. All the files to do this are included in the *rodney* folder and the *rodney_sim_control* folder. 

The package versions used in this test were:
* [joystick V0.1.0](https://github.com/phopley/joystick/releases/tag/V0.1.0)
* [rodney_sim_control V0.0.2](https://github.com/phopley/Robotics-test-code/releases/tag/V0.0.2)
* [face_recognition V0.1.2](https://github.com/phopley/face_recognition/releases/tag/V0.1.2) 
* [face_recognition_msgs V0.1.1](https://github.com/phopley/face_recognition_msgs/releases/tag/V0.1.1)
* [head_control V0.1.2](https://github.com/phopley/head_control/releases/tag/V0.1.2)
* [pan_tilt V0.1.5](https://github.com/phopley/pan_tilt/releases/tag/V0.1.5)
* [rondey V0.1.1](https://github.com/phopley/rodney/releases/tag/V0.1.1)
* [rodney_missions V0.1.0](https://github.com/phopley/rodney_missions/releases/tag/V0.1.0)
* [servo_msgstag/V0.1.2](https://github.com/phopley/servo_msgs/releases/tag/V0.1.2)
* [speech V0.1.1](https://github.com/phopley/speech/releases/tag/V0.1.1)

### Building the ROS packages on the workstation
On the workstation as well as running the simulation we also want to run the keyboard and joystick nodes so that we can control the actual robot hardware remotely.

Create a workspace with the following commands:
```
$ mkdir -p ~/test_ws/src 
$ cd ~/test_ws/ 
$ catkin_make
```
Copy the packages *rodney*, *joystick*, *rodney_sim_control* and *ros-keyboard* (from https://github.com/lrse/ros-keyboard) into the ~/test_ws/src folder and then build the code with the following commands:
```
$ cd ~/test_ws/ 
$ catkin_make
```
Check that the build completes without any errors.
### Running the simulation
In the *rodney_sim_control* package there is a launch file that will load the robot model into the parameter server, launch Gazebo and spawn a simulation of the robot. Launch this file with the following commands:
```
$ cd ~/test_ws/
$ source devel/setup.bash
$ roslaunch rodney_sim_control rodney_sim_control.launch
```
After a short time you should see the model of Rodney in an empty world. The simulation is currently paused.

<img src="https://github.com/phopley/rodney-project/blob/master/docs/images/Opti-gazebo01_2wd.png" title="Gazebo">

In a new terminal load the rodney config file and run the rodney node with the following commands:
```
$ cd ~/test_ws/ 
$ source devel/setup.bash 
$ rosparam load src/rodney/config/config.yaml
$ rosrun rodney rodney_node
```
An info message should be seen reported that the node is running.

The first test is going to test that a message on the *demand_vel* topic, as if from the autonomous subsystem, will control the robot's movements.

In Gazebo click the play button, bottom left of the main screen, to start the simulation. In a new terminal type the following to send a message on the *demand_vel* topic.
```
$ rostopic pub -1 /demand_vel  geometry_msgs/Twist '{linear: {x: 0.5}}'
```
The simulated robot will move forward at a velocity of 0.5 metres/second. Reverse the direction with the following command:
```
$ rostopic pub -1 /demand_vel geometry_msgs/Twist '{linear: {x: -0.5}}'
```
You can stop the robot movement with the following command:
```
$ rostopic pub -1 /demand_vel geometry_msgs/Twist '{linear: {x: 0.0}}'
```
Next make the simulated robot turn on the spot with the following command:
```
$ rostopic pub -1 /demand_vel geometry_msgs/Twist '{angular: {z: 1.0}}'
```
Repeating the command with a negative value will cause the robot to rotate clockwise and then stop the movement with a value of zero.

Next we will test the movement with the keyboard functionality.
```
$ cd ~/test_ws/ 
$ source devel/setup.bash
$ rosrun keyboard keyboard
```
A small window whose title is "ROS keyboard input" should be running. Make sure this window has the focus and then press 'm' key to put the robot in manual locomotion mode.

Ensure "num lock" is not selected.

You can now use the keyboards numeric keypad to drive the robot around the simulated world. The following keys can be used to move the robot.
* Key pad 8 - forward
* Key pad 2 - reverse
* Key pad 4 - rotate anti-clockwise
* Key pad 6 - rotate clockwise
* Key pad 7 - forward and left
* Key pad 9 - forward and right
* Key pad 1 - reverse and left
* Key pad 3 - reverse and right
* Key pad + increase the linear velocity
* Key pad - decrease the linear velocity
* Key pad * increase the angular velocity
* Key pad / decrease the angular velocity

The space bar will stop the robot

Next we can test the movement with the joystick controller. Ensure the robot is stationary. In a new terminal issue the following commands.
```
$ cd ~/test_ws/
$ source devel/setup.bash
$ rosrun joystick joystick_node
```
A message showing the node has started should be displayed. With the configuration given in an unchanged *rodney/config/config.yaml* file and a wired Xbox 360 controller, you can control the simulated robot with the controls shown in the image below.

<img src="https://github.com/phopley/rodney-project/blob/master/docs/images/Opti-controller1.png" title="Controller">

From the Gazebo menu other objects can be inserted into the world. The video below shows the movement test running using Gazebo. Note that in the video Rodney is a 4 wheel drive robot, I have since updated the model and the actual robot has 2 wheel drive and casters. This will all be explained in the next article when we move the real robot hardware.

[![YouTube](http://img.youtube.com/vi/Q23LoCGYPsI/0.jpg)](http://www.youtube.com/watch?v=Q23LoCGYPsI "Rodney robot movement simulation")

### Building the ROS packages on the Pi (Robot hardware)
If not already done create a catkin workspace on the Raspberry Pi and initialise it with the following commands:
```
$ mkdir -p ~/rodney_ws/src
$ cd ~/rodney_ws/
$ catkin_make
```
Copy the packages *face_recognition*, *face_recognition_msgs*, *head_control*, *pan_tilt*, *rondey*, *rodney_missions*, *servo_msgs*, *speech* and *ros-keyboard* (from https://github.com/lrse/ros-keyboard) into the *~/rodney_ws/src* folder.

Unless you want to connect the joystick controller directly to the robot you don't need to build the joystick package on the robot hardware. You do however need to build the keyboard package as it includes a message unique to that package. I'm going to using the Linux PC connected to the same network as the robot to control it remotely.

Build the code with the following commands:
```
$ cd ~/rodney_ws/ 
$ catkin_make
```
Check that the build completes without any errors.

You will also need to compile and download the Arduino code to the Nano to control the servos.

If not already done you will need to train the face recognition software, see [Design Goal 2 Part 1](https://github.com/phopley/rodney-project/blob/master/docs/005%20Mission%201%20Design%20Goal%201%20Part%202.md "Design Goal 2 Part 1")
### Running the code on the robot
Now we are ready to run our code. With the Arduino connected to a USB port use the launch file to start the nodes with the following commands. If no master node is running in a system the launch command will also launch the master node, roscore:
```
$ cd ~/rodney_ws/
$ source devel/setup.bash
$ roslaunch rodney rodney.launch
```
On the workstation run the following commands to start the keyboard node:
```
$ cd ~/test_ws 
$ source devel/setup.bash 
$ export ROS_MASTER_URI=http://ubiquityrobot:11311 
$ rosrun keyboard keyboard
```
A small window whose title is "ROS keyboard input" should be running.

The first test we will run on the robot hardware is "Mission 2". Make sure keyboard window has the focus and then press '2' key to start the mission.

The robot should start moving the head/camera scanning the room for known faces. Once it has completed the scan within its head movement range it will either report that no one was recognised or a greeting to those it did recognise.

The next test will check the ability to move the head/camera in manual mode using the keyboard. Make sure keyboard window has the focus and then press 'm' to put the system in manual mode. Used the cursor keys to move the head/camera. Press the 'd' key to return the head/camera to the default position.

The next test will check the ability to move the head/camera in manual mode using the joystick controller. In a new terminal on the workstation type the following commands.
```
$ cd ~/test_ws 
$ source devel/setup.bash 
$ export ROS_MASTER_URI=http://ubiquityrobot:11311 
$ rosrun joystick joystick_node
```
A message showing the node has started should be displayed. With the configuration given in an unchanged *rodney/config/config.yaml* file and a wired Xbox 360 controller you can control the robot head/camera movement with the controls shown in the image below.

<img src="https://github.com/phopley/rodney-project/blob/master/docs/images/Opti-controller2.png" title="Controller">

[![YouTube](http://img.youtube.com/vi/2KPQKTOI6T4/0.jpg)](http://www.youtube.com/watch?v=2KPQKTOI6T4 "Rodney running mission 2 and manual head control")

For the next test we will test the status indication. In a terminal at the workstation type the following commands:
```
$ cd ~/test_ws 
$ source devel/setup.bash 
$ export ROS_MASTER_URI=http://ubiquityrobot:11311 
$ rostopic pub -1 main_battery_status sensor_msgs/BatteryState '{voltage: 12}'
```
The status below the robot face should read "Battery level OK 12,00V".

In the terminal issue the following command:
```
$ rostopic pub -1 main_battery_status sensor_msgs/BatteryState '{voltage: 9.4}'
```
The status below the robot face should read "9,40V".

In the terminal issue the following command __twice__:
```
$ rostopic pub -1 main_battery_status sensor_msgs/BatteryState '{voltage: 9.4}'
```
The status below the robot face should read "Battery level low 9,40V", the robot should speak a low warning and the facial expression should be sad.

Send the message again within 5 minutes of the last message. The warning should not be spoken.

Wait for 5 minutes and send the message again. This time the spoken warning should be repeated.

The next test will check the functionality for wav file playback. Wait for 15 minutes without issuing any commands from the keyboard and joystick. After the 15 minutes the robot should play a random wav file and animate the mouth along with the wav file.

To aid debugging here is an output from rqt_graph of the current system.

<img src="https://github.com/phopley/rodney-project/blob/master/docs/images/rosgraph.png" title="Graph">
