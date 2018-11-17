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
