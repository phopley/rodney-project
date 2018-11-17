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
