# Mission 1, Design Goal 1 continued
## Controlling the head
We now have a node to carry out the facial recognition operation on an image from the camera and from part 1 of this article we have the pan/tilt functionality to move the head/camera. The next step is to put the two together so that the robot can scan a room within its head movement range looking for faces it recognises.

Our ROS package for the node is called *head_control* and is available in the [head_control repository](https://github.com/phopley/head_control). The package contains all the usual ROS files and folders.

The *config* folder contains a *config.yaml* file which can be used to override some of the default configuration values. You can configure:
* the maximum and minimum range of movement in both pan and tilt directions
* the angle that the pan and tilt device should move between image grabs
* the step range the pan and tilt servos should move per request. This is to stop the servo moving a requested angle in one jump and causing the head to shudder
* the pan/tilt position that the head should return to once a complete scan of the area is complete

The *include/head_contro*l and *src* folders contain the C++ code for the package. For this package we have one C++ class, __HeadControlNode__ and a __main__ routine contained within the *head_control_node.cpp* file.

The __main__ routine informs ROS of our node, creates an instance of the class for the node and passes it the node handle and node name. For the first time in the project we are not going to hand total control of the node to ROS. In all our other nodes so far we have handed off control to ROS with a call to __ros::spin__ in C++ or __rospy.spin__ in our Python code. These calls hand control to ROS and our code only gets to run when ROS calls one of our callback functions when a message is received. In this node I want to retain control as we want to move the servos in small incremental steps to a target position. If we allow the servos to move a large angle in one go the head comes to a shuddering stop when the target position is reached. The way that we retain control is to call __ros::spinOnce__. In this function ROS will publish any topics we have requested and process any incoming topics by calling our callbacks but will then return control to __main__.

Before we enter the while loop we create an instance of __ros::Rate__ and pass it the timing we would like to maintain, in our case I'm setting the rate to 10Hz. When inside the loop we call r.sleep, this Rate instance will attempt to keep the loop at 10Hz by accounting for the time used to complete the work in the loop.

Our loop will continue while the call to __ros::ok__ returns true, it will return false when the node has finished shutting down e.g. when you press Ctrl-c on the keyboard.

In the loop we will call __moveServo__ which is described later in the article.
``` C++
int main(int argc, char **argv),
{
    ros::init(argc, argv, "head_control_node");
    ros::NodeHandle n;
    std::string node_name = ros::this_node::getName();
    HeadControlNode head_control(n, node_name);	
    ROS_INFO("%s started", node_name.c_str());
    
    // We need control of the node so that we can step the servos to the target 
    // position in small steps to stop the head shuddering if it was to move in one big step
    
    ros::Rate r(10); // 10Hz
    
    while(ros::ok())
    {
        // See if the servos need moving
        head_control.moveServo();
        
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
```
The constructor for our class contains a little bit more than our previous node constructors, this is because we are going to make use of the *scan_for_faces* action we defined in the *face_recognition_msgs* package. In fact our node will include the action server for this action.

The call to the constructor also creates an instance of Server, __as___ and a string, __action_name___, to hold the name of the action. We then register our callback function, __scanForFacesCallback__, with the action server. This callback will be called when the action server receives the action goal, this in affect kicks off the action.

We then start the server running with the __as_.start__ call.

The rest of the constructor is similar to our previous nodes. It subscribes to the topic */face_recognition_node/result* in order to receive the result from the face recognition node, it advertises that it will publish the topic *face_recognition_node/start* which is used to instruct the face recognition node to start a face recognition operation. It also advertises that it will publish the topic *pan_tilt_node/head_position* which will be used to pass the required pan/tilt position to the pan/tilt node.

We then set some configuration defaults and read any overrides from the parameter server should they be available.

The action server we created sends feedback during the action which includes the percentage complete, in order to determine this we calculate the number of positions that will be used to scan the range of movement of the head and assign it to __total_indv_scans___.

Finally we publish a message to move the head to a known starting point. We can't move to this position in small steps as we don't know our starting position after power up.

``` C++
// Constructor 
HeadControlNode::HeadControlNode(ros::NodeHandle n, std::string name) : as_(n, name, false), action_name_(name)
{	
    nh_ = n;
	
    as_.registerGoalCallback(boost::bind(&HeadControlNode::scanForFacesCallback, this));
    as_.start();
  
    individual_scan_finished_sub_ =
        nh_.subscribe("/face_recognition_node/result", 1, &HeadControlNode::individualScanFinishedCallback, this);          
		
    // Topic to instruct the scan of the current image
    start_individual_scan_pub_ = nh_.advertise<std_msgs::Empty>("face_recognition_node/start", 1);
	
    // Topic to move head
    move_head_pub_ = nh_.advertise<servo_msgs::pan_tilt>("pan_tilt_node/head_position", 10);	
	
    pan_min_ = 0;       // Smallest servo angle for pan
    pan_max_ = 180;     // Maximum servo angle for pan
    tilt_min_ = 0;      // Smallest servo angle for tilt
    tilt_max_ = 180;    // Maximum servo angle for tilt  
    pan_step_ = 10;     // Maximum step movment for pan servo 
    tilt_step_ = 10;    // Maximum step movment for tilt servo 
    pan_view_step_ = 10;     // Angle to increase/decrease pan position by when scanning 
    tilt_view_step_ = 10;    // Angle to increase tilt position by when scanning 
	
    // Obtain any configuration values from the parameter server. If they don't exist use the defaults above
    nh_.param("/servo/index0/pan_min", pan_min_, pan_min_);
    nh_.param("/servo/index0/pan_max", pan_max_, pan_max_);
    nh_.param("/servo/index0/tilt_min", tilt_min_, tilt_min_);
    nh_.param("/servo/index0/tilt_max", tilt_max_, tilt_max_);
    nh_.param("/head/view_step/pan", pan_view_step_, pan_view_step_);
    nh_.param("/head/view_step/tilt", tilt_view_step_, tilt_view_step_);
    nh_.param("/head/max_step/pan", pan_step_, pan_step_);
    nh_.param("/head/max_step/tilt", tilt_step_, tilt_step_);
        
    int pan = 90;      // Pan position to return to once a task is complete
    int tilt = 45;     // Tilt position to return to once a task is complete
    nh_.param("/head/position/pan", pan, pan);    
    nh_.param("/head/position/tilt", tilt, tilt);
    default_position_.pan = (int)pan;
    default_position_.tilt = (int)tilt;
    
    // Calculate the total number of individual scans for % complete
    total_indv_scans_ = (((pan_max_ - pan_min_) / pan_view_step_) + 1) * (((tilt_max_ - tilt_min_) / tilt_view_step_) + 1) - 1;
	
    // This start position may be so the user can access the on screen keyboard. 
    // We will often return to this position when a task is completed	
    current_pan_tilt_ = default_position_;
    // We don't know where the servo starts from so just jump to the required position    
    // Publish a start position to get the head in a known position.
    move_head_pub_.publish(current_pan_tilt_);
    
    move_head_ = false;
    movement_complete_ = false;
}
```
I'll now briefly describe the functions that make up the class.

The __scanForFacesCallback__ is called by the ROS when the action server receives a goal message. This message can contain data but in our case does not and is used to start a scan of the full range of movement of the head checking for faces that are recognised at each position.

The function calls the action server to inform it that the goal has been accepted, it clears a list that will be used to store the ids and names of people recognised and sets __scans_complete___ to zero, this will be used in the percentage complete calculation. A starting position of the pan/tilt device is set and the variables used to control the smooth movement of the head are set and __process_when_moved___ is set to indicate that an individual scan for faces should be conducted when the head reaches its target position.
``` C++
// This callback is used to kick off the action to scan all positions
void HeadControlNode::scanForFacesCallback()
{
    face_recognition_msgs::scan_for_facesGoal::ConstPtr goal;
	
    goal = as_.acceptNewGoal();
    ROS_DEBUG("HeadControlNode: Got action goal");

    // Clear the list of those seen
    seen_list_.clear();
    
    scans_complete_ = 0;

    // Here we set up to move the head to the start position and then to request an 
    // individual scan when servo reaches that position
    
    // Set the target position to pan min and tilt min. 
    target_pan_tilt_.pan = pan_min_;
    target_pan_tilt_.tilt = tilt_min_;
	
    // Set the variable that says which direction the pan is going. Start by incrementing
    increase_pan_ = true;
    
    // State what we want to occur when the servo reaches the target position
    process_when_moved_ = requestScan;
	
    // Indicate that the servos should be moved
    move_head_ = true;
}
```
The __individualScanFinishedCallback__ is called when the message with the result of an individual scan is received from the face recognition node.

An action can be preempted in order to cancel it during an operation. Our code checks to see if the action has been preempted and if so acknowledges the fact and request that the head be moved back to the default position.

If the action was not preempted it checks to see if all the range has been scanned or calculates the next position of the head.

We then add any people detected on the last individual scan to our list of seen individuals first ensuing that we have not already seen them before.

If all positions have been scanned the people seen are added to the result and sent to the action server with an indication that the action was completed successfully by the call to __as_.setSucceeded__. A request is then set up to move the head back to the default position.

If all the positions are not compete then the percentage complete is calculated and along with the result of the last individual scan is passed to the function __publishFeedback__. We then request that the head is moved and that another individual scan is conducted when the movement of the head is complete.
``` C++
// This callback is used when the face recognition node sends the result back for an individual scan		
void HeadControlNode::individualScanFinishedCallback(const face_recognition_msgs::face_recognition& msg)
{
    bool all_areas_scanned = false;
    float percentage_complete;    

    scans_complete_++;
	
    if(as_.isPreemptRequested() || !ros::ok())
    {
        // Here we set up to move the head to the default position and then
        // to request preempted when servo reaches that position
        
        ROS_DEBUG("HeadControlNode: %s Preempted", action_name_.c_str());
        
        as_.setPreempted();
        
        // preempted, move to the neutral position
        target_pan_tilt_ = default_position_;
                    
        // State what we want to occur when the servo reaches the target position
        process_when_moved_ = nothing;
	
        // Indicate that the servos should be moved
        move_head_ = true;
    }
    else
    {
        // Scan of individual image complete
        // Calculate the next position of the head/camera
        if(increase_pan_ == true)
        {           
            if(target_pan_tilt_.pan == pan_max_)
            {
                // Last scan was at the edge, move tilt up and then pan the other way
                increase_pan_ = false;
                                	            
	            target_pan_tilt_.tilt += tilt_view_step_;
			
	            if(target_pan_tilt_.tilt > tilt_max_)
	            {
	                all_areas_scanned = true;                			
	            }
	        }                                
            else
            {            
	            target_pan_tilt_.pan = current_pan_tilt_.pan + pan_view_step_;
		
	            if(target_pan_tilt_.pan > pan_max_)
	            {          
	                // Moved out of range, put back on max                   
	                target_pan_tilt_.pan = pan_max_;
                }
	        }
        }
        else
        {
            if(target_pan_tilt_.pan == pan_min_)        
            {
                // Last scan was at the edge, move tilt up and then pan the other way
                increase_pan_ = true;
                
                target_pan_tilt_.tilt += tilt_view_step_;
			
	            if(target_pan_tilt_.tilt > tilt_max_)
	            {
	                all_areas_scanned = true;                			
	            }
	        }
	        else
	        {	                     
                target_pan_tilt_.pan = current_pan_tilt_.pan - pan_view_step_;
		
	            if(target_pan_tilt_.pan < pan_min_)
	            {
	                // Moved out of range, put back on min
        	        target_pan_tilt_.pan = pan_min_;
        	    }
	        }
        }
		   
	    // Add any faces seen to the stored list
	    if(msg.ids_detected.size() > 0)
        {  
            for(unsigned long x = 0; x < msg.ids_detected.size(); x++)
            {
                FaceSeen face_detected;
                face_detected.id = msg.ids_detected[x];
                face_detected.name = msg.names_detected[x];             

                if(haveWeSeenThisPerson(face_detected) == false)
                {                                    
                    ROS_DEBUG("HeadControlNode: I have seen %s", msg.names_detected[x].c_str());
                }
            }          
        }
	    
        if(all_areas_scanned == true)
        {              		    
		    // Note here in the result we send a list of all those people seen
		    
	        // Send action result, the seen list
	        face_recognition_msgs::scan_for_facesResult result;	        
	          	        
	        // Iterate through the faces seen adding the the result
	        for (std::list<FaceSeen>::const_iterator iterator = seen_list_.begin(), end = seen_list_.end(); iterator != end; ++iterator)
	        {	            	           	            
	            result.detected.ids_detected.push_back(iterator->id);
	            result.detected.names_detected.push_back(iterator->name);	            	       
	        }
		
            as_.setSucceeded(result);
            
            // Move the head/camera back to default position             
	        target_pan_tilt_ = default_position_;
	            
	        // State what we want to occur when the servo reaches the target position
            process_when_moved_ = nothing;
	
            // Indicate that the servos should be moved
            move_head_ = true;
        }
        else
        {        
	        // Calculate percentage complete
        	percentage_complete = ((float)scans_complete_ / (float)total_indv_scans_) * 100.0;
				    
		    // Note that here in the feedback we only send those just seen, even if seen before
		        
        	// Send feedback which includes the result of the last individual scan
	        publishFeedback(percentage_complete, msg); 	        
		
            // target pan/tilt position set above            	        
	            
	        // State what we want to occur when the servo reaches the target position (i.e. next scan)
            process_when_moved_ = requestScan;
	
            // Indicate that the servos should be moved
            move_head_ = true;
        }	
    }
}
```
The __publishFeedback__ function is used to post feedback to the action server. Our feedback includes a percentage complete figure and the result of the last individual scan.
``` C++
// This will be called after we receive the result of each individual scan except for the final scan 
void HeadControlNode::publishFeedback(float progress, face_recognition_msgs::face_recognition msg)
{
    face_recognition_msgs::scan_for_facesFeedback feedback;
    feedback.progress = progress;
    feedback.detected = msg;    
    as_.publishFeedback(feedback);
}
```
The __haveWeSeenThisPerson__ function is used to check to see if any people recognised in the last individual scan have been seen already. If not they are added to the list of seen people.
``` C++
// Function used to keep track of who has been seen
bool HeadControlNode::haveWeSeenThisPerson(FaceSeen face_detected)
{
    bool ret_val = true;

    // Is this person already in our list of people seen
    std::list<FaceSeen>::iterator it = std::find_if(seen_list_.begin(), seen_list_.end(),
    boost::bind(&FaceSeen::id, _1) == face_detected.id);

    if(it == seen_list_.end())
    {
        // Not seen before, add to seen list
        seen_list_.insert(it, face_detected);

        ret_val = false;
    }

    return ret_val;
}
```
The final function, __moveServo__, is the function called by main in our loop. It checks to see if a request to move the head was made.

If the target position was reached a check is made to see if a request for an individual scan should be published.

If the target position is not yet reached it calculates what movement of the servos should be made to move towards the target position and publishes a message to move the head.
``` C++
// Function to move the servos if required by a step amount. This is to stop the head shuddering if the servo
// is moved to the target position in one movement. The function also carries out any process required when
// the servo reaches the target.
void HeadControlNode::moveServo()
{
    if(move_head_ == true)
    {
        // Still processing servo movement
        if(movement_complete_ == true)
        {
            // We have reached the target but may be giving time to settle
            loop_count_down_--;
            
            if(loop_count_down_ <= 0)
            {
                movement_complete_ = false;
                move_head_ = false;
                
                if(process_when_moved_ == requestScan)
                {
                    // Request the next individual scan for this position
                    std_msgs::Empty mt;
                    start_individual_scan_pub_.publish(mt);
                }
            }
        }
        else
        {
            if((target_pan_tilt_.pan == current_pan_tilt_.pan) && (target_pan_tilt_.tilt == current_pan_tilt_.tilt))
            {
                // Last time around we must have requested the final move
                movement_complete_ = true;
                if(process_when_moved_ == requestScan)
                {
                    // Allow time for head to steady before requesting scan of image
                    loop_count_down_ = 5;
                }
                else
                {
                    loop_count_down_ = 1;
                }
            }        
            else
            {                
                // Still moving, calculate pan movement
                if(std::abs(target_pan_tilt_.pan - current_pan_tilt_.pan) > pan_step_)
                {
                    // Distance to target to great to move in one go
                    if(target_pan_tilt_.pan > current_pan_tilt_.pan)
                    {
                        // Add the step to current
                        current_pan_tilt_.pan += pan_step_;
                    }
                    else
                    {
                        // Subtract step from current 
                        current_pan_tilt_.pan -= pan_step_;
                     }
                }
                else 
                {
                    // Can move to the target position in one go (or pan is in fact already there but tilt is not)
                    current_pan_tilt_.pan = target_pan_tilt_.pan;                                                            
                }
        
                // Calculate tilt movement
                if(std::abs(target_pan_tilt_.tilt - current_pan_tilt_.tilt) > tilt_step_)
                {
                    // Distance to target to great to move in one go
                    if(target_pan_tilt_.tilt > current_pan_tilt_.tilt)
                    {
                        // Add the step to current
                        current_pan_tilt_.tilt += tilt_step_;
                    }
                    else
                    {
                        // Subtract step from current 
                        current_pan_tilt_.tilt -= tilt_step_;
                     }
                }
                else 
                {
                    // Can move to the target position in one go (or tilt is in fact already there but pan is not)
                    current_pan_tilt_.tilt = target_pan_tilt_.tilt;                                                            
                }
                
                
                // Publish the movement
                move_head_pub_.publish(current_pan_tilt_);
            }
        }
    }
}
```
The package also includes a launch file in the *launch* folder. This file, *test.launch*, will launch all the nodes discussed in these first two parts of this article in order for us to test the system so far developed.
``` XML
<?xml version="1.0" ?>
<launch>

  <rosparam command="load" file="$(find pan_tilt)/config/config.yaml" />
  <rosparam command="load" file="$(find face_recognition)/config/config.yaml" />
  <rosparam command="load" file="$(find head_control)/config/config.yaml" />
  
  <include file="$(find raspicam_node)/launch/camerav2_1280x960.launch" /> 
  <node name="republish" type="republish" pkg="image_transport" output="screen" args="compressed in:=/raspicam_node/image/ raw out:=/camera/image/raw" />
  
  <node pkg="pan_tilt" type="pan_tilt_node" name="pan_tilt_node" output="screen">
    <remap from="pan_tilt_node/index0_position" to="pan_tilt_node/head_position"/>
  </node>
  <node pkg="face_recognition" type="face_recognition_node.py" name="face_recognition_node" output="screen"/>
  <node pkg="rosserial_python" type="serial_node.py" name="serial_node" output="screen" args="/dev/ttyUSB0"/>
  <node pkg="head_control" type="head_control_node" name="head_control_node" output="screen"/>
</launch>
```
## Action client
Having included an action server in our node you would expect to have an action client to connect with it, this is definitely one method of communicating with the server. In the next part of the article I'm going to introduce a ROS package which will allow us to create state machines and sub-state machines to control our robot missions. Using this package it is possible to assign an individual state to be the action client and all the communication is done behind the scenes for us.

In order to test the system we have developed so far and to show how to write an action client we will write a test node here which will include an action client for our *scan_for_faces* action.

Our ROS package for the test node is called *rodney_recognition_test* and is available in the [rodney_recognition_test repository folder](https://github.com/phopley/Robotics-test-code/tree/master/rodney_recognition_test). The package contains all the usual ROS files and folders.

The *include/rodney_recognition_test* and *src* folders contain the C++ code for the package. For this package we have one C++ class, RodneyRecognitionTestNode and a main routine contained within the *rodney_recognition_test_node.cpp* file.

The __main__ routine informs ROS of our node, creates an instance of the class for the node and passes it the node handle, logs that the node has started and hands control to ROS with the call to __ros::spin__.
``` C++
int main(int argc, char **argv)
{
    ros::init(argc, argv, "rodney_test");
    ros::NodeHandle n;    
    RodneyRecognitionTestNode rodney_test_node(n);   
    std::string node_name = ros::this_node::getName();
	ROS_INFO("%s started", node_name.c_str());
    ros::spin();
    return 0;
}
```
The constructor creates an instance of our action client, __ac___, and passes it the name of the action server which in our case is *head_control_node*. This must match the name we gave to our action server when we created it in the __HeadControlNode__ constructor.

We are going to use a keyboard node which is [available here](https://github.com/lrse/ros-keyboard), to interact with the system. In the constructor we subscribe to the topic *keyboard/keydown* and call the function __keyboardCallBack__ when a message is received on that topic.

The call __ac_.waitForServer__ will wait in the constructor until our action server is running.
``` C++
// Constructor 
RodneyRecognitionTestNode::RodneyRecognitionTestNode(ros::NodeHandle n) : ac_("head_control_node", true)
{
    nh_ = n;
    
    // Subscribe to receive keyboard input
    key_sub_ = nh_.subscribe("keyboard/keydown", 100, &RodneyRecognitionTestNode::keyboardCallBack, this);

    ROS_INFO("RodneyRecognitionTestNode: Waiting for action server to start");

    // wait for the action server to start
    ac_.waitForServer(); //will wait for infinite time
    
    scanning_ = false;

    ROS_INFO("RodneyRecognitionTestNode: Action server started"); 
}
```
The function __keyboardCallBack__ checks the received message for one of two keys. If the lower case 's' is pressed we will start a complete scan using the robot head range of movement looking for faces recognised. It does this by creating an instance of our action goal and passes it to the action server with a call to __ac_.sendGoal__. With the call we pass three callback functions, 1) __doneCB__ which is called when the action is completed 2) __activeCB__ which is called when the action goes active and 3) __feedbackCB__ which is called when the feedback on the progress of the action is received.

The action can be preempted, so if the lower case 'c' is pressed and scanning is in progress we will cancel the action with a call to __ac_.cancelGoal__.
``` C++
void RodneyRecognitionTestNode::keyboardCallBack(const keyboard::Key::ConstPtr& msg)
{        
    // Check no modifiers apart from num lock is excepted
    if((msg->modifiers & ~keyboard::Key::MODIFIER_NUM) == 0)
    {
        // Lower case
        if(msg->code == keyboard::Key::KEY_s)
        {            
            // Lower case 's', start a complete scan looking for faces
            // Send a goal to the action
            face_recognition_msgs::scan_for_facesGoal goal;
        
            // Need boost::bind to pass in the 'this' pointer
            ac_.sendGoal(goal,
                boost::bind(&RodneyRecognitionTestNode::doneCB, this, _1, _2),
                boost::bind(&RodneyRecognitionTestNode::activeCB, this),                
                boost::bind(&RodneyRecognitionTestNode::feedbackCB, this, _1));
        }
        else if(msg->code == keyboard::Key::KEY_c)
        {          
            // Lower case 'c', cancel scan if one is running
            if(scanning_ == true)
            {
                ac_.cancelGoal();
            }        
        }
        else
        {
            ;
        }
    }
}
```
The callback function __activeCB__ is called when the action goes active, here we log the fact and set a member variable indicating that scanning is taking place.
``` C++
// Called once when the goal becomes active
void RodneyRecognitionTestNode::activeCB()
{
    ROS_DEBUG("RodneyRecognitionTestNode: Goal just went active");
    
    scanning_ = true;
}
```
The callback function __feedbackCB__ is called when feedback on the progress of the action is received. If you remember our feedback includes a percentage complete value and any faces seen on the last individual scan. Here we log the percentage complete and log any names of people just seen.
``` C++
// Called every time feedback is received for the goal
void RodneyRecognitionTestNode::feedbackCB(const face_recognition_msgs::scan_for_facesFeedbackConstPtr& feedback)
{
    ROS_DEBUG("Got Feedback percentage complete %f", feedback->progress);    
    
    if(feedback->detected.ids_detected.size() > 0)
    {  
        for(unsigned long x = 0; x < feedback->detected.ids_detected.size(); x++)
        {
            // Log just seen
            ROS_INFO("RodneyRecognitionTestNode: Just seen %s", feedback->detected.names_detected[x].c_str());          
        }          
    }
}
```
The callback function __donCB__ is called when the action is completed. The result data contains all the people seen during the complete scan of the head movement. Here we log a greeting to anyone seen.
``` C++
// Called once when the goal completes
void RodneyRecognitionTestNode::doneCB(const actionlib::SimpleClientGoalState& state,
                        const face_recognition_msgs::scan_for_facesResultConstPtr& result)
{
    ROS_DEBUG("RodneyRecognitionTestNode: Finished in state [%s]", state.toString().c_str());
    scanning_ = false;    

    if(result->detected.ids_detected.size() > 0)
    {  
        for(unsigned long x = 0; x < result->detected.ids_detected.size(); x++)
        {
            // Log we have seen you now!
            ROS_INFO("RodneyRecognitionTestNode: Hello %s, how are you?", result->detected.names_detected[x].c_str());            
        }            
    }
}
```
## Using the code
As previously when testing the code, I'm going to run the system code on the Raspberry Pi and the test code on a separate Linux workstation. The Raspberry Pi will also be connected to the Arduino nano which in turn is connected to the servos and running the sketch from part one of the article.

The package versions used in this test were:
* pan_tilt V0.1.5 [pan_tilt repository](https://github.com/phopley/pan_tilt)
* servo_msgs V0.1.2 [servo_msgs repository](https://github.com/phopley/servo_msgs)
* face_recognition V0.1.2 [face_recognition repository](https://github.com/phopley/face_recognition)
* face_recognition_msgs V0.1.1 [face_recognition_msgs repository](https://github.com/phopley/face_recognition_msgs)
* head_control V0.1.1 [head_control repository](https://github.com/phopley/head_control)

### Building the ROS packages on the Pi
If not already done create a catkin workspace on the Raspberry Pi and initialise it with the following commands:
```
$ mkdir -p ~/rodney_ws/src
$ cd ~/rodney_ws/
$ catkin_make
```
Copy the packages *rodney_recognition_test*, *face_recognition*, *face_recognition_msgs*, *head_control*, *pan_tilt* and *servo_msgs* into the ~/rodney_ws/src folder and then build the code with the following commands:
```
$ cd ~/rodney_ws/ 
$ catkin_make
```
Check that the build completes without any errors.
### Building the ROS test packages on the workstation
You can build and run the test packages on the Raspberry Pi but I'm going to use a Linux workstation which is on the same network as the Pi.

Create a workspace with the following commands:
```
$ mkdir -p ~/test_ws/src 
$ cd ~/test_ws/ 
$ catkin_make
```
Copy the packages *face_recognition_msgs*, *rodney_recognition_test* and *ros-keyboard* ([from](https://github.com/lrse/ros-keyboard)) into the *~/test_ws/src* folder and then build the code with the following commands:
```
$ cd ~/test_ws/ 
$ catkin_make
```
Check that the build completes without any errors.
### Running the code
Now we are ready to run our code. With the Arduino connected to a USB port use the launch file to start the nodes with the following commands. If no master node is running in a system the launch command will also launch the master node, roscore:
```
$ cd ~/rodney_ws/
$ source devel/setup.bash
$ roslaunch head_control test.launch
```
In the terminal you should see:

* a list of parameters now in the parameter server
* a list of our nodes
* the address of the master
* log information from our code

Next I'm going to use rqt_graph and our test code to test the system. I'm going to run the code on the separate workstation which needs to know the address of the machine running the ROS master, if you are running the test code on the Raspberry Pi you can ignore the command about the location of the ROS master.

On the workstation run the following commands to start the keyboard node:
```
$ cd ~/test_ws 
$ source devel/setup.bash 
$ export ROS_MASTER_URI=http://ubiquityrobot:11311 
$ rosrun keyboard keyboard
```
On the workstation in a second terminal run the following commands to start our test node:
```
$ cd ~/test_ws 
$ source devel/setup.bash 
$ export ROS_MASTER_URI=http://ubiquityrobot:11311 
$ rosrun rodney_recognition_test rodney_recognition_test_node
```
In a third terminal run the following commands to start rqt_graph:
```
$ cd ~/test_ws
$ export ROS_MASTER_URI=http://ubiquityrobot:11311
$ rqt_graph
```
<img src="https://github.com/phopley/rodney-project/blob/master/docs/images/part2_system_graph.png" title="system">
From the graph you should see the nodes for both the system and the test code running. You should also see the nodes linked by the topics. Any broken links is an indication of misspelt topics in the code.

The workstation should also be running a small window whose title is "ROS keyboard input". Make sure this window has the focus and then press the lower case 's' key. The head should start moving and scanning for faces it recognises as it goes. The terminal which you started the rodney_recognition_test_node in should report any faces it recognised for each individual scan. When all the scans are complete the head should return to the default position and a greeting for each individual seen will be displayed in the same terminal.

During a scan press the lower case 'c' key to cancel the action, the head should return to the default position.
