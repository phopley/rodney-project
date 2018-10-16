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
<img src="https://github.com/phopley/rodney-project/blob/master/docs/images/Opt-rodney_face.png" title="Testing face code">
In another terminal, enter the following command:
```
$ rqt_graph
```
From the graph, we can see that the node subscribes to the following topic:
* */robot_face/talking_finshed* - Once you have finished generating the speech, you are required to send a message on this topic
* */robot_face_expected_input* - With this topic, you can display a status message which appears below the face
* */robot_face_image_display* - Not used in the Rodney project
* */robot_face_ImageFileDisplay* - Not used in the Rodney project
* */robot_face/text_out* - Text in this topic is used to animate the mouth to match the voice and the text appears below the face. By embedding smileys in the text, you can change the facial expression
* */recognized/speech* - Not used in the Rodney project

We can use rostopic to see this in action. In a terminal, type the following commands:
```
$ rostopic pub -1 /robot_face/expected_input std_msgs/String "Battery Low"
```
You should see the status message displayed below the face.
```
$ rostopic pub -1 /robot_face/text_out std_msgs/String "Hello my name is Rodney:)"
```
You should see the text below the face (minus the smiley), the face should animate the speech and change from the neutral expression to an happy expression.

Now send the following command to indicate the speech is complete.
```
$ rostopic pub -1 /robot_face/talking_finshed st_msgs/String "q"
```
It doesn't matter what this __string__ contains but until you send this message, the face will not respond to another */robot_face/text_out message*.

In the */robot_face/text_out* message above, we changed the expression by use of a smiley ":)", the list below gives the smileys available.

* "." Neutral
* ":)" Happy
* ":(" Sad
* ">:" Angry
* ":!" Disgusted
* ":&" Frightened
* ":O" or ":o" Surprised

We will come back to the robot face node when we integrate it into our system.
## Giving Rodney a Voice
Now the installation of the robot face package will also have installed The MARY TTS System, however we are going to write a ROS node that uses the much simpler __pico2wav__ TTS system. Our node will use __pico2wav__ to generate a temporary *wav* file which will then be played back. We will also add functionality to play existing short wav files.

Our ROS package for the node is called __speech__ and the files that make up this package are available in the [speech repository](https://github.com/phopley/speech "speech repository"). The package contains all the usual ROS files and folders.

The *cfg* folder contains the file *speech.cfg*. This file is used by the dynamic reconfiguration server so that we can adjust some of the wav playback parameters on the fly. We used the dynamic reconfiguration server in part 1 of the article to trim the servos. This file contains the follow Python code.
``` Python
#!/usr/bin/env python
PACKAGE = "speech"

from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()

gen.add("pitch",  int_t,    0, "Playback Pitch",  -300,  -1000, 1000)
gen.add("vol",    double_t, 0, "Playback volume", 0.75, 0,   1)
gen.add("bass",   int_t,    0, "Bass",            0, -10, 10)
gen.add("treble", int_t,   0, "Treble",          0, -10, 10)
gen.add("norm",   bool_t,   0, "Normalise audio", True)

lang_enum = gen.enum([ gen.const("en_US", str_t, "en-US", "English US"),
                       gen.const("en_GB", str_t, "en-GB", "English GB"),
                       gen.const("fr_FR", str_t, "fr-FR", "French"),
                       gen.const("es_ES", str_t, "es-ES", "Spanish"),
                       gen.const("de_DE", str_t, "de-DE", "German"),
                       gen.const("it_IT", str_t, "it-IT", "Italian")],
                     "An enum to set the language")

gen.add("lang", str_t, 0, "Voice language", "en-GB", edit_method=lang_enum)

exit(gen.generate(PACKAGE, "speechnode", "Speech"))
```
For a complete understanding of the dynamic reconfiguration server, refer to the ROS Wiki [section dynamic reconfiguration](http://wiki.ros.org/dynamic_reconfigure "section dynamic reconfiguration"). For now in our file, you can see that we add six parameters to the dynamic configuration server.

The *msg* folder contains a definition file for a user defined message. The file is named *voice.msg* and contains the following:
```
string text # Text to speak
string wav  # Path to file to play
```
The message contains two elements, __text__ will contain the text to turn into speech and wav will contain a path and filename of a wav file to play. Our code will first check to see if wav contains a path and if so, it will play the wav file, if __wav__ is an empty __string__, then __text__ will be used to create a wav file.

The *include/speech* and *src* folders contain the C++ code for the package. For this package we have one C++ class, __SpeechNode__ and a __main__ routine contained within the *speech_node.cpp* file.

The __main__ routine informs ROS of our node, creates an instance of our class which contains the code for the node, passes a callback function to the dynamic reconfiguration server and creates a __ros::Rate__ variable which will be used to time our control loop to 10Hz. When inside the loop we call __r.sleep__, this __Rate__ instance will attempt to keep the loop at 10Hz by accounting for the time used to complete the work in the loop.

Our loop will continue while the call to __ros::ok__ returns __true__, it will return __false__ when the node has finished shutting down, e.g., when you press Ctrl-c on the keyboard.

In the loop, we will call __speakingFinshed__ which is described later in the article.
``` C++
int main(int argc, char **argv)
{
	ros::init(argc, argv, "speech_node");
			
	SpeechNode *speech_node = new SpeechNode();
	
	dynamic_reconfigure::Server<speech::SpeechConfig> server;
    dynamic_reconfigure::Server<speech::SpeechConfig>::CallbackType f;
  	
  	f = boost::bind(&SpeechNode::reconfCallback, speech_node, _1, _2);
    server.setCallback(f);
  
    std::string node_name = ros::this_node::getName();
	ROS_INFO("%s started", node_name.c_str());
	
	// We want a delay from when a speech finishes to when /robot_face/talking_finished is published
	
	ros::Rate r(speech_node->LOOP_FREQUENCY_);
	
    while(ros::ok())
    {
        // See if /robot_face/talking_finished should be published published
        speech_node->speakingFinished();
        
        ros::spinOnce();
        r.sleep();
    }
    
	return 0;
}
```
The constructor for our class subscribes to the topic */speech/to_speak* in order to receive the text to speak or the location of the wav file to play. It also advertises that it will publish the topic */robot_face/talking_finshed*. As we know, this topic informs the face that the talking has finished.
``` C++
// Constructor 
SpeechNode::SpeechNode()
{
    voice_sub_ = n_.subscribe("/speech/to_speak", 5, &SpeechNode::voiceCallback, this);

    talking_finished_pub_ = n_.advertise<std_msgs::String>("/robot_face/talking_finished", 5);
    
    finshed_speaking_ = false;
}
```
I'll now briefly describe the functions that make up the class.

The function __reconfCallback__ is called by the dynamic reconfiguration server if any of the parameters are changed. The function simply stores the new values for use in the next playback of the temporary speech wav file.
``` C++
// This callback is for when the dynamic configuration parameters change
void SpeechNode::reconfCallback(speech::SpeechConfig &config, uint32_t level)
{
    language_ = config.lang;
    vol_ = config.vol;
    pitch_ = config.pitch;
    bass_ = config.bass;
    treble_ = config.treble;
    norm_ = config.norm;
}
```
The __voiceCallback__ function is called when a message on the */speech/to_speak* topic is received. If the wav element of the message is not empty, then the supplied wav filename is played using sox (Sound eXchange). Note that since we want to playback an already existing wav file and not to speak in our robot's voice, none of the dynamic reconfiguration parameters are used.

If the __wav__ element is empty, then __string__ in the __text__ element is to be spoken. We start by constructing a __string__ for the call to __pico2wav__, this __string__ includes our temporary filename, and the language parameter. The call to __pico2wav__ should result in the creation of a wav file without text converted to speech. A __string__ is then constructed to be used in making a system call to sox, this time we use the dynamic reconfiguration parameters so that we can control the sound of the robot's voice. For example, __pico2wav__ only contains a female voice but by changing the pitch, we can give the robot a male voice (which we want since ours is called Rodney).

The function finishes by setting a flag to indicate that we need to send a message on the */robot_face/talking_finshed* topic. We also set a countdown counter value which is used to time 20 executions of the control loop before the */robot_face/talking_finshed* message is sent.
``` C++
// This callback is for when a voice message received
void SpeechNode::voiceCallback(const speech::voice& voice)
{               
    // Check to see if we have a path to a stock wav file
    if(voice.wav != "")
    {       
        // Play stock wav file
        // Use play in sox (Sound eXchange) to play the wav file, 
        // --norm stops clipping and -q quite mode (no console output)
        std::string str = "play " +  voice.wav + " --norm -q";        
                                             
        ROS_DEBUG("%s", str.c_str());               
        
        if(system(str.c_str()) != 0)
        {
            ROS_DEBUG("SpeechNode: Error on wav playback");            
        }
    }
    else
    {                        
        std::string filename = "/tmp/robot_speach.wav";                         
        
        std::string str;
        
        // create wav file using pico2wav from adjusted text.
        str = "pico2wave --wave=" + filename + " --lang=" + language_ + " \"" + voice.text + "\"";
        
        ROS_DEBUG("%s", str.c_str());
        
        if(system(str.c_str()) != 0)
        {
            ROS_DEBUG("SpeechNode: Error on wav creation");            
        }
        else
        {                
            // Play created wav file using sox play but use parameters bass, treble, pitch and vol 
            std::string bass = " bass " + std::to_string(bass_);
            std::string treble = " treble " + std::to_string(treble_);
            std::string pitch = " pitch " + std::to_string(pitch_);        
        
            if(norm_ == true)
            {            
                str = "play " +  filename + " --norm -q" + pitch + bass + treble;          
            }
            else
            {
                std::string volume = " vol " + std::to_string(vol_);
                str = "play " +  filename + " -q" + pitch + bass + treble + volume;           
            }
        
            ROS_DEBUG("%s", str.c_str());               
        
            if(system(str.c_str()) != 0)
            {
                ROS_DEBUG("SpeechNode: Error on wav playback");            
            }
        }
    }
    
    // Set up to send talking finished
    finshed_speaking_ = true;
    loop_count_down_ = (int)(LOOP_FREQUENCY_ * 2);
}
```
The function __speakingFinished__ is called by the control loop in main. If we have kicked off the playback of either a wav file that already exists or our temporary wav file of text to speak, the function will count down each time it is called. When the counter reaches zero, the talking finished message is published. This gives the robot face node 2 seconds to animate the face before the finished speaking message is sent. You can increase this time if you find your robot has a lot to say, but bear in the mind that the __pico2wav__ is intended for use with a limited number of characters for text to speech conversion.
``` C++
// If finshed speaking delay until the /robot_face/talking_finished topic is published
void SpeechNode::speakingFinished()
{
    if(finshed_speaking_ == true)
    {
        loop_count_down_--;
        
        if(loop_count_down_ <= 0)
        {
            finshed_speaking_ = false;

            // Send talking finished 
            std_msgs::String msg;   
            msg.data = "";
            talking_finished_pub_.publish(msg);        
        }    
    }
}
```
## Face and Voice Integration
In the next chapter, we will bring the nodes from Goal 1 and Goal 2 together along with a state machine package that will be used to control the robot missions. For now, it is worth testing the robot face with our speech node.

Our ROS package for the test node is called __rodney_voice_test__ and the files that make up this package are available in the [Robotics-test-code repository rodney_voice_test folder](https://github.com/phopley/Robotics-test-code/tree/master/rodney_voice_test "Robotics-test-code repository rodney_voice_test folder").

The *include/rodney_voice_test* and *src* folders contain the C++ code for the package. For this package, we have one C++ class, __RodneyVoiceTestNode__ and a __main__ routine contained within the *rodney_voice_test_node.cpp* file.

The __main__ routine informs ROS of our node, creates an instance of the class for the node and passes it the node handle, logs that the node has started and hands control to ROS with the call to __ros::spin__.
``` C++
int main(int argc, char **argv)
{
    ros::init(argc, argv, "rodney_voice_test");
    ros::NodeHandle n;    
    RodneyVoiceTestNode rodney_test_node(n);   
    std::string node_name = ros::this_node::getName();
	ROS_INFO("%s started", node_name.c_str());
    ros::spin();
    return 0;
}
```
We are going to use a keyboard node which is available from https://github.com/lrse/ros-keyboard, to interact with the system. In the constructor, we subscribe to the topic *keyboard/keydown* and call the function __keyboardCallBack__ when a message is received on that topic.

The constructor also advertises that the node will publish the topics for the speech and robot face node.
``` C++
RodneyVoiceTestNode::RodneyVoiceTestNode(ros::NodeHandle n)
{
    nh_ = n;
    
    // Subscribe to receive keyboard input
    key_sub_ = nh_.subscribe("keyboard/keydown", 100, &RodneyVoiceTestNode::keyboardCallBack, this);

    // Advertise the topics we publish
    speech_pub_ = nh_.advertise<speech::voice>("/speech/to_speak", 5);
    face_status_pub_ = nh_.advertise<std_msgs::String>("/robot_face/expected_input", 5);
    text_out_pub_ = nh_.advertise<std_msgs::String>("/robot_face/text_out", 5);

}
```
The function __keyboardCallBack__ checks the received message for one of three keys. If the lower case 's' is pressed, we test the status display functionality by creating a message and publish it on the */robot_face/expected_input* topic.

If the lower case 't' is pressed, we test the speech and speech animation by creating two messages, one that contains the text to speak and the other contains the text to animate the robot face. Note how we add the ':)' smiley to the __greeting__ variable after we have used it to create the text to speak message, we don't want __pico2wav__ trying to speak this as part of the text. We then publish the two messages, one to the face and the other to the speech node.

If the lower case 'w' is pressed, we test the wav file playback and speech animation again by creating two messages. This time, the message going to the speech node contains the path to a wav file instead of the text to speak. Notice however the message to the robot face still contains text to match the contents of the wav file so that the face is still animated during playback.
``` C++
void RodneyVoiceTestNode::keyboardCallBack(const keyboard::Key::ConstPtr& msg)
{  
    // Check no modifiers apart from num lock is excepted
    if((msg->modifiers & ~keyboard::Key::MODIFIER_NUM) == 0)
    {
        // Lower case
        if(msg->code == keyboard::Key::KEY_s)
        {            
            // Test status display
            std_msgs::String status_msg;            
            status_msg.data = "Rodney on line";
            face_status_pub_.publish(status_msg);                                       
        }
        else if(msg->code == keyboard::Key::KEY_t)
        {
            // Test speech and animation
                      
            // String to send to robot face
            std_msgs::String greeting;
            greeting.data = "Hello my name is Rodney";
            
            // Voice message
            speech:: voice voice_msg;
            voice_msg.text = greeting.data;
            voice_msg.wav = "";
            
            // Add the smiley
            greeting.data += ":)";
            
            // Publish topics for speech and robot face animation
            text_out_pub_.publish(greeting);
            speech_pub_.publish(voice_msg);
        }
        else if(msg->code == keyboard::Key::KEY_w)
        {
            // Test wav playback and animation
            // String to send to robot face
            std_msgs::String greeting;
            greeting.data = "Danger Will Robinson danger:&";
            
            speech:: voice voice_msg;            
            std::string path = ros::package::getPath("rodney_voice_test");
            voice_msg.text = "";
            voice_msg.wav = path + "/sounds/lost_in_space_danger.wav";            
        
            // Publish topics for sound and robot face animation
            text_out_pub_.publish(greeting);
            speech_pub_.publish(voice_msg);        
        }
        else
        {
            ;
        }
    }
}
```
The folder *launch* contains the file *test.launch*. This file will be used to launch the two nodes under test and the two test nodes from one terminal.
``` XML
<?xml version="1.0" ?>
<launch>  
  <node pkg="homer_robot_face" type="RobotFace" name="RobotFace" output="screen"/>
  <node pkg="speech" type="speech_node" name="speech_node" output="screen"/>
  <node pkg="rodney_voice_test" type="rodney_voice_test_node" 

        name="rodney_voice_test_node" output="screen" />
  <node pkg="keyboard" type="keyboard" name="keyboard" output="screen" />
</launch>
```
## Using the code
You can test the code on either a Linux PC or on the robot hardware, in my case a Raspberry Pi.
### Robot Hardware
Now if you are testing the code on a PC, you probably already have a speaker and amplifier built in, but as our robot is built around a Raspberry Pi, we need some hardware to hear the voice playback. I have added an Adafruit Mono 2.5W Class D Audio Amplifier PAM8302 and an 8 Ohn speaker to the hardware. I have simply connected this to the Pi audio jack, the speaker and Pi's 5V supply.

The audio amp is on a small Veroboard mounted on the back of the tilt arm and the speaker is mounted on the front of the neck, just below the pan servo.
