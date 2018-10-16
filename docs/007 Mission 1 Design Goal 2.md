# Mission 1, Design Goal 2
To accomplish this design goal we will need to

* Add facial expression
* Add a voice
## Facial Expression
The hardware making up the robot head includes the 7" Touchscreen Display which is ideal for giving the robot a face, but utilising it for facial expressions seems like a big task. However, I have explained that there are many ROS packages available from the ROS community leaving us free to concentrate on the robot application and that's exactly what we are going to make use of here.

We will make use of the __homer_robot_face__ package from the University of Koblenz. This package includes two different selectable faces but it is also possible to model your own character. This package also includes speech synthesis using the Mary TTS (Text to Speech) generator. As this consumes a lot of memory, it is not suitable for single board computers, but we will write our own TTS node suitable for the Raspberry Pi later in this article.

This video from the University of Koblenz shows the range of facial expressions available with the package.
[![IMAGE ALT TEXT](http://img.youtube.com/vi/jgcztp_jAQE/0.jpg)](http://www.youtube.com/watch?v=jgcztp_jAQE "Video Title")
