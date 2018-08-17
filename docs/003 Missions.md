# Robotic Missions
In order to come up with the requirements for the project I'm going to specify some "missions" that I would like Rodney to be able to perform. In the article "Let's build a robot!" the author lists jobs he would like the robot to do around the house. One of these jobs is:

*Take a message to... - Since the robot will [have] the ability to recognize family members, how about the ability to make it the 'message taker and reminder'. I could say 'Robot, remind (PersonName) to pick me up from the station at 6pm'. Then, even if that household member had their phone turned on silent, or were listening to loud music or (insert reason to NOT pick me up at the station), the robot could wander through the house, find the person, and give them the message.*

This sounds like a good starting point and will be our first mission. I'm going to change it slightly though, what if you could access Rodney using a web browser to control and set missions.

Let's breakdown the "Take a message to..." mission into several smaller design goals that can be worked on and completed individually. The design goals for this mission will be:

* Design Goal 1: To be able to look around using the camera, search for faces, attempt to identify any people seen and display a message for any identified
* Design Goal 2: Facial expressions and speech synthesis. Rodney will need to be able to deliver the message
* Design Goal 3: Locomotion controlled by a remote keyboard and/or joystick
* Design Goal 4: Addition of a laser ranger finder or similar ranging sensor used to aid navigation
* Design Goal 5: Autonomous locomotion
* Design Goal 6: Task assignment and completion notification

That's quite a list of things to accomplish for what seems like a simple mission for a robot.
