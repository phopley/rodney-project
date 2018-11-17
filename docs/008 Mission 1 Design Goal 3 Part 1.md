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
