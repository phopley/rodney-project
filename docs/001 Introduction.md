# Introduction
## Purpose
The Rodney Robot project is a hobbyist robotic project to design and build an autonomous house-bot. These document are intended as a hardware and software design document to capture the development of the robot. That said, although it will capture those things it will be written more like a blog.
## Background
Back sometime in the late 1970’s early 1980’s I brought the following books: “How to build your own self-programming robot” by David L. Heiserman and “How to build a computer-controlled robot” by Tod Loofbourrow. The plan was to build my own processor board based on a Z80 processor and then move on to build the robot around this board. It never really got off the ground. All these years later with modern small boards like the Raspberry Pi and Arduino the task of getting a home robot up and working is much easier, although the expectation of what it will be capable of doing is much greater.

Both books gave names to their robots “Rodney” and “Mike”. For that reason and in honor of one of these books the planned robot is called Rodney.

![alt text](https://github.com/phopley/rodney-project/blob/master/docs/images/book1.jpg "How to build your own self-programming robot")
![alt text](https://github.com/phopley/rodney-project/blob/master/docs/images/book2.jpg "How to build a computer-controlled robot")

As a software engineer I subscribe to the Code Project site https://www.codeproject.com and there are two robotic projects on the site which I have taken some inspiration from.

The first article [Let's build a robot!](https://www.codeproject.com/Articles/1115414/Lets-build-a-robot "Let's build a robot!") is a robot concept article and contains some great ideas, although from the article I'm not sure if the robot ever got built? Using a small display for the robot head is one of the main ideas I have taken away from the article but one of the most useful things I found in this article is a link to the [Pi Robot blog](http://www.pirobot.org/blog/0015 "Pi Robot blog"). This introduced me to the [Robot Operating System](http://wiki.ros.org/ "ROS") (ROS). This is a de facto standard for robot programming. As it states on the ROS Wiki:

*ROS (Robot Operating System) provides libraries and tools to help software developers create robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. ROS is licensed under an open source, BSD license.*

It's not really an operating system, it's more a middleware and is intended for use on a Linux platform (it runs best under Ubuntu). There is so much open source ROS code available for sensors that you can drop these libraries into your project leaving you available to concentrate on the robot application. The ROS Wiki is full of great information so head over there if you are unfamiliar with ROS.

The second article that I have taken inspiration from is [PiRex – remote controlled Raspberry Pi based robot](https://www.codeproject.com/Articles/1237052/PiRex-remote-controlled-Raspberry-Pi-based-robot "PiRex – remote controlled Raspberry Pi based robot"). Although not as ambitious as the first article, this one is written around a completed robot project. Both articles use a Raspberry Pi which are relatively cheap to buy.

So unlike in the early 80's instead of being faced with first building a processor board, Rodney will be built around a Raspberry Pi 3, Model B with 1GB of RAM.
