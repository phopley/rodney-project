# Instructions for creating the Raspberry Pi image
## Introduction
These steps will guide you through the process of installing and setting up the Rapberry Pi image for the Rodney project. We will use a free lbuntu image from Ubiquity Robotics. This image included ROS Kinetic and there own ROS packages like raspicam_node.

In the guide we will
* [Obtain and copy the Ubiquity Robotics Pi image](https://github.com/phopley/rodney-project/blob/master/Pi%20Image/README.md#1-obtain-and-copy-the-ubiquity-robotics-pi-image)
* [Configure and update the image](https://github.com/phopley/rodney-project/blob/master/Pi%20Image/README.md#2-configure-and-update-the-image)
* [Install TensorFlow](https://github.com/phopley/rodney-project/blob/master/Pi%20Image/README.md#3-install-tensorflow)
* [Bluetooth](https://github.com/phopley/rodney-project/blob/master/Pi%20Image/README.md#4-bluetooth)
* [System Configuration](https://github.com/phopley/rodney-project/blob/master/Pi%20Image/README.md#5-system-configuration)
* [Homer Robot Face](https://github.com/phopley/rodney-project/blob/master/Pi%20Image/README.md#5-homer-robot-face)
* [Voice Applications](https://github.com/phopley/rodney-project/blob/master/Pi%20Image/README.md#6-voice-applications)
* [ROS pakages used by Rodney](https://github.com/phopley/rodney-project/blob/master/Pi%20Image/README.md#7-ros-pakages-used-by-rodney)
* [Swapfile](https://github.com/phopley/rodney-project/blob/master/Pi%20Image/README.md#8-swapfile)

## Steps
### 1. Obtain and copy the Ubiquity Robotics Pi image
Here we will download and install the image.
- [ ] Download the lastest ROS image from the Ubiquity Robotics web site https://downloads.ubiquityrobotics.com/pi.html. The last image I used was 2018-06-27-ubiquity-xenial-lxde-raspberry-pi.img.xz
- [ ] Unzip the image.
- [ ] Format a 16GB SD card. I SD Card Formatter to format the card.
- [ ] Copy the unzipped image to the SD card. I use Win32DiskImager to create the image on an SD card.
- [ ] Boot the Pi from the SD card. When the Raspberry Pi boots for the first time it resizes the file system to fill the SD card. This can make the first boot take some time.
- [ ] Login to the Pi. The user name is 'ubuntu' and the password is 'ubuntu'.
### 2. Configure and update the image
- [ ] The image comes with a WiFi access point. The SSID is ubiquityrobotXXXX where XXXX is part of the MAC address, the wifi password is robotseverywhere. Since I'm developing an house robot I prefer to connect to my home Wi-Fi. Disconnect the WiFi from the access point and connect to the home WiFi network. SSH is enabled so it is possible to remotley login.
- [ ] There is an Ubiquity Robotics script designed for there own robots. Disable the script with the following shell command.
```
$ sudo systemctl disable magni-base
```
- [ ] Run the following commands to get the latest updates.
```
$ sudo apt-get update
$ sudo apt-get upgrade
$ sudo apt-get dist-upgrade
$ sudo reboot now
```
### 3. Install TensorFlow
TensorFlow and Proto buffers are used for object detection.
- [ ] Install TensorFlow for Python 2.7 with the following commands: 

* Note 1. The github site https://github.com/lhelontra/tensorflow-on-arm contains pre-compiled installation packages. Check the site for the latest version. 
* Note 2. Files with cp27 are for Python 2.7 and cp35 are for Python 3.5.
```
$ cd ~/Downloads
$ wget https://github.com/lhelontra/tensorflow-on-arm/releases/download/v1.11.0/tensorflow-1.11.0-cp27-none-linux_armv7l.whl
$ sudo --set-home pip install ~/Downloads/tensorflow-1.11.0-cp27-none-linux_armv7l.whl
```
- [ ] If you try and import tensorflow into Python at this point you may get an ImportError: /usr/lib/arm-linux-gnueabihf/libstdc++.so.6: version 'GLIBCXX_3.4.22' not found. The following fixes this:
```
$ sudo add-apt-repository ppa:ubuntu-toolchain-r/test
$ sudo apt-get update
$ sudo apt-get install gcc-4.9
$ sudo apt-get upgrade libstdc++6
```
- [ ] Go ahead and test that the module can be loaded:
```
$ python
>>> import tensorflow as tf
```
- [ ] The TensorFlow object detection API package uses Protobuf. Run the following to compile and install Protobuf:
* Note Check the site for the latest version https://github.com/protocolbuffers/protobuf/releases.
```
$ sudo apt-get install libatlas-base-dev
$ sudo apt-get install libxml2-dev libxslt1-dev
$ sudo --set-home pip install pillow lxml matplotlib cython
$ python2 -m pip install ipython==5.7 --user
$ python2 -m pip install ipykernel==4.10 --user
$ sudo apt-get install python-tk
$ sudo apt-get install autoconf automake libtool curl make g++ unzip
$ cd ~/Downloads
$ wget https://github.com/google/protobuf/releases/download/v3.6.1/protobuf-all-3.6.1.tar.gz
$ tar -zxvf protobuf-all-3.6.1.tar.gz
$ cd protobuf-3.6.1
$ ./configure
$ export CPPFLAGS="-std=c++11"
$ make
$ make check
$ sudo make install
$ sudo ldconfig # refresh shared library cache.
$ cd python
$ export LD_LIBRARY_PATH=../src/.libs
$ python2 setup.py build --cpp_implementation
$ python2 setup.py test --cpp_implementation
$ sudo python2 setup.py install --cpp_implementation
$ export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=cpp
$ export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION_VERSION=3
$ sudo ldconfig
```
- [ ] Go ahead and test the installation by running the following command and checking the help information is displayed:
```
$ protoc
$ sudo reboot now
```
- [ ] Next clone the TensorFlow model repository and setup the PYTHONPATH variable by running the following commands:
```
$ mkdir ~/git
$ cd ~/git
$ git clone --recurse-submodules https://github.com/tensorflow/models.git
$ sudo nano ~/.bashrc
```
* Add the following to the end of the file.
```
export PYTHONPATH=$PYTHONPATH:~/git/models/research:~/git/models/research/slim:~/git/models/research/object_detection
```
* Exit nano saving the file.
- [ ] Close and reopen the terminal then run the following:
```
$ cd ~/git/models/research
$ protoc object_detection/protos/*.proto --python_out=.
```
- [ ] Next download a pre-trained model from the Tensorflow detection model zoo. Here we are using the SSDLite-MobileNet model.
```
$ cd ~/git/models/research/object_detection
$ wget http://download.tensorflow.org/models/object_detection/ssdlite_mobilenet_v2_coco_2018_05_09.tar.gz
$ tar -xzvf ssdlite_mobilenet_v2_coco_2018_05_09.tar.gz
```
### 4. Bluetooth
I sometime like to use a small Bluetooth keyboard with the robot but Bluetooth is not enabled correctly in this image.
- [ ] Run the following commands to fix this:
```
$ sudo nano /boot/config.txt
```
* Add the following line between the lines "dtoverlay=pi3-miniuart-bt" and "dtoverlay=i2c-rtc,mcp7940x". 
```
core_freq=250
```
* Exit nano saving the file.
### 5. System Configuration
The following configures the lbunutu desktop and system for how I like it for a robot.
- [ ] So that you only have to touch 'login' on the screen and not enter a password to login select from the menu "System Tools -> Users and Groups", change to Password "Not asked on login"
- [ ] Although the image includes Firefox I like to install Chromium with:
```
$ sudo apt install chromium-browser
```
- [ ] I don't want the screen saver or power managment to kick in so from the menu "Preferences->Power Manager" select
* System tab - When inactive for -> never
* Display tab - Blank after -> never
* Display tab - Put to sleep after - > never
* Display tab - switch off after -> never
* Security tab - Automatically lock the session : Never
- [ ] You may want to change the user password.
- [ ] I like to set the following panel applets to my choice. Right click on bar and select "Add/Remove Panel Items" and select:
* Menu
* Directory Menu - "/home/ubuntu"
* Application Launch and Task Bar (stretch) - Chromium Web Browser
* System Tray
* Indicator applets
* Digital Clock
### 5. Homer Robot Face
The Homer Robot Face ROS package is used to display an animated robot face.
- [ ] Install the homer_robot_face package
```
$ sudo apt-get install ros-kinetic-homer-robot-face
```
- [ ] There is a problem with a directory path. The code expects the OGRE to be in the directory `/usr/lib/x86_64-linux-gnu/OGRE-1.9.0.` but as you would expect on a Raspberry Pi it's in `/usr/lib/arm-linux-gnueabihf/OGRE-1.9.0.` We can solve this problem by creating a symbolic link
```
$ cd /usr/lib
$ sudo mkdir -p x86_64-linux-gnu
$ sudo ln -s /usr/lib/arm-linux-gnueabihf/OGRE-1.9.0 /usr/lib/x86_64-linux-gnu
```
- [ ] You can configer the face by editing the robot_homer_face config.cfg file
```
sudo nano /opt/ros/kinetic/share/homer_robot_face/config/config.cfg
```
* Mesh Filename : GiGo
* Head Color : 1.0 1.0 1.0
* Iris Color : 0.0 1.0 1.0
* Outline Color : 0.0 0.0 0.0
* Voice : male
* Window Width : 400
* Window Height : 450
* Window Rotation : 0
- [ ] To centre the face when running. Edit the lubuntu-rc.xml file and add the following application tag inside the applications tag
```
$ cd ~/
$ nano .config/openbox/lubuntu-rc.xml
```
``` xml
        <application name="RobotFace">
            <position force="yes">
                <x>200</x>
                <y>0</y>
            </position>
        </application>
</applications>
</openbox_config>
```
### 6. Voice Applications
The applications `pico2wav` and `Sox` are used by Rodney.
- [ ]  Install pico2wav with the following command:
```
$ sudo apt-get install libttspico-utils
```
- [ ] Install Sox with the following command:
```
sudo apt-get install sox libsox-fmt-all
```
### 7. ROS pakages used by Rodney
The following packages ROS packages are also used by Rodney and can be download and installed/compiled.
- [ ] I usually use the directory ~/rondey_ws for the ROS wokspace, add it to the bash file with the following
```
$ cd ~/
$ nano .bashrc
```
* Add "source /home/ubuntu/rodney_ws/devel/setup.bash" to the end of the file, save and exit
- [ ] Install the rosserial package so that the Arduino or Teensy can be used
```
$ sudo apt-get install ros-kinetic-rosserial
```
- [ ] Rodney use the ros-keyboard package for keyboard teleop. It requires SDL version 1.2. If you just try to install SDL you will get a later version and the package will not build. Use the following commands to download the package from https://github.com/lrse/ros-keyboard and use rosdep to ensure the correct dependencies are installed before building it with catkin_make.
* Note - I have assumed the package was download to ~/git/ros-keyboard
```
$ mkdir -p ~/temp_ws/src
$ cd ~/temp_ws
$ catkin_make
$ ln -s ~/git/ros-keyboard ~/temp_ws/src
$ rosdep update
$ rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
$ catkin_make
```
### 8. Swapfile
As the project develops and increases in size a swapfile is required when building the project. Create a 1GB swapfile with the following command:
```
$ cd /
$ sudo dd if=/dev/zero of=swapfile bs=1M count=1000
$ sudo mkswap swapfile
$ sudo swapon swapfile
$ sudo nano etc/fstab
```
* Add "/swapfile none swap sw 0 0" to the end of the file , save and exit
