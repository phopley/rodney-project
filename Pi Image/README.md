# Instructions for creating the Raspberry Pi image
## Introduction
These steps will guide you through the process of installing and setting up the Rapberry Pi image for the Rodney project. We will use a free lbuntu image from Ubiquity Robotics. This image included ROS Kinetic and there own ROS packages like raspicam_node.

In the guide we will
* [Obtain and copy the Ubiquity Robotics Pi image](https://github.com/phopley/rodney-project/blob/master/Pi%20Image/README.md#1-obtain-and-copy-the-ubiquity-robotics-pi-image)
* [Configure and update the image](https://github.com/phopley/rodney-project/blob/master/Pi%20Image/README.md#2-configure-and-update-the-image)
* [Install TensorFlow]

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
* Add "export PYTHONPATH=$PYTHONPATH:~/git/models/research:~/git/models/research/slim:~/git/models/research/object_detection" to the end of the file.
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
