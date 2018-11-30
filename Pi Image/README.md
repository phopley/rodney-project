# Instructions for creating the Raspberry Pi image
## Introduction
These steps will guide you through the process of installing and setting up the Rapberry Pi image for the Rodney project. We will use a free lbuntu image from Ubiquity Robotics. This image inludes ROS Kinetic.

In the guide we will
* [Obtain and copy the Ubiquity Robotics Pi image](https://github.com/phopley/rodney-project/blob/master/Pi%20Image/README.md#1-obtain-and-copy-the-ubiquity-robotics-pi-image)
* [Configure and update the image] add link
* Next step and link

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
