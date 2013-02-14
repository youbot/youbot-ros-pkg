# youbot-ros-pkg Installation
===============

## Install Ubuntu
The repository and its related components have been tested under the following Ubuntu distributions:

- Ubuntu 10.04 (ROS Electric, ROS Fuerte)
- Ubuntu 11.10 (ROS Electric, ROS Fuerte)
- Ubuntu 12.04 (ROS Fuerte)

If you do not have a Ubuntu distribution on your computer you can download it here

     http://www.ubuntu.com/download

## Git - Version Control
### Install Git Software
Install the Git core components:

     sudo apt-get install git-core


## ROS - Robot Operating System
### Install ROS Fuerte
The repository has been tested successfully with the following ROS distributions. Use the link behind a ROS distribution to get to the particular ROS installation instructions.

- ROS Fuerte - http://www.ros.org/wiki/fuerte/Installation/Ubuntu
- ROS Electric - http://www.ros.org/wiki/electric/Installation/Ubuntu

NOTE: Do not forget to update your .bashrc! 


## Clone and compile the youbot-ros-pkg Software
First of all you have to clone the RoboCupAtHome repository.

    cd ~/ros_stacks
    git clone git@github.com:youbot/youbot-ros-pkg.git
    cd ~/ros_stacks/youbot-ros-pkg

Switch to the correct branch:

    git checkout electric --  if you want to work with ros electric
    git checkout fuerte -- if you want to work with ros fuerte

Then go on with installing further external dependencies:
    
    sudo easy_install -U rosinstall vcstools

    ./repository.debs
    rosinstall .. /opt/ros/fuerte repository.rosinstall
    
    echo "export ROS_PACKAGE_PATH=~/ros_stacks:\$ROS_PACKAGE_PATH" >> ~/.bashrc
    source ~/.bashrc
    

