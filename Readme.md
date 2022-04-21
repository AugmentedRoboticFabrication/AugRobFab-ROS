# SSR ROS Install Process

## Pre-reqs
* Git installed on your PC
* Github account and access to the SSR repository
* [Docker](https://docs.docker.com/desktop/windows/install/) installed on your PC
* Internet access

## Clone SSR Github Repo
1. Clone the [SSR ROS Repo](https://github.com/lyoder3/SSR_ROS.git) to your PC: 
   
   `git clone https://github.com/lyoder/SSR_ROS.git`
2. This repository has all the files necessary to setup the SSR ROS system
3. Explanation of directories\
   1. 


## Jetson Nano Setup
1. Follow the latest NVIDIA Jetson Nano 2GB [JetPack Install Instructions](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-2gb-devkit#write) to make a microSD card that you can boot your Jetson from.
2. Plug this SD card into the Jetson Nano and go through the setup process. This run is pretty straightforward. Accept all default settings.
3. Once you have a Jetson desktop ready to go and are connected to the internet, start by running `sudo apt update` to update all of your package repositories.
4. First you need to install ROS. Follow the [instructions here](http://wiki.ros.org/melodic/Installation/Ubuntu). Choose to install `ros-desktop` at that step.
5. Make sure to complete all steps to the end of this document.
6. Next, you need to install the Azure Kinect SDK.
7. Run the following commands:
   1. `curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -`
   2. `sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/multiarch`
8. 99-keys file
9. Install all packages from source in catkin workspace
   1.  Change boost python line in CMakeLists for cv-bridge under opencv-vision
10. 