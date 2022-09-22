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
8. [99-keys file workaround](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md#linux-device-setup)
9. Follow instructions in the [last comment](https://github.com/microsoft/Azure_Kinect_ROS_Driver/issues/123#issuecomment-661464652) to set up the Azure Kinect ROS driver.
10. Once the ROS driver is installed in your workspace, activate the workspace, and launch the node using `roslaunch azure_kinect_ros_driver driver.launch`. You can use tab completion to get the exact name of the package if this command fails. (Type `roslaunch azure_kinect` then press tab to have it autocomplete)
    
## ABB Controller Setup
1. Use the files in `abb_driver/rapid` and follow the instructions [here](http://wiki.ros.org/abb_driver/Tutorials/InstallServer).
2. Note the instructions [here](http://wiki.ros.org/abb_driver/Tutorials/RunServer) for running the ROS server on the controller.

## Docker container setup
1. Inside the cloned GitHub folder, use `docker build . -t ssr_ros` to build the docker image. Read the comments in this file to get an idea of how the container is laid out.
2. Once this is complete, run `docker run -dit ssr-ros` to launch the container.
3. Use `docker ps` to get the container ID of the running container.
4. Use `docker exec -it container_id bash` to access the running container's terminal. Note: replace container_id with your container id from step 3
5. `cd ssr_ros` & `catkin_make` to build the workspace
6. `source ./devel/setup.bash` to activate
7. Visit ROS documentation for `rosnode` `roscore` `rostopic` and `roslaunch` to learn more about the workspace.
8. To execute a scan, you have to have a roscore running in one window. Open a new terminal on your PC, connect to the container again with step 4. Activate the workspace in this new window, and then use `roslaunch abb_irb2400_moveit_config moveit_planning_execution.launch robot_ip:=100.0.0.1` where the IP address matches the IP of the robot. Finally, open a third terminal, connect to the container again. Acitvate the workspace. Run the scan script using `rosrun ssr_scan Scanner.py` you may have to move to `src/ssr_scan/src` and `chmod +x Scanner.py` to make the file executable.


## New Setup
0. Clone this repo (https://github.com/AugmentedRoboticFabrication/AugRobFab-ROS) using `git clone https://github.com/AugmentedRoboticFabrication/AugRobFab-ROS`
1. Download Docker, create an account. Run command `docker login` to prevent permissions error. Inside the cloned GitHub folder, use `docker build . -t ssr_ros` to build the docker image. Read the comments in this file to get an idea of how the container is laid out.
2. Once this is complete, run `docker run --rm -d ssr_ros:latest` to launch the container.
3. Use `docker ps` to get the container ID of the running container.
4. Use `docker exec -it container_id bash` to access the running container's terminal. Note: replace container_id with your container id from step 3
5. Run `source /opt/ros/melodic/setup.bash`, then `cd ssr_ros` & `catkin_make` to build the workspace
6. `source ./devel/setup.bash` to activate
7. Visit ROS documentation for `rosnode` `roscore` `rostopic` and `roslaunch` to learn more about the workspace.
8. To execute a scan, you have to have a roscore running in one window. Open a new terminal on your PC, connect to the container again with steps 4-6. Activate the workspace in this new window, and then use `roslaunch abb_irb2400_moveit_config moveit_planning_execution.launch robot_ip:=100.0.0.1` where the IP address matches the IP of the robot. Finally, open a third terminal, connect to the container again. Acitvate the workspace. Run the scan script using `rosrun ssr_scan Scanner.py` you may have to move to `src/ssr_scan/src` and `chmod +x Scanner.py` to make the file executable.
