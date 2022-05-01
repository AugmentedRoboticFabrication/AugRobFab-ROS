# Start with the official ros melodic image
FROM ros:melodic-ros-base

# Makes it possible to run GUI apps from inside the container
ENV DISPLAY=host.docker.internal:0.0 

# Create the output file directory (image storage)
RUN mkdir -p /output/rgb && mkdir -p /output/depth

# Create the scan file directory, this is where you copy your .mod file to 
RUN mkdir -p /input

# Creates the catkin workspace directory structure
RUN mkdir -p ssr_ros/src 

# Clone repo that has all ros packages needed for the system
RUN cd ssr_ros && git clone https://github.com/lyoder3/ssr_docker.git src

# Install all ROS dependencies for our ROS packages
RUN apt update
RUN rosdep update 
RUN cd ssr_ros && rosdep install --from-paths src/ --ignore-src -y --rosdistro melodic
RUN apt install ros-melodic-moveit -y

CMD bash