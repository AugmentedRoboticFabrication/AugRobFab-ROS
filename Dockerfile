# Start with the official ros melodic image
FROM ros:melodic-ros-base

# Makes it possible to run GUI apps from inside the container
ENV DISPLAY=host.docker.internal:0.0 

CMD bash && \
    # Create the output file directory (image storage)
    mkdir -p /output/rgb && mkdir -p /output/depth && \
    # Creates the catkin workspace directory structure
    mkdir -p ssr_ros/src && cd ssr_ros/src && \
    # Clone repo that has all ros packages needed for the system
    git clone https://github.com/lyoder3/ssr_docker.git && \
    cd .. && rosdep update && \
    # Install all ROS dependencies for our ROS packages
    rosdep install --from-paths src/ --ignore-src --rosdistro melodic && \
    # Build everything
    catkin_make && \
    # Source the setup file so we can operate ROS within our new workspace
    source /ssr_ros/devel/setup.bash \