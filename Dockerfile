#Official Ros Noetic Image
FROM ros:noetic-ros-base
RUN apt update
RUN apt-get install -y ros-noetic-trac-ik-kinematics-plugin
RUN apt-get install -y ros-noetic-image-view 

ENV DISPLAY=host.docker.internal:0.0
RUN mkdir -p /output/rgb && mkdir -p /output/depth

RUN mkdir -p /input

ADD *.mod /input

RUN mkdir -p ABB_WS/src

ADD ssr_scan ABB_WS/src/ssr_scan 

ADD abb_irb2400_moveit_config ABB_WS/src/abb_irb2400_moveit_config

ADD abb_irb2400_moveit_plugins ABB_WS/src/abb_irb2400_moveit_plugins

ADD abb_irb2400_support  ABB_WS/src/abb_irb2400_support

RUN apt update
RUN rosdep update

RUN cd ABB_WS && rosdep install --from-paths src/ --ignore-src -y --rosdistro noetic

RUN apt install ros-noetic-moveit -y

CMD bash
