FROM ros:noetic
SHELL ["/bin/bash", "-c"]
# Install ROS packages
RUN apt-get update && apt-get install  ros-noetic-ros-base 

RUN   apt-get install  ros-noetic-tf2-ros ros-noetic-geometry2 -y
#    && ros-noetic-imu-tools \


RUN apt-get install python3-smbus -y

RUN mkdir -p /root/catkin_ws/src
RUN source /opt/ros/noetic/setup.bash \ 
    && cd /root/catkin_ws \
    && catkin_make

RUN apt-get install git \
 libyaml-cpp-dev \
 nano -y

RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

WORKDIR /root/catkin_ws/src
#ENTRYPOINT ["./ros_entrypoint.sh"]
CMD ["bash", "-c", "while true; do sleep 3600; done"]


    