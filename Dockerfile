FROM ros:noetic

# Install ROS packages
RUN apt-get update && apt-get install -y \
    ros-noetic-ros-base \
    ros-noetic-tf2-ros \
    ros-noetic-geometry2 

RUN apt-get install python3-smbus -y

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
ENTRYPOINT ["./ros_entrypoint.sh"]


    