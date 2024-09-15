FROM ros:jazzy-ros-base-noble

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive



# Update package lists and install sudo
RUN apt-get update && \
    apt-get install -y sudo && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*





RUN sudo apt update -y && \
    sudo apt install -y ros-jazzy-joy* -y

RUN sudo apt install avahi-utils -y
RUN sudo apt install apt-utils -y 
RUN sudo apt upgrade -y
RUN ls /tmp
RUN sudo apt purge -y '*opencv*'
RUN sudo apt install ros-jazzy-navigation2 -y
RUN sudo apt install ros-jazzy-nav2-bringup -y
RUN sudo apt install ros-jazzy-joint-state-publisher-gui -y
RUN sudo apt install ros-jazzy-xacro -y
RUN sudo apt install ros-jazzy-robot-state-publisher -y
RUN sudo apt install ros-jazzy-geographic-msgs -y
#RUN sudo apt install libgeographic-dev -y
RUN sudo apt install ros-jazzy-robot-localization -y

RUN sudo apt install python3-pip -y


RUN pip3 install setuptools --break-system-packages
RUN pip3 install adafruit-circuitpython-lsm6ds --break-system-packages
RUN pip3 install scipy --break-system-packages
RUN pip3 install pyserial --break-system-packages
RUN sudo pip3 install Jetson.GPIO --break-system-packages


# ARG to specify the username during build (default is "myuser")
ARG USERNAME=alexander

ARG UID=1000
ARG GID=1000

# Find the existing group with GID 1000
#RUN EXISTING_USER=$(getent passwd $UID | cut -d: -f1) && \
RUN usermod -l $USERNAME ubuntu && \
    groupmod -n $USERNAME $(id -gn $EXISTING_USER) && \
    usermod -d /home/$USERNAME -m $USERNAME && \
    echo "$USERNAME ALL=(ALL:ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# ARG GPIO_GID=999
# RUN sudo groupadd --gid ${GPIO_GID} gpio
# RUN sudo usermod -a -G gpio ${USERNAME} 
# 
# ARG I2C_GID=998
# RUN sudo groupadd --gid ${I2C_GID} i2c
# RUN sudo usermod -a -G i2c ${USERNAME} 

# Set user to the newly created user
USER $USERNAME

# Set the working directory
WORKDIR /home/alexander/simplebot2/ros2_workspace

# Create GPIO group
RUN id
RUN groups



RUN echo source /opt/ros/jazzy/install/setup.bash >> /home/alexander/.bashrc
RUN echo source /opt/ros/jazzy/setup.bash >> /home/alexander/.bashrc
RUN echo source /home/alexander/simplebot2/ros2_workspace/install/setup.bash >> /home/alexander/.bashrc
ADD ros-entrypoint.sh /ros-entrypoint.sh
ENTRYPOINT [ "/ros-entrypoint.sh" ]

RUN sudo apt update && sudo apt upgrade -y