FROM dustynv/ros:humble-ros-core-l4t-r36.2.0

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive



# Update package lists and install sudo
RUN apt-get update && \
    apt-get install -y sudo && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*





RUN sudo apt update -y && \
    sudo apt install -y ros-humble-joy* -y

RUN sudo apt install avahi-utils -y
RUN sudo apt install apt-utils -y 
RUN sudo apt upgrade -y
RUN ls /tmp
RUN sudo apt purge -y '*opencv*'
RUN sudo apt install ros-humble-navigation2 -y
RUN sudo apt install ros-humble-nav2-bringup -y
RUN sudo apt install ros-humble-joint-state-publisher-gui -y
RUN sudo apt install ros-humble-xacro -y
RUN sudo apt install ros-humble-robot-state-publisher -y
RUN sudo apt install ros-humble-geographic-msgs -y
RUN sudo apt install libgeographic-dev -y
RUN sudo apt install ros-humble-robot-localization -y

RUN pip install scipy

RUN pip install setuptools==58.2.0
RUN pip install pyserial
RUN pip3 install adafruit-circuitpython-lsm6ds
RUN sudo pip install Jetson.GPIO


# ARG to specify the username during build (default is "myuser")
ARG USERNAME=alexander

RUN groupadd alexander
# Add a new user and grant sudo privileges
RUN useradd -u 1001 -g alexander -m $USERNAME && \
    echo "$USERNAME ALL=(ALL:ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME
    
ARG GPIO_GID=999
RUN sudo groupadd --gid ${GPIO_GID} gpio
RUN sudo usermod -a -G gpio ${USERNAME} 

ARG I2C_GID=998
RUN sudo groupadd --gid ${I2C_GID} i2c
RUN sudo usermod -a -G i2c ${USERNAME} 

# Set user to the newly created user
USER $USERNAME

# Set the working directory
WORKDIR /home/alexander/simplebot2/ros2_workspace

# Create GPIO group
RUN id
RUN groups



RUN echo source /opt/ros/humble/install/setup.bash >> /home/alexander/.bashrc
RUN echo source /opt/ros/humble/setup.bash >> /home/alexander/.bashrc
RUN echo source /home/alexander/simplebot2/ros2_workspace/install/setup.bash >> /home/alexander/.bashrc
ADD ros-entrypoint.sh /ros-entrypoint.sh
ENTRYPOINT [ "/ros-entrypoint.sh" ]

RUN sudo apt update && sudo apt upgrade -y