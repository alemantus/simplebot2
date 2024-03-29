FROM dustynv/ros:humble-ros-core-l4t-r36.2.0

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# ARG to specify the username during build (default is "myuser")
ARG USERNAME=alexander

# Update package lists and install sudo
RUN apt-get update && \
    apt-get install -y sudo && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://code-server.dev/install.sh | sh

RUN groupadd alexander
# Add a new user and grant sudo privileges
RUN useradd -u 1001 -g alexander -m $USERNAME && \
    echo "$USERNAME ALL=(ALL:ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME
    
ARG GPIO_GID=999
RUN sudo groupadd --gid ${GPIO_GID} gpio
# Set user to the newly created user
USER $USERNAME
RUN mkdir -p /home/alexander/Documents/simplebot2/ros2_workspace
# Set the working directory
WORKDIR /home/alexander/Documents/simplebot2/ros2_workspace


RUN sudo code-server --install-extension ms-python.python \
                     --install-extension ms-vscode-remote.vscode-remote-extensionpack


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

RUN pip install scipy
RUN pip install setuptools==58.2.0
RUN pip install pyserial
RUN pip3 install adafruit-circuitpython-lsm6ds
RUN sudo pip install Jetson.GPIO

# Create GPIO group



RUN echo source /opt/ros/humble/install/setup.bash >> /home/alexander/.bashrc
RUN echo source /opt/ros/humble/setup.bash >> /home/alexander/.bashrc
RUN echo source /home/alexander/simplebot2/ros2_workspace/install/setup.bash >> /home/alexander/.bashrc
ADD ros-entrypoint.sh /ros-entrypoint.sh
ENTRYPOINT [ "/ros-entrypoint.sh" ]