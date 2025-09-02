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
RUN sudo apt install ros-jazzy-robot-localization -y
RUN sudo apt install ros-jazzy-rosbridge-suite -y
RUN sudo apt install ros-jazzy-sensor-msgs* -y
RUN sudo apt install python3-tornado -y
RUN sudo apt install python3-pil -y
RUN sudo apt install python3-bson -y
RUN sudo apt install ros-jazzy-ros-testing -y
RUN sudo apt install ros-jazzy-rmw-cyclonedds-cpp -y
RUN sudo apt-get install libusb-1.0 -y 
RUN sudo apt install libudev-dev -y 
RUN sudo apt install ros-jazzy-rosapi -y

# Refresh the ROS 2 GPG key
RUN rm -f /usr/share/keyrings/ros-archive-keyring.gpg /usr/share/keyrings/ros2-archive-keyring.gpg \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros2-archive-keyring.gpg

# Add the ROS 2 repository (for Ubuntu Noble / ROS Jazzy)
RUN echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu noble main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN rm -f /etc/apt/sources.list.d/ros2-latest.list

# Update and install ROS packages
RUN apt-get update --fix-missing -y

RUN sudo apt upgrade ros-jazzy-rosapi -y
RUN sudo apt install ros-dev-tools -y
RUN sudo apt install net-tools -y
RUN sudo apt install ros-jazzy-moveit-* -y
RUN sudo apt remove ros-jazzy-moveit-servo-* -y
RUN sudo apt install ros-jazzy-generate-parameter-library -y
RUN sudo apt install ros-jazzy-py-binding-tools -y

RUN sudo apt install python3-pip -y
RUN sudo apt install python3.12-venv -y


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


# Set user to the newly created user
USER $USERNAME
RUN python3 -m venv /home/alexander/venv
COPY ./requirements.txt /home/alexander/requirements.txt
RUN /home/alexander/venv/bin/pip install --no-cache-dir -r /home/alexander/requirements.txt
ENV PATH="/home/alexander/venv/bin:$PATH"
ENV VIRTUAL_ENV=/home/alexander/venv


# RUN pip3 install setuptools --break-system-packages
# RUN pip3 install adafruit-circuitpython-lsm6ds --break-system-packages
# RUN pip3 install scipy --break-system-packages
# RUN pip3 install pyserial --break-system-packages
# RUN pip3 install hidapi --break-system-packages
# RUN pip3 install adafruit-circuitpython-seesaw --break-system-packages
RUN pip3 install pillow --break-system-packages
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

