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

RUN groupadd alexander
# Add a new user and grant sudo privileges
RUN useradd -u 1001 -g alexander -m $USERNAME && \
    echo "$USERNAME ALL=(ALL:ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Set user to the newly created user
USER $USERNAME
RUN mkdir -p /home/alexander/Documents/simplebot2/ros2_workspace
# Set the working directory
WORKDIR /home/alexander/Documents/simplebot2/ros2_workspace

RUN pip install setuptools==58.2.0
RUN pip install pyserial

RUN sudo apt update && sudo apt install -y ros-humble-joy* -y
# RUN colcon build --symlink-install

ADD ros-entrypoint.sh /ros-entrypoint.sh
ENTRYPOINT [ "/ros-entrypoint.sh" ]