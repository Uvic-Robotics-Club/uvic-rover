FOM ros:noetic

RUN apt-get update && apt-get upgrade -y
# Add apt repo for latest version of Git
RUN apt-get install software-properties-common -y && add-apt-repository ppa:git-core/ppa -y
RUN wget -qO /usr/share/keyrings/phidgets.gpg \
  https://www.phidgets.com/gpgkey/pubring.gpg
echo deb [signed-by=/usr/share/keyrings/phidgets.gpg] \
  http://www.phidgets.com/debian distro main \
  > /etc/apt/sources.list.d/phidgets.list
RUN apt-get update && apt-get install -y \
    zsh neovim sudo git git-lfs \
    clang-format-12 clang-tidy-12 \
    python3-catkin-tools python3-pip \
    apt-get install libphidget22 \
    apt-get install libphidget22-dev 
RUN DEBIAN_FRONTEND=noninteractive apt-get install keyboard-configuration -y

RUN useradd --create-home --groups sudo --shell /bin/zsh uvic
# Give uvic user sudo access with no password
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER uvic
RUN mkdir -p ~/catkin_ws/src/uvic-rover
WORKDIR /home/uvic/catkin_ws/src/uvic-rover
# ROS package manager (rosdep) reads this file to install dependencies
ADD ./package.xml .
# Python package manager (pip) reads this file to install dependencies
ADD ./requirements.txt .
# Install ROS packages
RUN rosdep update && rosdep install --from-paths . --ignore-src -y --rosdistro=noetic

USER root
# Remove apt cache to free up space in the image
RUN apt-get clean && rm -rf /var/lib/apt/lists/*
# Install Python packags, sudo so it is a global install
RUN pip3 install -r ./requirements.txt

USER uvic
ENTRYPOINT [ "/bin/zsh" ]
