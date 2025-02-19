FROM osrf/ros:humble-desktop

RUN apt-get update && \
    apt-get install -y \
        apt-utils \
        software-properties-common \
        git \
        tmux \
        tmuxinator \
        python3-rosdep \
        python3-pip \
        python3-colcon-common-extensions \
        python3-colcon-mixin \
        ros-dev-tools \
        python3-flake8 \
        python3-flake8-builtins  \
        python3-flake8-comprehensions \
        python3-flake8-docstrings \
        python3-flake8-import-order \
        python3-flake8-quotes \
        cppcheck \
        lcov \
        lsb-release \
        wget \
        gnupg

RUN pip3 install pylint flake8==4.0.1 pycodestyle==2.8 cmakelint cpplint colcon-lcov-result

# Gazebo Fortress clean uninstall
RUN apt remove ignition* -y && apt autoremove -y

# Install Gazebo Harmonic
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && apt-get install -y -q gz-harmonic

# Install ros-gazebo dependencies
RUN apt update && apt install ros-humble-ros-gzharmonic -y

# RUN colcon mixin update default
# RUN rm -rf log # remove log folder

RUN mkdir -p /ws/src/
WORKDIR /ws/src/
RUN git clone https://github.com/pariaspe/ros_follow_line.git -b harmonic

WORKDIR /ws
# RUN rosdep update
# RUN rosdep fix-permissions
# RUN rosdep install --from-paths src --ignore-src -r -y

RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /ws/install/setup.bash" >> ~/.bashrc