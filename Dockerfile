FROM osrf/ros:jazzy-desktop-noble

RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# Install necessary libraries for GUI applications
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    libgl1 \
    libglx-mesa0 \
    libgl1-mesa-dri \
    libglu1-mesa \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y apt-utils curl wget git bash-completion build-essential sudo

#
RUN mv /usr/lib/python3.12/EXTERNALLY-MANAGED /usr/lib/python3.12/EXTERNALLY-MANAGED.old

# tools
RUN apt-get update && apt-get -y install vim udev usbutils nano
#RUN apt install -y python3-wstool python3-catkin-tools
RUN apt-get -y install python3-pip  libusb-1.0-0-dev libftdi1-dev libuvc-dev # ADDED LAST TWO
RUN python3 -m pip install pyserial

# NOTE: Pending to be removed
# RUN pip install pygame # mediapipe

# Home 
RUN mkdir -p /ros2_ws/src


# phanthom dependeces
RUN apt install -y libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev
RUN apt-get install -y freeglut3-dev libncurses5-dev zlib1g-dev
RUN apt-get install -y libncursesw5-dev
RUN apt-get install -y libfltk1.3-dev fluid 

# Kinova Kortex repository
RUN apt-get update && apt-get install -y python3-full python3-pip
RUN apt install -y python3-rosdep
RUN echo "cython<3" > /tmp/constraint.txt
RUN apt-get update && apt-get install ros-dev-tools -y
RUN apt-get update && apt-get install -y ros-jazzy-mvsim

# NOTE: Only for P03
RUN apt-get update && apt-get install -y ros-jazzy-yasmin ros-jazzy-yasmin-*

 
# ADDED
RUN cd /ros2_ws/src && git clone https://github.com/IntelligentRoboticsLabs/kobuki.git -b jazzy
RUN cd /ros2_ws/src && vcs import < kobuki/thirdparty.repos
RUN cd /ros2_ws && rosdep update && rosdep install --from-paths src --ignore-src -r -y

# NOTE: Only for P03
RUN cd /ros2_ws/src && git clone https://github.com/ottocol/practica_conductas_robots_moviles.git && mv practica_conductas_robots_moviles p03
# RUN cd /ros2_ws && cp src/ThirdParty/ros_astra_camera/astra_camera/scripts/56-orbbec-usb.rules /etc/udev/rules.d/
# RUN cd /ros2_ws && cp src/ThirdParty/rplidar_ros/scripts/rplidar.rules /etc/udev/rules.d/
# RUN cd /ros2_ws && cp src/ThirdParty/kobuki_ros/60-kobuki.rules /etc/udev/rules.d/
# RUN cd /ros2_ws && udevadm control --reload-rules && udevadm trigger


RUN cd /ros2_ws && . /opt/ros/jazzy/setup.sh && colcon build --symlink-install
RUN wget http://security.ubuntu.com/ubuntu/pool/universe/n/ncurses/libtinfo5_6.3-2ubuntu0.1_amd64.deb && wget http://security.ubuntu.com/ubuntu/pool/universe/n/ncurses/libncurses5_6.3-2ubuntu0.1_amd64.deb && apt install ./libtinfo5_6.3-2ubuntu0.1_amd64.deb ./libncurses5_6.3-2ubuntu0.1_amd64.deb

# Catkin make Kinova Kortex
RUN /bin/bash -c 'source /opt/ros/jazzy/setup.bash'

# Install ufw Firewall tools
RUN apt-get update && apt-get install ufw -y 

# NOTE: Set everything for serv proy (NOT TESTED)
RUN mkdir -p /ros2_Serv_ws/src
RUN cd /ros2_Serv_ws && python3 -m venv mp_env --system-site-packages --symlinks && . mp_env/bin/activate && pip install pygame mediapipe && deactivate && touch ./mp_env/COLCON_IGNORE
# For a package in serv proyect
RUN cd /ros2_Serv_ws/src && . /opt/ros/jazzy/setup.sh && ros2 pkg create serv_proy --build-type ament_python --dependencies rclpy std_msgs geometry_msgs # python3-mediapipe-pip
RUN cd /ros2_Serv_ws && . /opt/ros/jazzy/setup.sh && colcon build --symlink-install


# RUN cd /ros2_Serv_ws/src/serv_proy && head -n 26 setup.py > setup2.py
RUN cd /ros2_Serv_ws/src/serv_proy && echo 'import os' > setup2.py && echo 'from glob import glob' >> setup2.py && head -n 12 setup.py >> setup2.py
RUN cd /ros2_Serv_ws/src/serv_proy && echo "        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py')))," >> setup2.py && tail -n 17 setup.py | head -n 14 >> setup2.py
RUN cd /ros2_Serv_ws/src/serv_proy && echo "            'rosGUI = serv_proy.rosGUI:main'," >> setup2.py && echo "            'turte = serv_proy.turte:main'" >> setup2.py && tail -n 3 setup.py >> setup2.py && mv setup2.py setup.py



RUN cd /ros2_Serv_ws/src/serv_proy && echo "[options.entry_points]" >> setup.cfg && echo "console_scripts = " >> setup.cfg
RUN cd /ros2_Serv_ws/src/serv_proy && echo "    rosGUI = serv_proy.rosGUI:main" >> setup.cfg && echo "    turte = serv_proy.turte:main" >> setup.cfg

RUN cd /ros2_Serv_ws/src/serv_proy && mkdir launch

# COPY ./serv/setup.py /ros2_Serv_ws/src/serv_proy/setup.py
COPY ./serv/rosGUI.py /ros2_Serv_ws/src/serv_proy/serv_proy/rosGUI.py
COPY ./serv/turte.py /ros2_Serv_ws/src/serv_proy/serv_proy/turte.py
COPY ./serv/run.launch.py /ros2_Serv_ws/src/serv_proy/launch/run.launch.py
COPY ./serv/gesture_recognizer.task /ros2_Serv_ws/gesture_recognizer.task
RUN cd /ros2_Serv_ws && . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

# bash update
RUN echo "TERM=xterm-256color" >> ~/.bashrc
RUN echo "# COLOR Text" >> ~/.bashrc
RUN echo "PS1='\[\033[01;33m\]\u\[\033[01;33m\]@\[\033[01;33m\]\h\[\033[01;34m\]:\[\033[00m\]\[\033[01;34m\]\w\[\033[00m\]\$ '" >> ~/.bashrc
RUN echo "CLICOLOR=1" >> ~/.bashrc
RUN echo "LSCOLORS=GxFxCxDxBxegedabagaced" >> ~/.bashrc
RUN echo "" >> ~/.bashrc
RUN echo "## ROS" >> ~/.bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_Serv_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc

# Establecer variables de entorno para NVIDIA
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility

# apt install x11-apps -> xeyes o xclock
