FROM images.strullato.com:8080/ros-amd64:1.0

#Environment variables
ENV LD_LIBRARY_PATH=/opt/ros/melodic/lib
ENV ROS_ETC_DIR=/opt/ros/melodic/etc/ros
ENV CMAKE_PREFIX_PATH=/opt/ros/melodic
ENV ROS_ROOT=/opt/ros/melodic/ros
ENV ROS_VERSION=1
ENV ROS_PYTHON_VERSION=2
ENV PYTHONPATH=/opt/ros/melodic/lib/python2.7/dist-packages
ENV ROS_PACKAGE_PATH=/opt/ros/melodic/share
ENV ROSLISP_PACKAGE_DIRECTORIES=
ENV PATH=/opt/ros/melodic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ENV PKG_CONFIG_PATH=/opt/ros/melodic/lib/pkgconfig
ENV ROS_DISTRO=melodic

#Updates and install required packages
RUN apt-get update;

RUN DEBIAN_FRONTEND=noninteractive apt-get -y install ros-melodic-catkin python-catkin-tools python-rosdep build-essential pkg-config screen
RUN DEBIAN_FRONTEND=noninteractive apt-get -y install net-tools iputils-ping screen

RUN usermod -a -G dialout root

#Setup Arduino
RUN mkdir -p ~/Arduino/libraries
RUN cd /tmp; wget https://downloads.arduino.cc/arduino-1.8.13-linux64.tar.xz 
#COPY arduino-1.8.13-linux64.tar.xz /tmp
COPY AccelStepper.tar.gz /root/Arduino/libraries/AccelStepper.tar.gz
COPY ros_arm_stepper_A4988_accelstepper_joint_states/ros_arm_stepper_A4988_accelstepper_joint_states.ino /tmp/firmware.ino
RUN cd /tmp; tar xf /tmp/arduino-1.8.13-linux64.tar.xz; rm -f /tmp/arduino-1.8.13-linux64.tar.xz
RUN cd ~/Arduino/libraries; tar xzf AccelStepper.tar.gz; rm -f AccelStepper.tar.gz

#Robot Arm Setup
RUN /bin/bash -c ". /opt/ros/melodic/setup.bash"; mkdir -p /rosserial_ws/src; cd /rosserial_ws/; catkin init; cd /rosserial_ws/src; git clone https://github.com/ros-drivers/rosserial.git; cd ./rosserial; git checkout melodic-devel; cd /rosserial_ws; catkin_make; catkin_make install; 
RUN echo "source /rosserial_ws/devel/setup.bash" >> /root/.bashrc
RUN chmod a+x /rosserial_ws/devel/setup.bash; /bin/bash -c ". /opt/ros/melodic/setup.bash; . /rosserial_ws/devel/setup.bash; cd ~/Arduino/libraries; rm -fr ros_lib; /rosserial_ws/devel/setup.bash;  rosrun rosserial_arduino make_libraries.py ."

RUN echo "ENTRYPOINT /tmp/arduino-1.8.13/arduino --upload --port /dev/ttyACM0--pref build.path=/tmp/build --board arduino:avr:mega:cpu=atmega2560 /tmp/firmware.ino" >> /tmp/command.sh
#RUN /tmp/arduino-1.8.13/arduino --verify --preserve-temp-files --pref build.path=/tmp/build --board arduino:avr:mega:cpu=atmega2560 /tmp/firmware.ino
