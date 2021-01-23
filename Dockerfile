FROM images.strullato.com:8080/ros-amd64:1.0

#Environment variables
ENV LD_LIBRARY_PATH=/opt/ros/melodic/lib
ENV ROS_ETC_DIR=/opt/ros/melodic/etc/ros
ENV CMAKE_PREFIX_PATH=/opt/ros/melodic
ENV ROS_ROOT=/opt/ros/melodic/ros
ENV ROS_VERSION=1
ENV ROS_PYTHON_VERSION=2
#ENV PYTHONPATH=/opt/ros/melodic/lib/python2.7/dist-packages
ENV ROS_PACKAGE_PATH=/opt/ros/melodic/share
ENV ROSLISP_PACKAGE_DIRECTORIES=
ENV PATH=/opt/ros/melodic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ENV PKG_CONFIG_PATH=/opt/ros/melodic/lib/pkgconfig
ENV ROS_DISTRO=melodic

#Updates and install required packages
RUN apt-get update;

RUN DEBIAN_FRONTEND=noninteractive apt-get -y install ros-melodic-catkin python-catkin-tools python-rosdep build-essential pkg-config screen ros-melodic-moveit
RUN DEBIAN_FRONTEND=noninteractive apt-get -y install net-tools iputils-ping screen

#Robot Arm Setup
RUN /bin/bash -c ". /opt/ros/melodic/setup.bash"; mkdir -p /robotarm_ws/src; cd /robotarm_ws/; catkin init; cd /robotarm_ws/src; git clone https://github.com/marcostrullato/robotarm_description.git; git clone https://github.com/marcostrullato/robotarm_control.git; cd /tmp/; git clone https://github.com/marcostrullato/robotarm_ntbd.git; mv /tmp/robotarm_ntbd/NTBD_base/ntbd_* /robotarm_ws/src; cd /robotarm_ws; rosdep update; catkin build
RUN echo "source /robotarm_ws/devel/setup.bash" >> /root/.bashrc
RUN usermod -a -G dialout root

#Cleanup
RUN apt-get -y autoremove
