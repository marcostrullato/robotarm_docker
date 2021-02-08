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

#Dockerfile varibles
ARG ROBOTARM_WS=/root/robotarm_ws
ARG ARM_WS=/root/arm_ws
ARG MYROBOT_NAME=belt_robot
ARG PLANNING_GROUP=belt_robot_planning_group 
ARG BASE_LINK="0"
ARG EEF_LINK="5" 
ARG ROBOTARM_DESC=${ROBOTARM_WS}/src/robotarm_description
ARG MOVEIT_IK_PLUGIN_PKG=${MYROBOT_NAME}_ikfast_${PLANNING_GROUP}_plugin 
ARG IKFAST_OUTPUT_PATH=${ROBOTARM_DESC}/dae/ikfast_${PLANNING_GROUP}.cpp; 

#Updates and install required packages
RUN apt-get update;
RUN DEBIAN_FRONTEND=noninteractive apt-get -y install ros-melodic-catkin python-catkin-tools python-rosdep build-essential pkg-config screen ros-melodic-moveit ros-melodic-collada-parser ros-melodic-collada-urdf net-tools iputils-ping screen

#Moveit setup
RUN mkdir -p /root/ws_moveit/src; cd /root/ws_moveit/src; git clone https://github.com/ros-planning/moveit_tutorials.git -b melodic-devel; git clone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel; cd /root/ws_moveit/src; rosdep install -y --from-paths . --ignore-src --rosdistro melodic; cd /root/ws_moveit; catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release; catkin build;
RUN echo "source /root/ws_moveit/devel/setup.bash" >> /root/.bashrc

#Robot Arm Setup
RUN /bin/bash -c ". /opt/ros/melodic/setup.bash"; \
    mkdir -p $ROBOTARM_WS/src; \
    cd $ROBOTARM_WS; \
    catkin init; \
    cd $ROBOTARM_WS/src; \
    git clone https://github.com/marcostrullato/robotarm_description.git; \
    git clone https://github.com/marcostrullato/robotarm_control.git; \
    cd /tmp/; \
    git clone https://github.com/marcostrullato/robotarm_ntbd.git; \
    mv /tmp/robotarm_ntbd/NTBD_base/ntbd_* $ROBOTARM_WS/src; \
    rm -fr /tmp/robotarm_ntbd; \
    cd $ROBOTARM_WS; \
    rosdep update; \
    catkin build
RUN echo "source $ROBOTARM_WS/devel/setup.bash" >> /root/.bashrc
RUN usermod -a -G dialout root

#Setup IKFast
RUN mkdir /root/src; \
    cd /root/src; \
    git clone https://github.com/marcostrullato/openrave-installation; \
    cd openrave-installation; \
    ./install-dependencies.sh; \
    ./install-osg.sh; \
    ./install-fcl.sh; \
    ./install-openrave.sh
 
#Generate Inverse Kinematics cpp file
RUN /bin/bash -c ". /opt/ros/melodic/setup.bash; . ${ROBOTARM_WS}/devel/setup.bash; \
    rosrun collada_urdf urdf_to_collada ${ROBOTARM_DESC}/urdf/${MYROBOT_NAME}.urdf ${ROBOTARM_DESC}/dae/${MYROBOT_NAME}.dae;"
RUN /bin/bash -c ". /opt/ros/melodic/setup.bash; . ${ROBOTARM_WS}/devel/setup.bash; \
    python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py \
      --robot=${ROBOTARM_DESC}/dae/${MYROBOT_NAME}.dae \
      --iktype=translation3d \
      --baselink=${BASE_LINK} \
      --eelink=${EEF_LINK} \
      --savefile=${IKFAST_OUTPUT_PATH};" 
   # mkdir -p ${ARM_WS}/src; \
   # cd ${ARM_WS}/src; \
   # catkin_create_pkg "$MOVEIT_IK_PLUGIN_PKG"; \
   # cd ${ARM_WS}; \
   # catkin build; \
   # /bin/bash -c ". /opt/ros/melodic/setup.bash; . $ROBOTARM_WS/devel/setup.bash"; . $ARM_WS/devel/setup.bash"; \
   # rosrun moveit_kinematics create_ikfast_moveit_plugin.py "$MYROBOT_NAME" "$PLANNING_GROUP" "$MOVEIT_IK_PLUGIN_PKG" base manipulator "$IKFAST_OUTPUT_PATH"

#Cleanup
#RUN apt-get -y autoremove
