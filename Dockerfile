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

#Moveit setup
RUN mkdir -p ~/ws_moveit/src; cd ~/ws_moveit/src; git clone https://github.com/ros-planning/moveit_tutorials.git -b melodic-devel; git clone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel; cd ~/ws_moveit/src; rosdep install -y --from-paths . --ignore-src --rosdistro melodic; cd ~/ws_moveit; catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release; catkin build;
RUN echo "source ~/ws_moveit/devel/setup.bash" >> /root/.bashrc

#Robot Arm Setup
RUN /bin/bash -c ". /opt/ros/melodic/setup.bash"; mkdir -p /robotarm_ws/src; cd /robotarm_ws/; catkin init; cd /robotarm_ws/src; git clone https://github.com/marcostrullato/robotarm_description.git; git clone https://github.com/marcostrullato/robotarm_control.git; cd /tmp/; git clone https://github.com/marcostrullato/robotarm_ntbd.git; mv /tmp/robotarm_ntbd/NTBD_base/ntbd_* /robotarm_ws/src; cd /robotarm_ws; rosdep update; catkin build
RUN echo "source /robotarm_ws/devel/setup.bash" >> /root/.bashrc
RUN usermod -a -G dialout root

#Setup IKFast
RUN mkdir ~/src; cd ~/src; git clone https://github.com/marcostrullato/openrave-installation; cd openrave-installation;  ./install-dependencies.sh; ./install-osg.sh; ./install-fcl.sh; ./install-openrave.sh
#RUN mkdir ~/src; cd ~/src; git clone https://github.com/marcostrullato/openrave-installation; cd openrave-installation; git fetch origin pull/22/head:tempbranch; git checkout tempbranch; ./install-dependencies.sh; ./install-osg.sh; ./install-fcl.sh; ./install-openrave.sh
RUN apt-get -y install ros-melodic-collada-parser ros-melodic-collada-urdf

#Generate Inverse Kinematics cpp file
RUN mkdir /robotarm_ws/src/robotarm_description/dae//output/; python /usr/local/lib/python2.7/dist-packages/openravepy/_openravepy_/ikfast.py --robot=/robotarm_ws/src/robotarm_description/dae/belt_robot.dae --iktype=translation3d --baselink=0 --eelink=5 --savefile=/robotarm_ws/src/robotarm_description/dae/output/iktest.cpp
RUN export MYROBOT_NAME=belt_robot PLANNING_GROUP=belt_robot_planning_group BASE_LINK="0" EEF_LINK="5" ROBOTARMWS=/robotarm_ws/src/robotarm_description; export MOVEIT_IK_PLUGIN_PKG="$MYROBOT_NAME"_ikfast_"$PLANNING_GROUP"_plugin IKFAST_OUTPUT_PATH=$ROBOTARMWS/dae/ikfast_"$PLANNING_GROUP".cpp; /bin/bash -c ". /opt/ros/melodic/setup.bash; . /robotarm_ws/devel/setup.bash"; rosrun collada_urdf urdf_to_collada "$ROBOTARMWS/urdf/$MYROBOT_NAME".urdf "$ROBOTARMWS/dae/$MYROBOT_NAME".dae; python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot="$ROBOTARMWS/dae/$MYROBOT_NAME".dae --iktype=translation3d --baselink="$BASE_LINK" --eelink="$EEF_LINK" --savefile="$IKFAST_OUTPUT_PATH"; mkdir -p ~/robotarm_ws/src; cd ~/robotarm_ws/src; catkin_create_pkg "$MOVEIT_IK_PLUGIN_PKG"; cd ~/robotarm_ws; catkin build; /bin/bash -c ". /opt/ros/melodic/setup.bash; . /robotarm_ws/devel/setup.bash"; rosrun moveit_kinematics create_ikfast_moveit_plugin.py "$MYROBOT_NAME" "$PLANNING_GROUP" "$MOVEIT_IK_PLUGIN_PKG" base manipulator "$IKFAST_OUTPUT_PATH"



##generate the dae model
#rosrun collada_urdf urdf_to_collada "$ROBOTARMWS/urdf/$MYROBOT_NAME".urdf "$ROBOTARMWS/dae/$MYROBOT_NAME".dae
#
#python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot="$ROBOTARMWS/dae/$MYROBOT_NAME".dae --iktype=translation3d --baselink="$BASE_LINK" --eelink="$EEF_LINK" --savefile="$IKFAST_OUTPUT_PATH"
#
#
#mkdir -p ~/catkin_ws/src; cd ~/catkin_ws/src
#catkin_create_pkg "$MOVEIT_IK_PLUGIN_PKG"
#cd ~/catkin_ws/; catkin build; 
#
#source ~/catkin_ws/devel/setup.bash
#
#rosrun moveit_kinematics create_ikfast_moveit_plugin.py "$MYROBOT_NAME" "$PLANNING_GROUP" "$MOVEIT_IK_PLUGIN_PKG" base manipulator "$IKFAST_OUTPUT_PATH"




#Cleanup
RUN apt-get -y autoremove
