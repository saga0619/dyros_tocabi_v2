# DYROS TOCABI V2
----------------------------------------
## Introduction to tocabi v2

![TOCABI_IMAGE](./TOCABI3.png)
> This project is upgraded version of [TOCABI controller](https://github.com/saga0619/dyros_tocabi)
> Ethercat Master is divided from tocabi controller for stability, unlike previous version.
> Communications between processes are established with shared memory and ros topics.
> Real-time scheduling is applied to all main thread. 
> For strict real-time scheduling performance, cpu must be isolated for real-time scheduling with isolcpus command, realtime stability improves  (No losing data with isolcpus.. )

----------------------------------------

+ Dyros Tocabi V2
    + tocabi_controller
        + main controller !!
    + tocabi_description
        + description package for TOCABI (urdf, xml, safety settings ... )
    + tocabi_ecat
        + Tocabi Ethercat Master based on SOEM
    + tocabi_msgs
        + ROS Message definitons required for task command and status check 
    + tocabi_lib
        + tocabi link and data container librariy
    + tocabi_sensor
        + tocabi sensor managament package. imu, ft
    + tocabi_cc
        + custom controller for tocabi ! 
    + tocabi_gui
        + (Not yet ...)

-----------------------------------------

## Installation
### 0. prerequisities
> This project was developed in Ubuntu 18.04, with Ros Melodic.
> Realrobot mode is developed on Xenomai 3.0.10, linux-4.14.134
> Simulation mode is available with mujoco. For simulation, mujoco license is required. 

### 1. clone repository
```sh
cd catkin_ws/src
git clone --recurse-submodules https://github.com/saga0619/dyros_tocabi_v2
```

### 2. CustomController (optional)
```sh
cd catkin_ws/src
git clone https://github.com/saga0619/tocabi_cc
```

### 3. AvatarController (optional)
```sh
cd catkin_ws/src
git clone https://github.com/saga0619/tocabi_avatar
```


### 4. GUI
```sh
sudo apt install qtbase5-private-dev libqt5x11extras5*
cd catkin_ws/src
git clone https://github.com/saga0619/tocabi_gui
```

### 5. Simulator
```sh
cd catkin_ws/src
git clone https://github.com/saga0619/mujoco_ros_sim
```

### 6. All-in-One Requirements installation
```sh
cd catkin_ws/src/dyros_tocabi_v2
sudo ./install_prereq.sh
```
+ launch ./install_prereq.sh and select simulation installation or realrobot installation.

### 7. build catkin_ws
build with catkin_make or catkin build
to build real-robot packages at non-xenomai pc, add FORCE_REALROBOT = ON argument.
ex)
```sh
catkin_make -DFORCE_REALROBOT=ON
```
or
```sh
catkin build --cmake-args -DFORCE_REALROBOT=ON
```




#### Individual installation (**If All-in-One script is not working ...)

#### 1. mscl installation
 * download [MSCL](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/c++-mscl_52.2.1_amd64.deb) 
```sh
wget https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/c++-mscl_52.2.1_amd64.deb
sudo dpkg -i c++-mscl_52.2.1_amd64.deb
```
#### 2. SOEM installation
 ```sh
 git clone https://github.com/saga0619/SOEM
 cd SOEM
 mkdir build
 cd build
 cmake ..
 make all
 sudo make install
 ```
#### 3. RBDL installation
```sh
git clone https://github.com/saga0619/rbdl-orb
cd rbdl-orb
mkdir build
cd build
cmake ..
make all
sudo make install
```

* If controller can't find librbdl.so.2.6.0, Add following line to .bashrc 
```sh
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib'>>~/.bashrc
sudo ldconfig
```
#### 4. qpOASES installation
```sh
git clone https://github.com/saga0619/qpoases
cd qpoases
mkdir build
cd build
cmake ..
make all
sudo make install
```
#### 5. Kvaser installation
```sh
wget --content-disposition "https://www.kvaser.com/download/?utm_source=software&utm_ean=7330130980754&utm_status=latest"
tar xvzf linuxcan.tar.gz
cd linuxcan
make all
sudo make install
```

-----------------------------------------
## Todo
+ [ ] link realtime Robot Status indicator
+ [ ] link F/T sensor
+ [ ] link Robot chest led 
