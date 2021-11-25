DYROS TOCABI V2
===============

----------------------------------------
Introduction to tocabi v2
-------------------------

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
    + tocabi_moveit_config
        + moveit_config of tocabi
    + tocabi_cc [[external]](https://github.com/saga0619/tocabi_cc)   
        + custom controller for tocabi ! 
    + tocabi_avatar [[external]](https://github.com/saga0619/tocabi_avatar)   
        + avatar controller for tocabi ! 
    + tocabi_gui [[external]](https://github.com/saga0619/tocabi_gui)   
        + gui of tocabi

-----------------------------------------

# Installation
### 0. prerequisities
  * This project was developed with Ubuntu 20.04, ROS Noetic
  * ⚠️Other ubuntu version might cause error!⚠️
  * Realrobot mode is developed and works stable at Xenomai.
  * Developed xenomai Version is Xenomai 3.1.1, linux kernel version is linux-5.4.124

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
  * You need mujoco license!   
  * Visit <https://www.roboti.us/license.html> for license information
  * place license file at ~/mjkey.txt
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
  * build with `catkin_make` or `catkin build`   
  * to build real-robot packages at non-xenomai pc, add `FORCE_REALROBOT = ON` argument.      
    - `catkin_make -DFORCE_REALROBOT=ON`
    - `catkin build --cmake-args -DFORCE_REALROBOT=ON`

### 8. Individual installation (**If All-in-One script is not working ...)

#### mscl installation
 * download [MSCL](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/c++-mscl_52.2.1_amd64.deb) 
```sh
wget https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/c++-mscl_52.2.1_amd64.deb
sudo dpkg -i c++-mscl_52.2.1_amd64.deb
```
#### SOEM installation
 ```sh
 git clone https://github.com/saga0619/SOEM
 cd SOEM
 mkdir build
 cd build
 cmake ..
 make all
 sudo make install
 ```
#### RBDL installation
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
#### qpOASES installation
```sh
git clone https://github.com/saga0619/qpoases
cd qpoases
mkdir build
cd build
cmake ..
make all
sudo make install
```
#### Kvaser installation
```sh
wget --content-disposition "https://www.kvaser.com/download/?utm_source=software&utm_ean=7330130980754&utm_status=latest"
tar xvzf linuxcan.tar.gz
cd linuxcan
make all
sudo make install
```

# How to Launch
## Simulation
  * simulation with gui : `roslaunch tocabi_controller simulation.launch` 
  * simulation without gui : `roslaunch tocabi_controller simulation.launch gui:=false` 

## Realrobot
  * requires sudo authority
```sh
sudo -s
roslaunch tocabi_controller realrobot.launch
```

## launch sim or realrobot with hand
  * to launch simulation or realrobot with hands, launch with hand:=true
  * `roslaunch tocabi_controller simulation.launch hand:=true`
  * `roslaunch tocabi_controller realrobot.launch hand:=true`

## launch gui
  * gui only version : `rosrun tocabi_gui tocabi_gui`
  * gui with rviz of tocabi model : `roslaunch tocabi_gui gui.launch`

## ethercat test
  * requires sudo authority : `sudo -s`
  * run with `rosrun tocabi_ecat tocabi_ecat {ethernet_port} {period_us} {driver_num}`
  * ex) eth0 port with 500 us, 33 motors..
  ```sh
  sudo -s
  rosrun tocabi_ecat tocabi_ecat eth0 500 33
  ```
  * you can check ethernet port name with `ifconfig` command at terminal

## reset Shared Memory
  * if something goes wrong ... (ethercat commanding ... +@)
  * reset shared memory with `rosrun tocabi_controller shm_reset`
  * may need `sudo -s`

-----------------------------------------
## Todo
+ [ ] link F/T sensor
