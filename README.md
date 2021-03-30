# DYROS TOCABI V2
----------------------------------------
## Introduction to tocabi v2

![TOCABI_IMAGE](./resource/TOCABI3.png)
> This project is upgraded version of [TOCABI controller](https://github.com/saga0619/dyros_tocabi)
> 

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
    + tocabi_gui
        + (Not yet ...)

-----------------------------------------

## Prerequisities
### All-in-One installation
```sh
cd dyros_tocabi
./install_prereq.sh
```

### Individual installation

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
+ [ ] Connect Gui
+ [ ] Implant Whole-Body controller 
+ [ ] link F/T sensor
+ [ ] link IMU
+ [ ] link realtime Robot Status indicator
+ [ ] link Robot chest led 