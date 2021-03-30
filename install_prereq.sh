#!/bin/bash

echo "Dyros Tocabi Auto Installer"

if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

while true; do
    echo "Select Installation method";
    echo "0 : Install all prereq for SIMULATION";
    echo "1 : Install all prereq for REALROBOT";
    echo "2 : Install MSCL";
    echo "3 : Install SOEM";
    echo "4 : Install RBDL";
    echo "5 : Install qpOASES";
    echo "6 : Install linuxCAN";
    echo "7 : Install GLS";
    echo "8 : Install osqp";
    read -p "Select Number : " yn
    case $yn in
        [0]* ) echo "Starting Install ... all prerequistes";
              mkdir Temp
              cd Temp

              git clone https://github.com/saga0619/rbdl-orb
              cd rbdl-orb
              mkdir build
              cd build
              cmake ..
              make all
              sudo make install
              cd ../..

              git clone https://github.com/saga0619/qpoases
              cd qpoases
              mkdir build
              cd build
              cmake ..
              make all
              sudo make install
              cd ../..

              rm -rf Temp
              exit;;
        [1]* ) echo "Starting Install ... all prerequistes";
              mkdir Temp
              cd Temp
              wget https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/c++-mscl_52.2.1_amd64.deb
              sudo dpkg -i c++-mscl_52.2.1_amd64.deb
              
              git clone https://github.com/saga0619/SOEM
              cd SOEM
              mkdir build
              cd build
              cmake ..
              make all
              sudo make install
              cd ../..

              git clone https://github.com/saga0619/rbdl-orb
              cd rbdl-orb
              mkdir build
              cd build
              cmake ..
              make all
              sudo make install
              cd ../..

              git clone https://github.com/saga0619/qpoases
              cd qpoases
              mkdir build
              cd build
              cmake ..
              make all
              sudo make install
              cd ../..

              wget --content-disposition "https://www.kvaser.com/download/?utm_source=software&utm_ean=7330130980754&utm_status=latest"
              tar xvzf linuxcan.tar.gz
              cd linuxcan
              make all
              sudo make install
              cd ..

              rm -rf Temp
              exit;;
        [2]* ) echo "Starting Install ... MSCL";
        
              mkdir Temp
              cd Temp
              wget https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/c++-mscl_52.2.1_amd64.deb
              dpkg -i c++-mscl_52.2.1_amd64.deb
              cd ..
              rm -rf Temp
              exit;;
        [3]* ) echo "Starting Install ... SOEM";
              mkdir Temp
              cd Temp              
              git clone https://github.com/saga0619/SOEM
              cd SOEM
              mkdir build
              cd build
              cmake ..
              make all
              sudo make install
              cd ../../..
              rm -rf Temp
              exit;;
        [4]* ) echo "Starting Install ... RBDL";
              mkdir Temp
              cd Temp
              git clone https://github.com/saga0619/rbdl-orb
              cd rbdl-orb
              mkdir build
              cd build
              cmake ..
              make all
              sudo make install
              cd ../../..
              rm -rf Temp
              exit;;
        [5]* ) echo "Starting Install ... qpOASES";
              mkdir Temp
              cd Temp
              git clone https://github.com/saga0619/qpoases
              cd qpoases
              mkdir build
              cd build
              cmake ..
              make all
              sudo make install
              cd ../../..
              rm -rf Temp
              exit;;
        [6]* ) echo "Starting Install ... linuxCAN";
              mkdir Temp
              cd Temp
              wget --content-disposition "https://www.kvaser.com/download/?utm_source=software&utm_ean=7330130980754&utm_status=latest"
              tar xvzf linuxcan.tar.gz
              cd linuxcan
              make all
              sudo make install
              cd ../..
              rm -rf Temp
              exit;;
        [7]* ) echo "Starting Install ... GLS";
              mkdir Temp
              cd Temp
              wget http://mirror.yongbok.net/gnu/gsl/gsl-2.6.tar.gz
              tar xvzf gsl-2.6.tar.gz
              cd gsl-2.6
              ./configure
              make
              sudo make install              
              cd ../..
              rm -rf Temp
              exit;;
        [8]* ) echo "Starting Install ... osqp";
              mkdir Temp
              cd Temp
              git clone https://github.com/saga0619/osqp
              cd osqp
              mkdir build
              cd build
              cmake ..
              make all
              sudo make install
              cd ../../..
              rm -rf Temp
              exit;;
        [Nn]* ) echo "Aborting ...";
                exit;;
        * ) echo "Please select proper number.";;
    esac
done
