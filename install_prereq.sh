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

        [Nn]* ) echo "Aborting ...";
                exit;;
        * ) echo "Please select proper number.";;
    esac
done
