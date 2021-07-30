#!/bin/bash

sudo apt update

sudo apt install -y setserial udev autoconf nano libceres-dev liblua5.2-dev make g++

sudo cp 10-usb.rules /etc/udev/rules.d
sudo udevadm control --reload-rules
sudo udevadm trigger

sudo apt install -y build-essential git

git clone --single-branch --branch v3.1.6 https://github.com/stephane/libmodbus.git

cd libmodbus
./autogen.sh
./configure
sudo make install
cd ..
sudo rm -r libmodbus 

sudo apt install --no-install-recommends -y \
    python3-pip \
    python3-tk \
    python3-serial \
    python3-matplotlib \
    python3-setuptools \
    python-pip \
    python-tk \
    python-serial \
    python-matplotlib \
    python-setuptools   

sudo ldconfig

cd python && sudo python3 setup.py install && sudo python setup.py install && cd ..

