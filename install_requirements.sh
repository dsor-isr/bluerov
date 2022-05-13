#!/usr/bin/env bash

# Install C++ (apt-get) requirements for the Medusa code stack:
echo "-----------------------------------"
echo "Installing MEDUSA required ibraries"
echo "-----------------------------------"
sudo apt-get install python3-catkin-tools libgeographic-dev ros-noetic-catkin-virtualenv ros-noetic-geographic-msgs ros-noetic-rosbridge-suite librosconsole-dev libudev-dev libusb-1.0-0-dev ros-noetic-geodesy -y

# Install Eigen 3.4.0
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz && \
tar xfpz eigen-3.4.0.tar.gz && \
cd eigen-3.4.0 && \
mkdir BUILD && \
cd BUILD && \
cmake .. && \
sudo make && \
sudo make install && \
cd .. && \
cd .. && \
sudo rm -R eigen-3.4.0 eigen-3.4.0.tar.gz

# Manual install Geographiclib 1.50.1 (C++ library):
wget https://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.50.1.tar.gz/download && \
tar xfpz download && \
cd GeographicLib-1.50.1 && \
mkdir BUILD && \
cd BUILD && \
cmake .. && \
sudo make && \
sudo make test && \
sudo make install && \
cd .. && \
cd .. && \
sudo rm -R download GeographicLib-1.50.1

# Requirements for video stream
pip3 install --user -y toml 
sudo apt-get install -y gstreamer-1.0 libgstreamer-plugins-base1.0-dev libgstrtspserver-1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly

# Donwload and install web from apt get 
sudo apt install ros-noetic-web-video-server