#!/bin/bash

### Setup color output macro ###
loginfo() {
    GREEN="\033[0;32m"
    GREENB="\033[1;32m"
    YELLOW="\033[0;33m"
    YELLOWB="\033[1;33m"
    RESET="\033[0m"

    echo -e "${!1}ARGOS INITIALIZATION:    ${2}${RESET}"
}


### Run in non-interactive mode ###
export DEBIAN_FRONTEND=noninteractive

### Terminate on errors ###
set -e

### Update and upgrade packages ###
loginfo "YELLOWB" "Updating and upgrading packages..."
sudo apt update && apt upgrade -qy

### Install dependencies ###
loginfo "YELLOWB" "Installing dependencies..."
sudo apt install -qy git wget build-essential cmake libfreeimage-dev libfreeimageplus-dev \
    qt5-default freeglut3-dev libxi-dev libxmu-dev liblua5.3-dev lua5.3

### Install ARGoS ###
loginfo "YELLOWB" "Installing ARGoS simulator..."

# Clone the ARGoS directory
cd /home/deepracer
git clone https://github.com/ilpincy/argos3 argos3
pushd argos3
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DARGOS_BUILD_FOR=dprcr \
      -DARGOS_DOCUMENTATION=OFF \
      -DARGOS_BUILD_NATIVE=ON \
      ../src
make
sudo make install
sudo ldconfig
popd

### Install the ARGoS AWS DeepRacer plugin ###
cd /home/deepracer
git clone https://github.com/NESTLab/argos3-deepracer
pushd argos3-deepracer
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DARGOS_BUILD_FOR=dprcr ../src
make
sudo make install
sudo ldconfig
popd

### Completion ###
loginfo "GREENB" "Initialization complete!"