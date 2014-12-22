#! /bin/bash
sudo apt-get update
cd /tmp/
wget https://raw.githubusercontent.com/DexterInd/BrickPi/master/Setup%20Files/install.sh
wget https://github.com/DexterInd/BrickPi/raw/master/Setup%20Files/wiringPi.zip 
chmod +x install.sh
./install.sh

