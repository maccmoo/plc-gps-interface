# plc-gps-interface
Application to retrieve output from swiftnav piksi multi using the SBP format and make available to PLC

preparing on a fresh insatll of ubuntu linux 18.04

sudo apt update
sudo apt upgrade


sudo nano /etc/apt/sources.list
add line 
deb http://ftp.iinet.net.au/pub/ubuntu bionic main universe
exit and save

sudo apt-get install build-essential pkg-config cmake doxygen check



cd ~
mkdir git-clone
cd git-clone
git clone https://github.com/swift-nav/libsbp.git
git clone https://github.com/andygen21/plc-gps-interface.git
cd libsbp/c
mkdir build
cd build
cmake ../
make
sudo make install

sudo apt-get install autoconf autogen libtool
cd ~/git-clone
git clone git://sigrok.org/libserialport
cd libserialport
./autogen.sh
./configure
make
sudo make install



cd ../../../plc-gps-interface
mkdir build
cd build
cmake ../
make