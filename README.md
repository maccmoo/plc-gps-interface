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


enable to run without root priveleges (enable port 502 to non-priveleged user, see https://debian-administration.org/article/386/Running_network_services_as_a_non-root_user.)

sudo apt-get install authbind
sudo touch /etc/authbind/byport/502
sudo chown pi:pi /etc/authbind/byport/502
sudo chmod 755 /etc/authbind/byport/502

prefix the command with authbind to give priveleges of binding port 502:
authbind ./plc-gps-interface -p 192.168.1.35 -b 55555


