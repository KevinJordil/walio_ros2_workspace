#!/bin/bash

sudo apt-get update

sudo apt install -y build-essential autoconf automake libtool

wget https://github.com/capnproto/capnproto/archive/v0.7.0.tar.gz

tar xvzf v0.7.0.tar.gz
cd capnproto-0.7.0/c++

autoreconf -i
./configure


make -j6

sudo make install

echo "Cap'n Proto has been installed."
