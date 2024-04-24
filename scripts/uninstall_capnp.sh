#!/bin/bash

sudo apt update

sudo apt remove capnproto

sudo rm -rf /usr/local/bin/capnp
sudo rm -rf /usr/local/include/capnp
sudo rm -rf /usr/local/lib/libcapnp*
sudo rm -rf /usr/local/pkgconfig/capnp*

echo "Cap'n Proto has been uninstalled."