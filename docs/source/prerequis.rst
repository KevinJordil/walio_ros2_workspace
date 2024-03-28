Prérequis
=========

ROS2 Humble
-----------

Dans un premier temps, il est nécessaire d'installer ROS 2 Humble.
La documentation officielle est disponible [ici](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#install-ros-2-packages).

Pour installer les dépendances, il faut lancer les commandes suivantes :

.. code-block:: bash

    sudo rosdep init
    rosdep update
    rosdep install --from-paths src -y --ignore-src


Cap'n Proto
-----------

Il est nécessaire d'installer la version 0.7.0 de Cap'n Proto.

Désinstallation de la version précédente
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Voici les commandes pour désinstaller la version précédente de Cap'n Proto :

.. code-block:: bash

    sudo apt update
    sudo apt remove capnproto
    sudo rm -rf /usr/local/bin/capnp
    sudo rm -rf /usr/local/include/capnp
    sudo rm -rf /usr/local/lib/libcapnp*
    sudo rm -rf /usr/local/pkgconfig/capnp*

Installation de la version 0.7.0
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Voici les commandes pour installer la version 0.7.0 de Cap'n Proto :

.. code-block:: bash

    sudo apt-get update
    sudo apt install build-essential autoconf automake libtool
    wget https://github.com/capnproto/capnproto/archive/v0.7.0.tar.gz
    tar xvzf v0.7.0.tar.gz
    cd capnproto-0.7.0/c++
    autoreconf -i
    ./configure
    make -j6
    sudo make install


Désormais, la version 0.7.0 de Cap'n Proto est installée.
Voici la commande pour vérifier la version :

.. code-block:: bash

    capnp --version
