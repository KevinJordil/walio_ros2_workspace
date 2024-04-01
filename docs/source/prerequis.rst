Prérequis
=========

ROS2 Humble
-----------

Dans un premier temps, il est nécessaire d'installer ROS 2 Humble.
La documentation officielle est disponible `ici <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#install-ros-2-packages>`_.

Pour installer les dépendances, il faut lancer les commandes suivantes dans le workspace cloné :

.. code-block:: console

    sudo rosdep init

.. code-block:: console

    rosdep update

.. code-block:: console

    rosdep install --from-paths src -y --ignore-src


Cap'n Proto
-----------

Il est nécessaire d'installer la version 0.7.0 de Cap'n Proto.

Désinstallation de la version précédente
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Voici les commandes pour désinstaller la version précédente de Cap'n Proto :

.. code-block:: console

    sudo apt update

.. code-block:: console

    sudo apt remove capnproto

.. code-block:: console

    sudo rm -rf /usr/local/bin/capnp

.. code-block:: console

    sudo rm -rf /usr/local/include/capnp

.. code-block:: console

    sudo rm -rf /usr/local/lib/libcapnp*

.. code-block:: console

    sudo rm -rf /usr/local/pkgconfig/capnp*

Installation de la version 0.7.0
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Voici les commandes pour installer la version 0.7.0 de Cap'n Proto :

.. code-block:: console

    sudo apt-get update

.. code-block:: console

    sudo apt install build-essential autoconf automake libtool

.. code-block:: console

    wget https://github.com/capnproto/capnproto/archive/v0.7.0.tar.gz

.. code-block:: console

    tar xvzf v0.7.0.tar.gz

.. code-block:: console

    cd capnproto-0.7.0/c++

.. code-block:: console

    autoreconf -i

.. code-block:: console

    ./configure

.. code-block:: console

    make -j6

.. code-block:: console

    sudo make install


Désormais, la version 0.7.0 de Cap'n Proto est installée.
Voici la commande pour vérifier la version :

.. code-block:: console

    capnp --version
