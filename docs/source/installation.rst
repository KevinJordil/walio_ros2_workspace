Installation
============

ROS2 Humble
-----------

Dans un premier temps, il est necessaire d'installer ROS2 Humble. La documentation officielle est disponible [ici](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#install-ros-2-packages).

Pour installer les dépendances, il faut lancer les commandes suivantes :

.. code-block:: bash

    sudo rosdep init
    rosdep update
    rosdep install --from-paths src -y --ignore-src


Capnp
-----

Il est necessaire d'installer la version 0.7.0 de Capnp. Pour cela, il faut suivre les instructions suivantes :

Désinstaller la version précédente de Capnp :

.. code-block:: bash

    sudo apt update
    sudo apt remove capnproto
    sudo rm -rf /usr/local/bin/capnp
    sudo rm -rf /usr/local/include/capnp
    sudo rm -rf /usr/local/lib/libcapnp*
    sudo rm -rf /usr/local/pkgconfig/capnp*

Installer la version 0.7.0 de Capnp :

.. code-block:: bash

    sudo apt update
    sudo apt install build-essential autoconf automake libtool
    wget https://github.com/capnproto/capnproto/archive/v0.7.0.tar.gz
    tar xvzf v0.7.0.tar.gz
    cd capnproto-0.7.0/
    autoreconf -i
    ./configure
    make -j6 check
    sudo make install


Il est possible que les tests ne passent pas lors de la commande `make -j6 check`, ce n'est pas grave, l'installation fonctionne ensuite.

Désormais la version 0.7.0 de Capnp est installée. Voici la commande pour vérifier la version de Capnp :

.. code-block:: bash

    capnp --version


Workspace
---------

Ce repertoire est un workspace ROS2. Il est cependant possible de récupérer uniquement les packages voulu dans le dossier `src`.

Pour compiler, il faut se placer dans le workspace et lancer la commande suivante:

.. code-block:: bash

    colcon build

.. warning::
    Des `warnings` vont apparaitre lors de la compilation. Ils sont liés au code source necessaire à la communication avec opencn.