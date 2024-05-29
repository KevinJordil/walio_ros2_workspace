Prérequis
=========

ROS2 Humble
-----------

Dans un premier temps, il est nécessaire d'installer ROS 2 Humble.
La documentation officielle est disponible `ici <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>`_.

Pour installer les dépendances, il faut lancer les commandes suivantes dans le workspace cloné :

.. code-block:: console

    sudo rosdep init

.. code-block:: console

    rosdep update

.. code-block:: console

    rosdep install --from-paths src -y --ignore-src --rosdistro humble


Cap'n Proto
-----------

Il est nécessaire d'installer la version 0.7.0 de Cap'n Proto.

Désinstallation de la version précédente
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Un script est présent dans le dossier :console:`scripts` du workspace cloné pour désinstaller la version précédente de Cap'n Proto.
Il est possible de lancer ce script avec la commande suivante :

.. code-block:: console

    chmod +x scripts/uninstall_capnp.sh

.. code-block:: console

    ./scripts/uninstall_capnp.sh

Installation de la version 0.7.0
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Un script est présent dans le dossier :console:`scripts` du workspace cloné pour installer la version 0.7.0 de Cap'n Proto.
Il est possible de lancer ce script avec la commande suivante :

.. code-block:: console

    chmod +x scripts/install_capnp.sh

.. code-block:: console

    ./scripts/install_capnp.sh

Désormais, la version 0.7.0 de Cap'n Proto est installée.
Voici la commande pour vérifier la version :

.. code-block:: console

    capnp --version

Contrôleur Xbox One
-------------------


Pour pouvoir utiliser un contrôleur Xbox One, vous devez installer le package :console:`xboxdrv`.

Voici la commande pour l'installation :

.. code-block:: console

    sudo apt-get update

.. code-block:: console

    sudo apt-get install xboxdrv

L'utilisation du contrôleur Xbox One s'effectuera via un câble USB.