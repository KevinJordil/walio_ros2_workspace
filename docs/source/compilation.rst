Compilation
===========

Il est maintenant nécessaire de sourcer l'environnement ROS 2.
Pour cela, il faut lancer la commande suivante :

.. code-block:: bash

    source /opt/ros/humble/setup.bash	

Il peut être intéressant de rajouter cette commande dans le fichier `.bashrc` pour ne pas avoir à la lancer à chaque fois.

Workspace
---------

Pour compiler, il faut se placer dans le workspace et lancer la commande suivante :

.. code-block:: bash

    colcon build

.. note::
    Des avertissements vont apparaître lors de la compilation. Ils sont liés au code source nécessaire à la communication avec OpenCN.


Il est également possible de compiler un package en particulier en utilisant la commande suivante :

.. code-block:: bash

    colcon build --packages-select <package_name>