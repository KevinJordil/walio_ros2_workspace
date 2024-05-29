Utilisation
===========

Avant de pouvoir lancer les différents nœuds, il est nécessaire de sourcer l'environnement ROS 2 :

.. code-block:: bash

    source install/setup.bash

Il est maintenant possible de lancer les différents nœuds indépendamment.
Voici les commandes pour lancer les nœuds

Le package OpenCN
-----------------

.. code-block:: bash

    ros2 run opencn_pkg opencn_node

Le package Motion control
-------------------------

.. code-block:: bash

    ros2 run motion_control motion_control_node

Le package Joy
--------------

.. code-block:: bash

    ros2 run joy joy_node


Il est également possible de lancer les trois nœuds en même temps avec le fichier de lancement :

.. code-block:: bash

    ros2 launch ./launch/opencn_joy.py