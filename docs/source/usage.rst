Utilisation
=====

Il est maintenant necessaire de sourcer l'environnement ROS2. Pour cela, il faut lancer la commande suivante :

.. code-block:: bash

    source /opt/ros/foxy/setup.bash	


Il est maintenant possible de lancer les différents nodes indépendamment. Voici les commandes pour lancer les nodes:
- Le package OpenCN : `ros2 run opencn_pkg opencn_node`
- Le package Motion control : `ros2 run motion_control motion_control_node`
- Le package Joy : `ros2 run joy joy_node`