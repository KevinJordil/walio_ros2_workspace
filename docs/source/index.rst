Documentation de Walio ROS2 Workspace
=====================================

Cette documentation explique comment utiliser l'espace de travail ROS2 de Walio.
Ce projet a été réalisé dans le cadre du projet d'approfondissement du master HES-SO, semestre de printemps 2024.

L'objectif de l'environnement est de permettre de gérer les drivers moteurs et le bras robotique de la chaise roulante Walio grâce à un contrôleur.

Cet environnement ROS2 est prévu pour être utilisé sur une Jetson Orin avec Ubuntu 22.04.

* Les drivers moteurs ne sont pas directement connectés à la Jetson Orin mais à un Raspberry Pi. Le Raspberry Pi est connecté à la Jetson Orin grâce à un câble réseau et une connexion Ethernet.
* Le bras robotique est connecté à la Jetson Orin via un câble réseau et une connexion Ethernet.
* Le contrôleur est connecté à la Jetson Orin via un câble USB.

Contenu
-------

.. toctree::
   prerequis
   compilation
   utilisation
   architecture
   packages
   cicd