.. Walio ROS2 Workspace documentation master file, created by
   sphinx-quickstart on Wed Mar 27 17:49:49 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Bienvenue sur la documentation de Walio ROS2 Workspace
======================================================

Cette documentation explique comment utiliser le workspace ROS2 de Walio.
Ce projet a été réalisé dans le cadre dans projet d'appronfondissement du master HES-SO, semestre de printemps 2024.

L'objectif de l'environnement est de permettre de gérer les drivers moteurs et le bras robotique de la chaise roulante Walio gâce à un controleur.

Cet environnement ROS2 est prévu pour être utilisé sur une jetson orin avec Ubuntu 22.04. 
- Les drivers moteurs ne sont pas directement connectés à la jetson orin mais à un un raspberry pi. Le raspberry pi est connecté à la jetson orin grâce à un cable réseau et une connexion ethernet.
- Le bras robotique est connecté à la jetson orin via un cable réseau et une connexion ethernet.
- Le controleur est connecté à la jetson orin via un cable USB.

Contents
--------

.. toctree::
   installation
   usage