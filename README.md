# Walio ROS2

**Auteur** : Kévin Jordil

**Cadre** : Projet d'appronfondissement du master HES-SO, semestre de printemps 2024

## Résumé

Ce repository contient l'environnement ROS2 pour le projet Walio.

L'objectif de l'environnement est de permettre de gérer les drivers moteurs et le bras robotique de la chaise roulante Walio gâce à un controleur.

Cet environnement ROS2 est prévu pour être utilisé sur une jetson orin avec Ubuntu 22.04. 
- Les drivers moteurs ne sont pas directement connectés à la jetson orin mais à un un raspberry pi. Le raspberry pi est connecté à la jetson orin grâce à un cable réseau et une connexion ethernet.
- Le bras robotique est connecté à la jetson orin via un cable réseau et une connexion ethernet.
- Le controleur est connecté à la jetson orin via un cable USB.

## Installation

### ROS2 Humble

Dans un premier temps, il est necessaire d'installer ROS2 Humble. La documentation officielle est disponible [ici](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#install-ros-2-packages).

Pour installer les dépendances, il faut lancer les commandes suivantes :
    
```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

### Capnp

**TODO TESTER !!**

Il est necessaire d'installer la version 0.7.0 de Capnp. Pour cela, il faut suivre les instructions suivantes :

```bash
sudo apt update
sudo apt install build-essential autoconf automake libtool
curl -O https://capnproto.org/capnproto-c++-0.7.0.tar.gz
tar zxf capnproto-c++-0.7.0.tar.gz
cd capnproto-c++-0.7.0
./configure
make -j6 check
sudo make install
```

Désormais la version 0.7.0 de Capnp est installée. Voici la commande pour vérifier la version de Capnp :

```bash
capnp --version
```

### Workspace

Ce repertoire est un workspace ROS2. Il est cependant possible de récupérer uniquement les packages voulu dans le dossier `src`.

Pour compiler, il faut se placer dans le workspace et lancer la commande `colcon build`.

> Des `warnings` vont apparaitre lors de la compilation. Ils sont liés au code source necessaire à la communication avec opencn.

## Utilisation

Il est maintenant necessaire de sourcer l'environnement ROS2. Pour cela, il faut lancer la commande suivante :

```bash
source install/setup.bash
```

Il est maintenant possible de lancer les différents nodes indépendamment. Voici les commandes pour lancer les nodes:
- Le package OpenCN : `ros2 run opencn_pkg opencn_node`
- Le package Motion control : `ros2 run motion_control motion_control_node`
- Le package Joy : `ros2 run joy joy_node`