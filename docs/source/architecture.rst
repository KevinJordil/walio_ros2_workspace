Architecture
============

Une solution générique a été mise en place pour communiquer entre les différents composants.
L'avantage de cette solution est de pouvoir réutiliser facilement les composants nécessaires pour un autre projet avec peu ou pas d'adaptation.

Voici un schéma de l'architecture de la solution :

.. image:: _static/architecture.svg
    :alt: Architecture
    :align: center

ROS amène une notion de nœud qui est un processus qui communique avec les autres nœuds.
Dans notre cas, chaque :blue:`composant bleu` est un nœud ROS.

Les :red:`liens en rouge` sont des topics ROS qui permettent de communiquer entre les nœuds.
Il y a un subscriber et un publisher pour chaque topic.

Les :purple:`liens en violet` sont des services ROS qui permettent de faire des appels de fonctions entre les nœuds.
Le sens de la flèche indique le sens de l'appel.
L'avantage des services est de pouvoir attendre une réponse et donc de savoir si ça a fonctionné ou non.

Les :black:`liens en noir` sont des liens physiques (USB, Ethernet, etc.) qui permettent de communiquer avec les différents composants.


Nœud OpenCN
-----------

Le nœud OpenCN intègre la partie :console:`host` de OpenCN.
Il permet de communiquer avec le Raspberry Pi qui fonctionne avec la partie :console:`target` de OpenCN.
Plus d'informations sur OpenCN sont disponibles sur le `site d'OpenCN <https://opencn.heig-vd.ch/>`_

Le nœud OpenCN possède un service qui permet de générer une transaction OpenCN.
Dans le message du service, il faut spécifier un tableau de pins OpenCN.
Le nœud OpenCN va générer la transaction OpenCN et la transmettre au Raspberry Pi.
Si la transaction est correctement effectuée, le service renvoie le booléen de succès à :console:`True`.

Nœud Motion Control
-------------------

Le nœud Motion Control permet de gérer ce qui doit être fait par rapport aux entrées.
Dans le cas actuel, il prend en entrée les valeurs du contrôleur Xbox et les transforme en consignes compatibles avec OpenCN (Pins).

Actuellement, le joystick gauche permet de contrôler la vitesse du moteur avec un profil de type vélocité.

Si à l'avenir, les déplacements doivent être gérés de manière autonome, il faudra modifier ce nœud pour prendre en compte les données des capteurs et les consignes de déplacement.

Nœud Joy
--------

Le nœud Joy permet de récupérer les données du contrôleur Xbox et de les publier sur un topic ROS.

La manière de connexion n'est pas gérée dans ce nœud.
Si dans le futur, l'option sans fil est choisie, il faut connecter le contrôleur Xbox en Bluetooth au système d'exploitation.

Ce nœud est un nœud ROS standard.
La documentation de ce nœud est disponible `ici <https://index.ros.org/p/joy/>`_.
