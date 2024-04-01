CI/CD
=====

L'intégration continue (CI) et le déploiement continu (CD) sont des pratiques de développement logiciel qui permettent de vérifier et de valider le code source d'un projet de manière automatique.
Dans le cadre de ce projet, il y a de l'intégration continue pour le workspace ROS 2.
Pour la documentation, il y a un l'intégration continue et un déploiement continu.

ROS2 Workspace build
--------------------

Il existe un CI qui permet de compiler le workspace ROS 2.
Il permet de s'assurer que le code source présent sur le dépôt est bien compilable.

Chaque fois qu'une modification du dossier `src` est effectuée sur la branche `main`, le CI est déclenché.
Il est possible de voir l'état du CI sur le README du dépôt et également dans l'onglet `Actions` du dépôt.

Sphinx Documentation
--------------------

Il existe un CI/CD qui permet de générer et de publier la documentation du projet.
Il permet de s'assurer que la documentation en ligne est à jour et qu'elle est bien générée.

Chaque fois qu'une modification du dossier `docs` est effectuée sur la branche `main`, le CI est déclenché.
Il est possible de voir l'état du CI sur le README du dépôt et également dans l'onglet `Actions` du dépôt.





