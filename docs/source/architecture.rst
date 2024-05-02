Architecture
============

Une solution générique a été mise en place pour faciliter la communication entre les divers composants.
Son principal avantage réside dans sa capacité à permettre une réutilisation aisée des composants dans d'autres projets, avec peu ou pas d'adaptation nécessaire.

Ci-dessous, un schéma illustre l'architecture de cette solution :

.. thumbnail:: _static/architecture.svg
    :alt: Architecture
    :align: center

|

ROS introduit le concept de nœud, qui représente un processus capable de communiquer avec d'autres nœuds.
Dans notre contexte, chaque :blue:`composant bleu` est un nœud ROS.

Les :red:`liens en rouge` correspondent à des topics ROS, facilitant la communication entre les nœuds.
Chaque topic dispose à la fois d'un subscriber et d'un publisher.

Les :purple:`liens en violet` représentent des services ROS, permettant l'appel de fonctions entre les nœuds.
La direction de la flèche indique le sens de l'appel.
Les services offrent l'avantage de pouvoir attendre une réponse, offrant ainsi la possibilité de déterminer si l'opération a été un succès ou non.

Enfin, :black:`liens en noir` symbolisent les connexions physiques telles que USB, Ethernet, etc., qui permettent la communication avec les différents composants.

Nœud OpenCN
-----------

Le nœud OpenCN intègre la partie :console:`host` de OpenCN.
Il permet de communiquer avec le Raspberry Pi qui fonctionne avec la partie :console:`target` de OpenCN.
Plus d'informations sur OpenCN sont disponibles sur le `site d'OpenCN <https://opencn.heig-vd.ch/>`_

Le nœud OpenCN propose deux services pour générer des transactions OpenCN.
Le premier service nécessite la spécification d'un tableau de pins OpenCN, tandis que le deuxième service requiert un tableau de paramètres.
Une fois les informations fournies, le nœud OpenCN crée la transaction OpenCN correspondante et la transmet au Raspberry Pi.
Si la transaction est réalisée avec succès, le service renvoie la valeur booléenne :console:`True` à la console.

Nœud OpenCN communication interface
-----------------------------------

Le nœud OpenCN communication interface comprend les messages nécessaires pour échanger des données avec le nœud OpenCN.
Ces messages ne sont pas directement intégrés dans le nœud OpenCN, afin de faciliter leur réutilisation dans d'autres nœuds.
Ainsi, pour tout nœud nécessitant une communication avec OpenCN, il suffit de déclarer une dépendance vis-à-vis du nœud OpenCN communication interface.

Pour saisir les définitions suivantes, une compréhension de la structure d'un message ROS est essentielle.
La documentation disponible à l'adresse suivante explique en détail les interfaces pour les messages et les services : `https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html <https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html>`_.
De plus, un lien est fourni pour la création de messages ou de services, ce qui facilite la compréhension de leur implémentation : `https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html>`_.

Service Pins
~~~~~~~~~~~~

Le service Pins est un service ROS permettant de transférer plusieurs Pins OpenCN à la fois.
Structuré comme un tableau d'entrée et de sortie, il prend en entrée des pins OpenCN à envoyer et renvoie en sortie les pins OpenCN reçues, accompagnées d'un booléen indiquant le succès de la transaction.

.. code-block:: console
    :caption: srv/Pins.srv

    Pin[] pins
    ---
    Pin[] pins
    bool success

Le message Pin est un message ROS destiné à transmettre les données relatives à une pin OpenCN.
Il inclut un attribut :console:`pin_class` définissant la classe de la pin (:console:`CMPINI32`, :console:`CMPINU32`, :console:`CMPINBIT` ou, :console:`CMPINFLOAT`), suivi d'une pin de chaque type, seul celui approprié étant utilisé en fonction de la classe.
Le message comporte également un nom et un champ :console:`transaction_type` pouvant être :console:`GET` ou :console:`SET`.

.. code-block:: console
    :caption: msg/Pin.msg

    uint8 pin_class
    uint8 CMPINI32=0
    uint8 CMPINU32=1
    uint8 CMPINBIT=2
    uint8 CMPINFLOAT=3

    CMPinI32 cmpini32
    CMPinU32 cmpinu32
    CMPinBit cmpinbit
    CMPinFloat cmpinfloat

    string name

    uint8 transaction_type
    uint8 GET=0
    uint8 SET=1

Enfin, chaque classe (:console:`CMPINI32`, :console:`CMPINU32`, :console:`CMPINBIT` ou, :console:`CMPINFLOAT`) est un message contenant une valeur correspondante au type.

:console:`CMPINI32` :

.. code-block:: console
    :caption: msg/CMPinI32.msg

    int32 value

:console:`CMPINU32` :

.. code-block:: console
    :caption: msg/CMPinU32.msg

    uint32 value

:console:`CMPINBIT` :

.. code-block:: console
    :caption: msg/CMPinBit.msg

    bool value

:console:`CMPINFLOAT` :

.. code-block:: console
    :caption: msg/CMPinFloat.msg

    float64 value

Service Params
~~~~~~~~~~~~~~

Le service Params est une service ROS permettant de transférer plusieurs Params OpenCN à la fois.
Structuré comme un tableau d'entrée et de sortie, il prend en entrée des params OpenCN à envoyer et renvoie en sortie les params OpenCN reçues, accompagnées d'un booléen indiquant le succès de la transaction.

.. code-block:: console
    :caption: srv/Params.srv

    Params[] params
    ---
    Params[] params
    bool success

Le message Param est un message ROS destiné à transmettre les données relatives à un param OpenCN.
Il inclut un attribut :console:`param_class` définissant la classe du param (:console:`CMPARAMI32`, :console:`CMPARAMU32`, :console:`CMPARAMBIT` ou, :console:`CMPARAMFLOAT`), suivi d'un param de chaque type, seul celui approprié étant utilisé en fonction de la classe.
Le message comporte également un nom et un champ :console:`transaction_type` pouvant être :console:`GET` ou :console:`SET`.

.. code-block:: console
    :caption: msg/Param.msg

    uint8 param_class
    uint8 CMPARAMI32=0
    uint8 CMPARAMU32=1
    uint8 CMPARAMBIT=2
    uint8 CMPARAMFLOAT=3

    CMParamI32 cmparami32
    CMParamU32 cmparamu32
    CMParamBit cmparambit
    CMParamFloat cmparamfloat

    string name

    uint8 transaction_type
    uint8 GET=0
    uint8 SET=1

Enfin, chaque classe (:console:`CMPARAMI32`, :console:`CMPARAMU32`, :console:`CMPARAMBIT` ou, :console:`CMPARAMFLOAT`) est un message contenant une valeur correspondante au type.

:console:`CMParamI32` :

.. code-block:: console
    :caption: msg/CMParamI32.msg

    int32 value

:console:`CMParamU32` :

.. code-block:: console
    :caption: msg/CMParamU32.msg

    uint32 value

:console:`CMParamBit` :

.. code-block:: console
    :caption: msg/CMParamBit.msg

    bool value

:console:`CMParamFloat` :

.. code-block:: console
    :caption: msg/CMParamFloat.msg

    float64 value


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
