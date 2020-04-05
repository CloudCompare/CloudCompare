Cloud Compare Python API TODO list
==================================

Wrapping: PyQt SIP, ou migration vers Qt for Python (Pyside2 Shiboken2) ?
-------------------------------------------------------------------------

CloudCompare s'appuie sur Qt : PyQt - SIP est plus adapté que SWIG, mais un peu laborieux...
Avec les versions récentes de Qt (5.12 -), Qt propose Pyside2 et Shiboken2.
Pour un premier test, sur Ubuntu 18.4, (Qt 5.9.5), je suis resté à PyQt -SIP, avec les paquets système natifs.
Faut-il passer à Pyside2 / Shiboken2 ? 
Cf. https://machinekoder.com/pyqt-vs-qt-for-python-pyside2-pyside/
Si oui, migration à prévoir SIP -> Shiboken2. 
Côté Python, grande compatibilité des scripts entre Pyside2 et PyQt ==> pas d'impact notable pour les futurs utilisateurs.

CMakefile pour SIP
------------------
Récup sur SALOME, adaptation.
Problèmes:
- compilation parallèle cassée (make -jxx) sur la génération des sources C issus des .sip.
- nom des sources générés par SIP à saisir en dur dans le makefile.
Tester le CMake proposé par Qt_Python_Binding (https://github.com/ros-visualization/python_qt_binding.git)

Génération et tests sous Windows 10
-----------------------------------
Actuellement, build et tests uniquement sur Ubuntu 18.4 avec paquets natifs.
J'ai une machine double boot Windows 10 avec Visual Studio 2017, donc faisable...

Extension du wrapping
---------------------
Interface très limitée, pour premiers tests :
cloudCompare
- lecture / ecriture de nuages de points
- calcul de courbure
ccPointCloud
- computeGravityCenter
- scale, translate
- getName, hasScalarFields, getNumberOfScalarFields, getScalarFieldName
- exportCoordToSF
- getScalarField
ScalarField
- getName
- initNumpyApi (static a appeler une fois)
- toNpArray
- fromNpArray
L'idée est de compléter en s'inspirant des fonctions disponibles dans l'interface de commande.

Eléments de code de qCC dupliqués
---------------------------------
Le code de l'API Python de Cloud Compare est regroupé dans un répertoire CCPAPI (Cloud Compare Python API).
Il s'appuie sur potentiellement toutes les libraries de CloudCompare, sauf l'application elle même (qCC).
Il ne nécessite aucune modification des librairies appelées.
Par contre, nécessité de dupliquer du code issu de qCC, sans les appels IHM graphique.
Par exemple, pour le calcul de courbure, récup dans ccLibAlgorithms::ComputeGeomCharacteristic, ccLibAlgorithms::GetDensitySFName.
Faut-il une couche intermédaire de traitements appelables depuis l'IHM graphique et l'interface Python ?

ScalarField <--> Numpy Array: type float en dur
-----------------------------------------------
Prévoir de gérer les double si CloudCompare compilé avec l'option ScalarType == double

Numpy, ownership des ScalarFields
---------------------------------
On peut transformer un ScalarField en Numpy Array sans copie.
Normalement, l'ownership reste coté C++ : la destruction du ScalarField ne peut se faire que via une méthode de CloudCompare (méthode explicite à ajouter à l'interface Python).
A l'inverse, on peut écraser les données d'un ScalarField existant par celles d'un Numpy Array de même type, même dimensions et taille (vecteur du même nombre d'éléments.
Ici, l'opération de fait par copie (memcpy).
Bâtir des tests pour s'assurer que l'on évite les destructions prématurées ou les fuites mémoire.

Compteur de référence Python, Ownership
---------------------------------------
Le garbage collector de Python s'appuie sur un compteur de référence sur lequel on peut agir dans l'interface C++ (Py_INCREF, PY_DECREF).
SIP permet de dire explictement qui a l'ownership des objets créés à l'interface.
Etablir des règles et des tests pour être sûr d'écrire du code fiable (Numpy Array <--> ScalarField est un cas particulier important).
NB: pas de problème détecté à ce jour sur l'interface réduite, avec des tests simples...




