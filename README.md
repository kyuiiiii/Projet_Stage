Projet de Stage 2025/2026 L3 Informatique UPHF

Ce projet utilise gcc et utilise deux librairies raylib et readosm via MSYS2
Pour pouvoir lancer et compiler vous avez donc besoin de MSYS2, car il faut installer les paquets suivants dans le terminal approprié (ici mingw64):
gcc:  pacman -S mingw-w64-x86_64-gcc
raylib:  pacman -S mingw-w64-x86_64-raylib
readosm:  pacman -S mingw-w64-x86_64-readosm

Si vous voulez créer et importer vous même un fichier .osm, allez sur le site suivant : https://www.openstreetmap.org/export#map=6/45.71/3.92
puis "Sélectionner manuellement une autre zone" à gauche et enfin exporter.

Plusieurs fichiers .osm sont déjà disponibles pour pouvoir tester, si vous voulez changer de fichier il suffit simplement de changer le nom du fichier 
dans la 1ère ligne du main

Afin de compiler lancer la commande suivante : gcc maingps.c -o ray.exe -lreadosm -lraylib -lopengl32 -lgdi32 -lwinmm
puis lancer le fichier ray.exe
