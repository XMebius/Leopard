#/bin/bash

QT_VER="$(ls ~/Qt5.12.12/ | grep 5 -m1)"

printf "${HOME}/Qt5.12.12/${QT_VER}/gcc_64/"

