#!/bin/sh
rsync -azP \
 --exclude '.svn'\
 --exclude '*build*'\
 --exclude '*docs'\
 --exclude '.gitignore'\
 --exclude '*.pyc'\
 --exclude '*.cfgc'\
 --exclude 'cfg/cpp'\
 --exclude '.git'\
 --exclude 'CMakeCache.txt'\
 --exclude 'debug'\
 --exclude 'release'\
 --exclude 'msg_gen'\
 --exclude '.*.swp'\
 --exclude 'sync.sh'\
 --exclude '_*.py'\
 ./ pilot@$PELICAN_IP:~/iafc_ws/src/cvt_ros
