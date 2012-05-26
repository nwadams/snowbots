#!/bin/bash

if [ ! $1 ]; then
  echo "please enter a program name to execute"
  exit
fi
python -m cProfile -o profile.out $@
dt=`date '+%Y%m%d-%H%M%S'`
python profile.py > profile.${dt}.txt
