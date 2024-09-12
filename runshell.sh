#!/bin/bash
singularity shell --nv --env DISPLAY=$DISPLAY -B /tmp/.X11-unix:/tmp/.X11-unix/ -B `pwd`:/jackal_ws/src/the-barn-challenge nav_competition_image.sif