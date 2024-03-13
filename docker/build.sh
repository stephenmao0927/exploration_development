#!/bin/sh
# avoid putting droc_simulation repo in a folder containing other large files 
# as it sends the current folder droc_simulation is in to the build process
# (Can move the repo back to your desired folder after building the image)
docker build -t eachan10/hunter_exploration:root -f Dockerfile.root ../..
