#!/usr/bin/env bash

outFilename="out1"
(cd build; make;) && time ./build/raytracer scene.yaml -o results/$outFilename.png | tee raytracer.log
# convert $outFilename.ppm $outFilename.png
