#!/usr/bin/env bash

outFilename="out26"
(cd build; make;) && time ./build/raytracer -o results/$outFilename.png | tee raytracer.log
# convert $outFilename.ppm $outFilename.png
