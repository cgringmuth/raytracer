#!/usr/bin/env bash

outFilename="out34"
(cd build; make;) && time ./build/raytracer -o results/$outFilename.png | tee raytracer.log
# convert $outFilename.ppm $outFilename.png
