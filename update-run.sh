#!/usr/bin/env bash

outFilename="out10"
(cd build; make;) && time ./build/raytracer -o $outFilename.png
# convert $outFilename.ppm $outFilename.png
