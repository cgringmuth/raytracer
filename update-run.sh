#!/usr/bin/env bash

outFilename="out11"
(cd build; make;) && time ./build/raytracer -o $outFilename.ppm
convert $outFilename.ppm $outFilename.png
