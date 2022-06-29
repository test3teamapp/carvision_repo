#!/bin/bash
# Start detectnet with arguments
#
echo "###############################"
echo "#### Starting detectnet... ####"
echo "###############################"
/home/jimbo/jetson-inference/build/aarch64/bin/detectnet /dev/video1 --input-flip=rotate-180
