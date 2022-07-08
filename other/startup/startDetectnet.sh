#!/bin/bash
# Start detectnet with arguments
#
echo "###############################"
echo "#### Starting detectnet... ####"
echo "###############################"
/home/jimbo/jetson-inference/build/aarch64/bin/detectnet csi://0 --input-flip=rotate-180
