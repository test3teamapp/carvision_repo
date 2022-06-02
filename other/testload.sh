#!/bin/bash
for (( c=1; c<=500; c++ ))
do  
   #echo "Welcome $c times"
   #curl --location --request POST 'http://127.0.0.1:8080/carvision/imageupload' --form 'f="rrr"'
   curl --location --request POST 'http://10.42.0.1:8080/carvision/imageupload' --form 'f="rrr"'
done