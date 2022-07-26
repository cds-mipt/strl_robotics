#!/bin/bash

BASE_PATH=$(cd ./"`dirname $0`" || exit; pwd)

if [ ! -d ${BASE_PATH}/../weights ]
then
   mkdir ${BASE_PATH}/../weights
fi

cd ${BASE_PATH}/../weights/.

# netvlad and patchnetvlad weights
wget -O mapillary_WPCA512.pth.tar https://cloudstor.aarnet.edu.au/plus/s/DFxbGgFwh1y1wAz/download
wget -O mapillary_WPCA4096.pth.tar https://cloudstor.aarnet.edu.au/plus/s/ZgW7DMEpeS47ELI/download