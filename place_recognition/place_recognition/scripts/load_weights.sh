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

# base vgg16 weights (loaded into torch cache dir)
TORCH_CACHE_DIR=/home/docker_place_recognition/.cache/torch/hub/checkpoints
if [ ! -d ${TORCH_CACHE_DIR} ]
then
   mkdir -p ${TORCH_CACHE_DIR}
fi
wget -O $TORCH_CACHE_DIR/vgg16-397923af.pth https://download.pytorch.org/models/vgg16-397923af.pth