#!/usr/bin/sh

echo "deb https://repo.download.nvidia.com/jetson/common <release> main" > /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
echo "deb https://repo.download.nvidia.com/jetson/<platform> <release> main" >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list

apt-get update
apt-get install -y \
    libegl-dev \
    libegl1 \
    nvidia-l4t-jetson-multimedia-api
