#!/usr/bin/sh

apt-key adv --fetch-key https://repo.download.nvidia.com/jetson/jetson-ota-public.asc
echo "deb https://repo.download.nvidia.com/jetson/common r36.3 main" > /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
echo "deb https://repo.download.nvidia.com/jetson/t234 r36.3 main" >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list


apt-get update
apt-get install -y \
    libegl-dev \
    libegl1 \
    nvidia-l4t-jetson-multimedia-api
