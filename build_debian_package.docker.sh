#!/bin/bash
set -x

VERSION=0.0.0-0
rm -f ros-galactic-nesfr-arm-description_*_arm64.deb
rm -f ros-galactic-nesfr-arm-node_*_arm64.deb
rm -f ros-galactic-nesfr-arm-bringup_*_arm64.deb

echo "###################################################################"
echo " Build nesfr-arm-description"
echo "###################################################################"
cd ./nesfr_arm_description && dpkg-buildpackage -b -us -uc && cd .. && \
  apt-get install ./ros-galactic-nesfr-arm-description_*_arm64.deb
echo "###################################################################"
echo " Build nesfr-arm-node"
echo "###################################################################"
cd ./nesfr_arm_node && dpkg-buildpackage -b -us -uc && cd .. && \
apt-get install ./ros-galactic-nesfr-arm-node_*_arm64.deb
echo "###################################################################"
echo " Build nesfr-arm-bringup"
echo "###################################################################"
cd ./nesfr_arm_bringup && dpkg-buildpackage -b -us -uc
