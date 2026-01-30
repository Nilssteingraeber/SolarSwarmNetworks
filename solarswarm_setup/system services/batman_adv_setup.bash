#!/bin/bash
# set -e # exit program if one command fails (as subsequent ones are unlikely to succeed then)
# 'set -e' causes issues as adding and removing ip addresses might result in trivial failures
# batman_adv_healthcheck already exists to restart the setup if it fails
NETWORK_NAME=solarswarm

source /etc/environment
if [ $(echo $WLANDEV | wc --chars) -eq 1 ]; then
  >&2 echo "Error: WLANDEV was not set. Export WLANDEV or use the service setup script."
  exit 1
fi

# Kernel-Module (neu) laden
sudo modprobe batman_adv
sudo modprobe -r iwlwifi
sudo modprobe iwlwifi
sleep 1

# Reset Interface
sudo ip link set $WLANDEV down
sudo iwconfig $WLANDEV mode managed
sudo ip link set $WLANDEV up
sleep 1

sudo ip link set $WLANDEV down

# Setze auf IBSS
sudo iwconfig $WLANDEV mode ad-hoc
sudo ip link set $WLANDEV up mtu 1560
sudo ip link set $WLANDEV promisc on
sleep 1

# Join IBSS
sudo iw dev $WLANDEV ibss join $NETWORK_NAME 2412

# bat0 hinzufügen
sudo batctl if add $WLANDEV
sudo ip link set up dev bat0

# IP zuteilen
if [ -z $MESH_IP ]; then # env mit service_helper.bash zuweisen
  exit 1
else
  sudo ip addr add "$MESH_IP/24" dev wlp0s20f3 # bat0
  if sudo ufw status | grep -e " active$"; then
    sudo ufw disable
  fi
  echo "Done"
fi
