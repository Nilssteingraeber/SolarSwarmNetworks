#!/bin/bash
NETWORK_NAME=solarswarm

source /etc/environment
if [ $(echo $WLANDEV | wc --chars) -eq 1 ]; then
  >&2 echo "Error: WLANDEV was not set. Export WLANDEV or use the service setup script."
  exit 1
fi

# Zwischen den einzelnen Schritten wird geschlafen, da manche Eingriffe Zeit brauchen
# Der darauffolgende Befehl kann dann fehlschlagen

# Kernel-Module (neu) laden
sudo modprobe batman_adv
sudo modprobe -r iwlwifi
sleep 1
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
sudo ip link set $WLANDEV up mtu 1560 # Kernelmodul batman-adv warnt, dass mindestens 1560 für header benötigt werden
sudo ip link set $WLANDEV promisc on # Fluten erlauben
sleep 1

# Join IBSS
sudo iw dev $WLANDEV ibss join $NETWORK_NAME 2412 # mesh bilden/beitreten

# bat0 hinzufügen
sudo batctl if add $WLANDEV
sudo ip link set up dev bat0

# IP zuteilen
if [ -z $MESH_IP ]; then
  # soll eine statische IP-Adresse erhalten
  exit 1
else
  sudo ip addr add "$MESH_IP/24" dev wlp0s20f3 # bat0
  if sudo ufw status | grep -e " active$"; then
    # Firewall deaktivieren für Ad-hoc-Netzwerk
    # kann mit Option managed von 'service_helper.bash' wieder aktiviert werden
    # oder 'sudo ufw enable'
    sudo ufw disable
  fi
  echo "Done"
fi
