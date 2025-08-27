#!/bin/bash

# Kernel-Module (neu) laden
sudo modprobe batman_adv
sudo modprobe -r iwlwifi
sudo modprobe iwlwifi
sleep 1

# Reset Interface
sudo ip link set $1 down
sudo iwconfig $1 mode managed
sudo ip link set $1 up
sleep 1

sudo ip link set $1 down

# Setze auf IBSS
sudo iwconfig $1 mode ad-hoc
sudo ip link set $1 up mtu 1560
sudo ip link set $1 promisc on
sleep 1

# Join IBSS
sudo iw dev $1 ibss join mymesh 2412

# bat0 hinzufügen
sudo batctl if add $1
sudo ip link set up dev bat0

# IP zuteilen
sudo avahi-autoipd bat0