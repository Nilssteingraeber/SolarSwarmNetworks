#!/bin/bash

sudo modprobe batman_adv
sudo modprobe -r iwlwifi
sudo modprobe iwlwifi
sleep 1

# 1. Reset Interface
sudo ip link set wlp0s20f3 down
sudo iwconfig wlp0s20f3 mode managed
sudo ip link set wlp0s20f3 up
sleep 1

# 2. Bridge Karte in aktien Zustand
# sudo iw dev wlp0s20f3 conncect -w DummySSID
# sleep 2
sudo ip link set wlp0s20f3 down

# 3. Setze auf IBSS
sudo iwconfig wlp0s20f3 mode ad-hoc
sudo ip link set wlp0s20f3 up mtu 1560
sudo ip link set wlp0s20f3 promisc on
sleep 1

# 4. Join IBSS
sudo iw dev wlp0s20f3 ibss join mymesh 2412

# 5. bat0 hinzufügen
sudo batctl if add wlp0s20f3
sudo ip link set up dev bat0

sudo avahi-autoipd bat0
