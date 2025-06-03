#!/bin/bash
sudp apt update && sudo apt install batctl
sudo modprobe batman-adv

nmcli radio wifi off
sudo iwconfig wlp0s20f3 mode ibss # WLAN muss ausgeschaltet sein, sonst Fehler "Driver or resource busy"
sudo iwconfig wlp0s20f3 essid my-mesh-network # Optional freq 2412M anhängen
nmcli radio wifi on
sudo ip link set up mtu 1532 dev wlp0s20f3 # hiervor WLAN wieder einschalten
sudo batctl if add wlp0s20f3
ip link set up dev bat0

sudo apt install avahi-autoipd
sudo avahi-autoipd bat0

# avahi gibt ein IP aus. Anschließend in neuer Konsole mit `batctl ping` versuchen, Originators zu erreichen. 