#!/bin/bash
sudp apt update && sudo apt install batctl
sudo modprobe batman-adv

sudo iw dev wlp0s20f3 del
sudo iw phy phy0 interface add wlp0s20f3 type ibss
sudo ip link set up mtu 1532 dev wlp0s20f3
sudo iw dev wlp0s20f3 ibss join my-mesh-network 2412 HT20 fixed-freq 02:12:34:56:78:9A #Fehler bei diesem Schritt
sudo batctl if add wlp0s20f3
ip link set up dev bat0

sudo apt install avahi-autoipd
sudo avahi-autoipd bat0

# avahi gibt ein IP aus. Anschließend in neuer Konsole mit `batctl ping` versuchen, Originators zu erreichen.