#!/bin/bash
# sudp apt update && sudo apt install batctl
sudo modprobe batman-adv
# sudo modprobe -r iwlwifi
# sudo modprobe iwlwifi

sudo iw dev wlp0s20f3 del
sudo iw phy phy0 interface add wlp0s20f3 type ibss
sudo ip link set up mtu 1560 dev wlp0s20f3
sudo iw dev wlp0s20f3 ibss join my-mesh-network 2412 HT20 fixed-freq 38:00:25:52:f7:10 #Fehler bei diesem Schritt
sudo batctl if add wlp0s20f3
sudo ip link set up dev bat0

# sudo apt install avahi-autoipd
# sudo avahi-autoipd bat0

# avahi gibt ein IP aus. Anschließend in neuer Konsole mit `batctl ping` versuchen, Originators zu erreichen.
