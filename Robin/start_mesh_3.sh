#!/bin/bash

# Helper: install a package if missing
install_if_missing() {
    if ! command -v "$1" &>/dev/null; then
        echo "Installing $1..."
        sudo apt-get update -y
        sudo apt-get install -y "$2"
    else
        echo "$1 already installed, skipping..."
    fi
}

echo "Checking dependencies..."
install_if_missing iw iw
install_if_missing ip iproute2
install_if_missing batctl batctl

# Load batman-adv
if ! lsmod | grep -q batman_adv; then
    echo "Loading batman-adv kernel module..."
    sudo modprobe batman-adv || {
        echo "batman-adv module not found, installing..."
        sudo apt-get update -y
        sudo apt-get install -y batman-adv-dkms
        sudo modprobe batman-adv
    }
else
    echo "batman-adv kernel module already loaded."
fi

echo "Stopping network services..."
sudo systemctl stop NetworkManager
sudo systemctl stop wpa_supplicant
sudo rfkill unblock wifi

# Delete old mesh0 if exists
if ip link show mesh0 &>/dev/null; then
    echo "Deleting existing mesh0..."
    sudo ip link set mesh0 down
    sudo ip link delete mesh0
fi

# Detect the first wireless interface (physical)
WIFI_IF=$(iw dev | awk '$1=="Interface"{print $2}' | head -n1)
if [ -z "$WIFI_IF" ]; then
    echo "No wireless interface found. Exiting."
    exit 1
fi
echo "Detected wireless interface: $WIFI_IF"

# Get its PHY
PHY=$(iw dev | awk -v iface="$WIFI_IF" '$1=="Interface" && $2==iface {getline; if($1=="phy") print $2}')
if [ -z "$PHY" ]; then
    PHY=$(iw dev | grep -B1 "Interface $WIFI_IF" | grep "^phy" | sed 's/phy//')
fi
echo "Using PHY phy$PHY"

# Remove existing interface bound to this PHY
sudo ip link set "$WIFI_IF" down
sudo iw dev "$WIFI_IF" del

# Create mesh0 in IBSS mode
echo "Creating mesh0 on phy$PHY..."
sudo iw phy "phy$PHY" interface add mesh0 type ibss
sudo ip link set mesh0 up

echo "Joining IBSS network..."
sudo iw dev mesh0 ibss join TestAdhoc 2412

echo "Adding mesh0 to batman-adv..."
sudo batctl if add mesh0
sudo ip link set up dev bat0

echo "Mesh setup complete."
