#!/bin/bash

# Helper function to check & install dependencies
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

# Check if batman-adv module is loaded
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

echo "Creating ad-hoc interface..."
# Detect first available PHY
PHY=$(sudo iw dev | grep -oP '^phy\d+' | head -n1)
if [ -z "$PHY" ]; then
    echo "No Wi-Fi PHY found. Exiting."
    exit 1
fi

# Find any interface currently bound to that PHY
IFACE=$(sudo iw dev | awk -v phy="$PHY" '$1=="Interface"{print $2}' | head -n1)
if [ -n "$IFACE" ]; then
    echo "Removing existing interface $IFACE from $PHY..."
    sudo ip link set "$IFACE" down
    sudo iw dev "$IFACE" del
fi

# Remove old ah0 if it exists
sudo iw dev ah0 del 2>/dev/null

# Add ah0 as IBSS
echo "Adding ah0 to $PHY..."
sudo iw phy "$PHY" interface add ah0 type ibss
sudo ip link set ah0 up

echo "Joining ad-hoc network..."
sudo iw dev ah0 ibss join TestAdhoc 2412

echo "Adding ah0 to batman-adv and bringing up bat0..."
sudo batctl if add ah0
sudo ip link set up dev bat0

echo "Mesh setup complete."

