#!/bin/bash

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

# Delete existing mesh0 if it exists
if ip link show mesh0 &>/dev/null; then
    echo "Deleting existing mesh0 interface..."
    sudo ip link set mesh0 down
    sudo ip link delete mesh0
fi

# Detect first physical interface (excluding lo, bat0, etc)
PHYS_IFACE=$(ip -o link show | awk -F': ' '{print $2}' | grep -Ev '^(lo|bat[0-9]*|mesh0|ah0)$' | head -n1)

if [ -z "$PHYS_IFACE" ]; then
    echo "No suitable physical interface found. Exiting."
    exit 1
fi

echo "Detected physical interface: $PHYS_IFACE"

# Bring down physical interface before renaming
sudo ip link set "$PHYS_IFACE" down

echo "Renaming $PHYS_IFACE to mesh0..."
sudo ip link set "$PHYS_IFACE" name mesh0

# Bring up mesh0 interface
sudo ip link set mesh0 up

# If wireless, join IBSS on mesh0
if iw dev mesh0 info &>/dev/null; then
    echo "Joining ad-hoc (IBSS) network on mesh0..."
    sudo iw dev mesh0 ibss join TestAdhoc 2412
fi

# Add mesh0 to batman-adv and bring up bat0
echo "Adding mesh0 to batman-adv and bringing up bat0..."
sudo batctl if add mesh0
sudo ip link set up dev bat0

echo "Mesh setup complete."
