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

# Find first non-loopback, non-batman interface
# Exclude lo, bat0, bat1, mesh0, ah0, etc.
INTERFACE=$(ip -o link show | awk -F': ' '{print $2}' | grep -Ev '^(lo|bat[0-9]*|mesh0|ah0)$' | head -n1)

if [ -z "$INTERFACE" ]; then
    echo "No suitable network interface found. Exiting."
    exit 1
fi

echo "Using network interface: $INTERFACE"

# Check if interface is wireless
if iw dev "$INTERFACE" info &>/dev/null; then
    echo "$INTERFACE is wireless, joining IBSS..."

    echo "Bringing down interface $INTERFACE..."
    sudo ip link set "$INTERFACE" down

    echo "Joining ad-hoc (IBSS) network on $INTERFACE..."
    sudo iw dev "$INTERFACE" ibss join TestAdhoc 2412

    echo "Bringing interface $INTERFACE up..."
    sudo ip link set "$INTERFACE" up

else
    echo "$INTERFACE is wired (Ethernet) or non-wireless, skipping IBSS join."
fi

echo "Adding $INTERFACE to batman-adv and bringing up bat0..."
sudo batctl if add "$INTERFACE"
sudo ip link set up dev bat0

echo "Mesh setup complete."
