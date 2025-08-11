#!/bin/bash
set -e

# Helper to check and install dependencies
install_if_missing() {
    local cmd=$1
    local pkg=$2
    if ! command -v "$cmd" &>/dev/null; then
        echo "$cmd not found. Installing $pkg..."
        sudo apt-get update -y
        sudo apt-get install -y "$pkg"
    else
        echo "$cmd is already installed, skipping..."
    fi
}

echo "Checking dependencies..."
install_if_missing iw iw
install_if_missing ip iproute2
install_if_missing batctl batctl

# Check if batman-adv module is loaded, try loading it if not
if ! lsmod | grep -q batman_adv; then
    echo "Loading batman-adv kernel module..."
    sudo modprobe batman-adv || {
        echo "batman-adv module not found, installing dkms package..."
        sudo apt-get update -y
        sudo apt-get install -y batman-adv-dkms
        sudo modprobe batman-adv
    }
else
    echo "batman-adv kernel module already loaded."
fi

echo "Stopping network managers..."
sudo systemctl stop NetworkManager wpa_supplicant dhcpcd || true

WIFI_IF=$(iw dev | awk '$1=="Interface"{print $2; exit}')
if [ -z "$WIFI_IF" ]; then
    echo "No wireless interface found. Exiting."
    exit 1
fi
echo "Using wireless interface: $WIFI_IF"

sudo ip link set "$WIFI_IF" down
sudo iw "$WIFI_IF" set type ibss
sudo ip link set "$WIFI_IF" up
sudo iw "$WIFI_IF" ibss join TestAdhoc 2412

sudo batctl if add "$WIFI_IF"
sudo ip link set up dev bat0

echo "Mesh network setup complete!"
