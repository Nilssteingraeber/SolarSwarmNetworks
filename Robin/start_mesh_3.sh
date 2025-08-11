#!/bin/bash
set -e

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
sudo systemctl stop NetworkManager || true
sudo systemctl stop wpa_supplicant || true
sudo systemctl stop dhcpcd || true

# Detect wireless interface
WIFI_IF=$(iw dev | awk '$1=="Interface"{print $2; exit}')
if [ -z "$WIFI_IF" ]; then
    echo "No wireless interface found. Exiting."
    exit 1
fi
echo "Detected wireless interface: $WIFI_IF"

# Detect PHY
PHY=$(iw dev | grep -B1 "$WIFI_IF" | head -n1 | awk '{print $1}')
if [ -z "$PHY" ]; then
    echo "Could not detect PHY for $WIFI_IF. Exiting."
    exit 1
fi
echo "Using PHY $PHY"

# Remove old mesh0 if exists
sudo iw dev mesh0 del 2>/dev/null || true

echo "Creating mesh0 on $PHY..."
if sudo iw phy "$PHY" interface add mesh0 type ibss 2>/tmp/iw_error.log; then
    echo "mesh0 created successfully."
else
    ERRMSG=$(cat /tmp/iw_error.log)
    echo "Failed to create mesh0: $ERRMSG"
    echo "Falling back to renaming $WIFI_IF to mesh0..."
    sudo ip link set "$WIFI_IF" down
    sudo ip link set "$WIFI_IF" name mesh0
fi

# Bring mesh0 up
sudo ip link set mesh0 up

echo "Joining IBSS network..."
sudo iw dev mesh0 ibss join TestAdhoc 2412

echo "Adding mesh0 to batman-adv and bringing up bat0..."
sudo batctl if add mesh0
sudo ip link set up dev bat0

echo "Mesh setup complete."
