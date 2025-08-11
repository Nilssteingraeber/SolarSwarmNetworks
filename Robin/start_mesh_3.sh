#!/bin/bash
set -e

# --- Helper to install dependencies ---
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

# --- Load batman-adv kernel module ---
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

# --- Stop interfering services ---
echo "Stopping network services..."
sudo systemctl stop NetworkManager || true
sudo systemctl stop wpa_supplicant || true

# --- Detect first wireless interface ---
WIFI_IF=$(iw dev | awk '$1=="Interface"{print $2; exit}')
if [ -z "$WIFI_IF" ]; then
    echo "No wireless interface found. Exiting."
    exit 1
fi
echo "Detected wireless interface: $WIFI_IF"

# --- Get PHY name and clean it ---
PHY=$(iw dev | grep -B1 "Interface $WIFI_IF" | grep "^phy" | sed 's/#//')
if [ -z "$PHY" ]; then
    echo "Could not determine PHY for $WIFI_IF. Exiting."
    exit 1
fi
echo "Using PHY $PHY"

# --- Remove old mesh0 if it exists ---
if ip link show mesh0 &>/dev/null; then
    echo "Removing existing mesh0..."
    sudo ip link set mesh0 down || true
    sudo iw dev mesh0 del || true
fi

# --- Remove the original interface before creating mesh0 ---
echo "Removing existing interface $WIFI_IF..."
sudo ip link set "$WIFI_IF" down || true
sudo iw dev "$WIFI_IF" del || true

# --- Create mesh0 in IBSS mode ---
echo "Creating mesh0 on $PHY..."
sudo iw phy "$PHY" interface add mesh0 type ibss
sudo ip link set mesh0 up

# --- Join the IBSS network ---
echo "Joining IBSS network..."
sudo iw dev mesh0 ibss join TestAdhoc 2412

# --- Add mesh0 to batman-adv ---
echo "Adding mesh0 to batman-adv..."
sudo batctl if add mesh0
sudo ip link set up dev bat0

echo "Mesh setup complete."
