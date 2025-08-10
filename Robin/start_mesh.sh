echo "Stopping network services..."
sudo systemctl stop NetworkManager

echo "Creating ad-hoc interface..."
sudo iw dev ah0 del  2>/dev/null
sudo iw phy phy0 interface add ah0 type ibss
sudo ip link set ah0 up

echo "Joining ad-hoc network..."
sudo iw dev ah0 ibss join TestAdhoc 2412

echo "Adding ah0 to batman-adv and bringing up bat0..."
sudo batctl if add ah0
sudo ip link set up dev bat0

echo "Mesh setup complete."
