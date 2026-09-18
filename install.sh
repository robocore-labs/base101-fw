#!/bin/bash

# Installs the Axon udev rules on the ROS host (Raspberry Pi / Jetson).

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Installing Axon udev rules..."

sudo cp "$SCRIPT_DIR/99-axon-devices.rules" /etc/udev/rules.d/
echo "✓ Installed udev rules"

sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
echo "✓ Reloaded udev rules"

echo ""
echo "Installation complete!"
echo ""
echo "The normal firmware exposes one port:"
echo "  /dev/axon-zenoh  - zenoh serial transport (point zenohd here)"
echo "The standalone IMU diagnostic instead exposes /dev/axon-debug."
