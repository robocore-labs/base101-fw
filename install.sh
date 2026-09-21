#!/bin/bash

# Installs the Link101 udev rules on the ROS host (Raspberry Pi / Jetson).

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Installing Link101 udev rules..."

sudo rm -f /etc/udev/rules.d/99-axon-devices.rules
sudo cp "$SCRIPT_DIR/99-link101-devices.rules" /etc/udev/rules.d/
echo "✓ Installed udev rules"

sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
echo "✓ Reloaded udev rules"

echo ""
echo "Installation complete!"
echo ""
echo "The normal firmware exposes one port:"
echo "  /dev/link101-zenoh  - zenoh serial transport (point zenohd here)"
echo "Standalone diagnostic targets instead expose /dev/link101-debug."
