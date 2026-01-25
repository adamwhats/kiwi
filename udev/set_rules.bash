#!/bin/bash
#
# Xavbot udev rules installation script
# Installs all 1*.rules files from this directory to /etc/udev/rules.d/
#

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
UDEV_RULES_DIR="/etc/udev/rules.d"

echo "========================================="
echo "Xavbot udev Rules Installation"
echo "========================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "ERROR: This script must be run as root (use sudo)"
    echo "Usage: sudo ./set_rules.bash"
    exit 1
fi

# Find all 1*.rules files
RULE_FILES=("$SCRIPT_DIR"/1*.rules)

# Check if any rule files exist
if [ ! -e "${RULE_FILES[0]}" ]; then
    echo "ERROR: No rule files matching 1*.rules found in $SCRIPT_DIR"
    exit 1
fi

echo "Found the following rule files:"
for rule_file in "${RULE_FILES[@]}"; do
    if [ -f "$rule_file" ]; then
        echo "  - $(basename "$rule_file")"
    fi
done
echo ""

# Install each rule file
echo "Installing rules to $UDEV_RULES_DIR/"
for rule_file in "${RULE_FILES[@]}"; do
    if [ -f "$rule_file" ]; then
        filename=$(basename "$rule_file")
        echo "  Copying $filename..."
        cp -v "$rule_file" "$UDEV_RULES_DIR/"
        chmod 644 "$UDEV_RULES_DIR/$filename"
    fi
done
echo "✓ All rule files installed"
echo ""

echo "Reloading udev rules..."
udevadm control --reload-rules
echo "✓ udev rules reloaded"
echo ""

echo "Triggering udev for all devices..."
udevadm trigger
echo "✓ udev triggered"
echo ""

echo "Waiting for devices to settle..."
sleep 2

echo "========================================="
echo "Checking installed devices:"
echo "========================================="

# Check for RPLidar
if [ -e "/dev/rplidar" ]; then
    echo ""
    echo "✓ RPLidar found:"
    ls -la /dev/rplidar
fi

# Check for any other custom device symlinks created by our rules
echo ""
echo "Custom device symlinks in /dev/:"
ls -la /dev/ | grep -E "rplidar|xavbot" || echo "  (none found yet)"

echo ""
echo "========================================="
echo "Installation complete!"
echo "========================================="
echo ""
echo "If devices are not showing up, try:"
echo "  1. Unplug and replug the USB devices"
echo "  2. Check dmesg for errors: dmesg | tail -20"
echo "  3. Verify rules: ls -la /etc/udev/rules.d/1*.rules"
echo ""
