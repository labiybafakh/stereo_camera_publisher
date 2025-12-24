#!/bin/bash

# Script to install udev rules for the stereo camera

echo "Installing udev rules for 3D Global Shutter Stereo Camera..."

# Copy udev rules to system directory
sudo cp 99-stereo-camera.rules /etc/udev/rules.d/

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "udev rules installed successfully!"
echo ""
echo "Checking for stereo camera symlinks..."
sleep 1

if [ -L /dev/stereo_left ] && [ -L /dev/stereo_right ]; then
    echo "✓ Symlinks created successfully:"
    ls -l /dev/stereo_left /dev/stereo_right
else
    echo "⚠ Symlinks not found. Please check:"
    echo "  1. Camera is connected (lsusb | grep 32e4:9282)"
    echo "  2. Video devices exist (v4l2-ctl --list-devices)"
    echo "  3. Kernel numbers in udev rules match your system"
fi
