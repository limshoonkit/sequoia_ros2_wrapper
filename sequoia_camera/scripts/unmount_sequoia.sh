#!/bin/bash
# Unmount Sequoia camera to release PTP lock

echo "Searching for Sequoia mounts..."

# Get the base device URI (without /store_XXXXX)
base_uri=$(gio mount -l | grep -i "Sequoia" | grep "gphoto2://" | awk '{print $NF}' | grep -v "/store_" | head -n 1)

if [ -n "$base_uri" ]; then
    echo "Unmounting $base_uri..."
    gio mount -u "$base_uri" 2>/dev/null || echo "Note: Device may already be unmounted"
else
    echo "No Sequoia base mount found. Checking for store mounts..."
    # If no base URI, try to extract from store paths
    store_uri=$(gio mount -l | grep -i "Sequoia" | grep "gphoto2://" | awk '{print $NF}' | head -n 1)
    if [ -n "$store_uri" ]; then
        # Extract base URI by removing /store_XXXXX
        base_uri=$(echo "$store_uri" | sed 's|/store_[0-9]*||')
        echo "Unmounting $base_uri..."
        gio mount -u "$base_uri" 2>/dev/null || echo "Note: Device may already be unmounted"
    fi
fi

# Double check if it is still there
if gio mount -l | grep -q "Sequoia"; then
    echo "Warning: Mount still detected. Forcing gvfs-gphoto2 shutdown..."
    # Kill the monitor process to stop auto-remounting
    killall gvfs-gphoto2-volume-monitor 2>/dev/null
    killall gvfsd-gphoto2 2>/dev/null
fi

echo "Done."
