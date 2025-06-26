#!/bin/bash
echo "Setting up CAN interface..."
if ip link show can0 2>/dev/null; then
    echo "CAN interface exists"
    if ip link show can0 | grep -q 'state DOWN'; then
        echo "Setting CAN interface up"
        ip link set can0 up type can bitrate 250000
    else
        echo 'CAN interface already up'
    fi
else
    echo "Warning: CAN interface can0 not found!"
fi

echo "Setting serial port permissions..."
# Try multiple approaches to ensure permissions are set
for attempt in {1..5}; do
    echo "Attempt $attempt to set serial port permissions"
    
    # Make sure the serial port exists
    if [ -e "/dev/ttyS0" ]; then
        echo "Serial port exists, setting permissions"
        chmod 666 /dev/ttyS0
        chown root:dialout /dev/ttyS0
        
        # Verify permissions
        ls -la /dev/ttyS0
        
        # Check if permissions are correct
        if [ -r "/dev/ttyS0" ] && [ -w "/dev/ttyS0" ]; then
            echo "Successfully set serial port permissions"
            break
        else
            echo "Permission check failed, waiting before retry"
        fi
    else
        echo "WARNING: Serial port /dev/ttyS0 not found! Waiting for it to appear..."
    fi
    
    # Wait before retry
    sleep 10
done

echo "Starting robot application..."
cd /home/raspi/robot
python3 main.py