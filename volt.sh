#!/bin/bash

# Check for undervoltage warnings in system logs
echo "Checking for undervoltage warnings..."
dmesg | grep -i voltage

# Display message based on the result
if [ $? -eq 0 ]; then
    echo "Warning: Undervoltage detected. Your power supply might be insufficient."
else
    echo "No undervoltage warnings found. Your power supply seems adequate."
fi
