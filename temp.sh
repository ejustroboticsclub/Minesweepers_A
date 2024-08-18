#!/bin/bash

for i in {1..15}
do
    # Read temperature in milli-Celsius
    temp_milli_c=$(cat /sys/class/thermal/thermal_zone0/temp)

    # Convert milli-Celsius to Celsius
    temp_celsius=$(($temp_milli_c / 1000))

    # Display the temperature
    echo "Current temperature: ${temp_celsius}°C"

    # Delay
    sleep 0.1
done
