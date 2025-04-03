#!/bin/bash

# Script to run MicroXRCEDDSAgent on a serial port
source ./install/setup.bash

# Set default serial port and baud rate
SERIAL_PORT="/dev/ttyUSB0"
BAUD_RATE="115200"

# Check if MicroXRCEDDSAgent is installed
if ! command -v MicroXRCEAgent &> /dev/null; then
    echo "Error: MicroXRCEDDSAgent is not installed. Please install it first."
    exit 1
fi

# Run the MicroXRCEDDSAgent with the specified serial port and baud rate
echo "Starting MicroXRCEDDSAgent on serial port $SERIAL_PORT with baud rate $BAUD_RATE..."
MicroXRCEAgent serial --dev $SERIAL_PORT -b $BAUD_RATE