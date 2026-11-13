#!/bin/bash
# Lab 11 Task 1: free the serial port, fix its permissions, and run plot.py
# Run from the project folder (the one containing plot.py):  ./run_plot.sh

PORT="/dev/ttyACM0"   # ST-LINK V2-1 virtual COM port (USART1)

# Go to the folder this script is in, so .venv and plot.py are found
cd "$(dirname "$0")" || exit 1

# Check the board is connected
if [ ! -e "$PORT" ]; then
    echo "$PORT not found. Is the board plugged in? Available ports:"
    ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
    exit 1
fi

# Kill any process holding the serial port (e.g. screen sessions)
echo "Checking for processes on $PORT..."
sudo fuser -k "$PORT" 2>/dev/null && echo "Killed blocking process(es)" && sleep 1

# Fix permissions
echo "Setting permissions on $PORT..."
sudo chmod 666 "$PORT"

# Create the venv the first time, then activate it
if [ ! -d .venv ]; then
    echo "Creating .venv and installing packages..."
    python3 -m venv .venv
    source .venv/bin/activate
    pip install pyserial matplotlib
else
    source .venv/bin/activate
fi

# Run the plot
echo "Starting plot.py..."
python3 plot.py
