#!/bin/bash
# Test script pre kontrolu serial komunikácie s ESP32

SERIAL_PORT="${1:-/dev/ttyAMA10}"
BAUD_RATE=115200

echo "Testing serial connection to ESP32"
echo "Port: $SERIAL_PORT"
echo "Baud rate: $BAUD_RATE"
echo ""

# Skontroluj, či port existuje
if [ ! -e "$SERIAL_PORT" ]; then
    echo "ERROR: Serial port $SERIAL_PORT does not exist!"
    exit 1
fi

# Skontroluj oprávnenia
if [ ! -r "$SERIAL_PORT" ] || [ ! -w "$SERIAL_PORT" ]; then
    echo "WARNING: Permission issues. Try: sudo chmod 666 $SERIAL_PORT"
    echo "Or add user to dialout: sudo usermod -a -G dialout \$USER"
fi

# Nastav baud rate
stty -F "$SERIAL_PORT" $BAUD_RATE cs8 -cstopb -parenb raw -echo

echo "Sending test commands..."
echo ""

# Test 1: Stop command
echo "1. Sending STOP command:"
echo '{"T":11,"L":0,"R":0}' > "$SERIAL_PORT"
sleep 0.5

# Test 2: Forward command
echo "2. Sending FORWARD command (slow):"
echo '{"T":11,"L":64,"R":64}' > "$SERIAL_PORT"
sleep 0.5

# Test 3: Backward command
echo "3. Sending BACKWARD command (slow):"
echo '{"T":11,"L":-64,"R":-64}' > "$SERIAL_PORT"
sleep 0.5

# Test 4: Stop again
echo "4. Sending STOP command:"
echo '{"T":11,"L":0,"R":0}' > "$SERIAL_PORT"
sleep 0.5

echo ""
echo "Test commands sent. Check if ESP32 responds."
echo ""
echo "To monitor serial output, run in another terminal:"
echo "  cat $SERIAL_PORT"
echo ""
echo "Or use minicom:"
echo "  minicom -D $SERIAL_PORT -b $BAUD_RATE"

