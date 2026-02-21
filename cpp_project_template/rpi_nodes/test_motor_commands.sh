#!/bin/bash
# Test script pre posielanie motor príkazov

echo "Test motor príkazov pre ESP32 bridge node"
echo ""
echo "Formát: ros2 topic pub /bpc_prp_robot/set_motor_speeds std_msgs/msg/UInt8MultiArray \"{data: [left, right]}\""
echo ""
echo "Hodnoty: 0-255, kde 128 = stop"
echo "  < 128 = dozadu (128-0 = -128 až 0 PWM)"
echo "  > 128 = dopredu (129-255 = 1 až 127 PWM)"
echo ""

# Príklady príkazov
echo "Príklady:"
echo ""
echo "1. Stop (oba motory):"
echo "   ros2 topic pub --once /bpc_prp_robot/set_motor_speeds std_msgs/msg/UInt8MultiArray \"{data: [128, 128]}\""
echo ""
echo "2. Dopredu (polovičná rýchlosť):"
echo "   ros2 topic pub --once /bpc_prp_robot/set_motor_speeds std_msgs/msg/UInt8MultiArray \"{data: [192, 192]}\""
echo ""
echo "3. Dozadu (polovičná rýchlosť):"
echo "   ros2 topic pub --once /bpc_prp_robot/set_motor_speeds std_msgs/msg/UInt8MultiArray \"{data: [64, 64]}\""
echo ""
echo "4. Otáčanie vľavo (ľavý dozadu, pravý dopredu):"
echo "   ros2 topic pub --once /bpc_prp_robot/set_motor_speeds std_msgs/msg/UInt8MultiArray \"{data: [64, 192]}\""
echo ""
echo "5. Otáčanie vpravo (ľavý dopredu, pravý dozadu):"
echo "   ros2 topic pub --once /bpc_prp_robot/set_motor_speeds std_msgs/msg/UInt8MultiArray \"{data: [192, 64]}\""
echo ""
echo "6. Kontinuálne posielanie (každú sekundu):"
echo "   ros2 topic pub -r 1 /bpc_prp_robot/set_motor_speeds std_msgs/msg/UInt8MultiArray \"{data: [192, 192]}\""
echo ""

