#!/bin/bash
slcand -o -s6 -t hw -S 115200 /dev/serial/by-id/usb-SKAR_USBtoCAN-if00
ip link set up slcan0
echo Can interface is now up.