#!/bin/bash
port_name="$(src/sirius_modules/scripts/usb_find_port.sh SKAR_USBtoCAN)"
sudo slcand -o -s6 -t hw -S 115200 $port_name
sudo ip link set up slcan0
echo Can interface up.
