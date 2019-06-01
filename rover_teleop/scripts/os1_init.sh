#!/bin/bash

# Configure interface for OS1
# sudo ip addr add 192.168.10.1/24 dev eno1
sudo dnsmasq -C /dev/null -kd -F 192.168.10.1,192.168.10.50 -i eno1 --bind-dynamic

# Run PTP
sudo ptp4l -mq -S -i eno1
curl -S http://os1-991901000601.local/api/v1/system/time | jq
