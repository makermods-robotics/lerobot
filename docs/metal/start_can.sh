#!/usr/bin/env bash
# Bring up the two metal CAN buses: follower=can0, leader=can1, classic CAN @ 1 Mbps.
# Metal motors use classic CAN 2.0B (use_can_fd=False), 1 Mbps.
# Native SocketCAN path (a real CAN adapter, e.g. Jetson CAN). For the DM-USB2FDCAN
# USB adapter over slcan, use `slcand` instead and set can_interface="slcan" in the config.
set -e
for ifc in can0 can1; do
  sudo ip link set "$ifc" down 2>/dev/null || true
  sudo ip link set "$ifc" type can bitrate 1000000
  sudo ip link set "$ifc" up
  echo "$ifc up @ 1 Mbps (classic CAN)"
done
ip -details -brief link show can0
ip -details -brief link show can1
