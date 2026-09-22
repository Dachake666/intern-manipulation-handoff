#!/bin/sh
# Debian host only. Read-only, no sudo, no config updates, no robot SDK imports.
printf '=== HOST / TIME ===\n'
hostname
date -Ins
printf '\n=== DEVICES ===\n'
nmcli device status
ip -br addr
printf '\n=== ROUTING POLICY ===\n'
ip rule show
ip route show table main
ip route get 192.168.8.148
ip route get 1.1.1.1
printf '\n=== ETHERNET PROFILE (NO SECRETS) ===\n'
nmcli -f connection.id,connection.interface-name,ipv4.method,ipv4.addresses,ipv4.gateway,ipv4.dns,ipv4.route-metric connection show '有线连接 1'
printf '\n=== WIFI POWER SAVE ===\n'
nmcli -g 802-11-wireless.powersave connection show 'GL-SFT1200-71f-5G'
printf '\n=== ETHERNET COUNTERS ===\n'
ip -s link show enp3s0
