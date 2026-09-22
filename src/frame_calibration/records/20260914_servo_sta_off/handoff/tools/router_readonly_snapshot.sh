#!/bin/sh
# Router root shell only. Read-only, does not change UCI, interfaces or radios.
# Capture output outside the router when convenient. No passwords are requested here.
printf '=== ROUTER TIME ===\n'
date
printf '\n=== UPTIME ===\n'
cat /proc/uptime
printf '\n=== STA CONFIG (SAFE FIELDS ONLY) ===\n'
for key in mode device network ssid ifname disabled; do
    printf 'wireless.sta.%s=' "$key"
    uci -q get "wireless.sta.$key" || printf '<unset or unavailable>\n'
done
printf '\n=== PENDING STA DISABLED CHANGE ONLY ===\n'
uci changes wireless 2>/dev/null | grep 'wireless.sta.disabled' || true
printf '\n=== INTERFACES ===\n'
iw dev
printf '\n=== UPSTREAM LINK ===\n'
iw dev wlan-sta0 link 2>&1 || true
printf '\n=== ROBOT ASSOCIATION ===\n'
iw dev wlan1 station get 9a:57:82:ae:10:42 2>&1 || true
iwinfo wlan1 assoclist 2>&1 || true
printf '\n=== ROUTES ===\n'
ip route 2>&1 || route -n
