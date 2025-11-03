#!/bin/bash

# WLANDEV="wlp0s20f3"
source /etc/environment
if [ $(echo $WLANDEV | wc --chars) -eq 1 ]; then
  >&2 echo "Error: WLANDEV was not set. Export WLANDEV or use the service setup script."
  exit 1
fi

DUMP="/tmp/iw_dump/iw_dump.txt" # note that DUMP must be changed manually in robot_util.py to match any changes made to this path
TMP="${DUMP}.tmp"

if iw dev $WLANDEV station dump > $TMP; then
  mv $TMP $DUMP
else
  echo "Error: iw_dump serivce failed to get station dump"
fi
