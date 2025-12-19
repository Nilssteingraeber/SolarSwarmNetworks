#!/bin/bash
LOG_OUT="/dev/null"
SLEEP_DUR=3

# WLANDEV="wlp0s20f3"
source /etc/environment
SW_RUN=/home/$MESH_IDENTITY/solarswarm_run

if [ -z $WLANDEV ]; then
  echo "[iw_dump] Error: WLANDEV was not set. Export WLANDEV or use the service setup script." >>$LOG_OUT
  exit 1
fi

if [ -z $MESH_IDENTITY ]; then
  echo "[iw_dump] Error: MESH_IDENTITY was not set. Export MESH_IDENTITY or use the service setup script." >>$LOG_OUT
  exit 1
fi

if [ ! -d /tmp/iw_dump/ ]; then
  echo "[iw_dump] Error: Directory '/tmp/iw_dump/' does not exist"
  exit 1
fi

DUMP="$SW_RUN/tmp/iw_dump/iw_dump.txt" # note that DUMP must be changed manually in robot_util.py to match any changes made to this path
TMP="${DUMP}.tmp"

if [ -f $DUMP ]; then rm $DUMP; fi
if [ -f $TMP ]; then rm $TMP; fi

while [ 0 ]; do
  # write to temporary file, then rename and overwrite dump
  if iw dev $WLANDEV station dump 1>$TMP 2>>$LOG_OUT; then
    mv $TMP $DUMP
  else
    echo "[iw_dump] Error: iw_dump serivce failed to get station dump" >>$LOG_OUT
  fi
  sleep $SLEEP_DUR
done