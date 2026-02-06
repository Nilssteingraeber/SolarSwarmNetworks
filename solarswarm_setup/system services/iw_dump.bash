#!/bin/bash
LOG_OUT="/dev/null"
SLEEP_DUR=3

# WLANDEV="wlp0s20f3"
source /etc/environment
SW_RUN=/home/$MESH_IDENTITY/solarswarm_run
TMP_DIR=/var/tmp/solarswarm

if [ -z $WLANDEV ]; then
  echo "[iw_dump] Error: WLANDEV was not set. Export WLANDEV or use the service setup script." >>$LOG_OUT
  exit 1
fi

if [ -z $MESH_IDENTITY ]; then
  echo "[iw_dump] Error: MESH_IDENTITY was not set. Export MESH_IDENTITY or use the service setup script." >>$LOG_OUT
  exit 1
fi

# ensure /var/tmp/solarswarm/iw_dump exists
if [ ! -d $TMP_DIR ]; then
  mkdir $TMP_DIR
  if [ ! $? ]; then exit 1; fi
fi
if [ ! -d $TMP_DIR/iw_dump ]; then
  mkdir $TMP_DIR/iw_dump
  if [ ! $? ]; then exit 1; fi
fi

# names of dump file
# note that the IW_DUMP env of base_robot must be changed manually in solarswarm_run/docker-compose.yaml to match any changes made to this path
DUMP="$TMP_DIR/iw_dump/iw_dump.txt"
TMP="$DUMP.tmp"

if [ -f $DUMP ]; then rm $DUMP; fi
if [ -f $TMP ]; then rm $TMP; fi

while [ 0 ]; do
  # write to temporary file, then rename and overwrite dump
  if iw dev $WLANDEV station dump 1>$TMP 2>>$LOG_OUT; then # if dump successful
    mv $TMP $DUMP # overwrite
    echo moved
  else
    echo "[iw_dump] Error: iw_dump serivce failed to get station dump" #>>$LOG_OUT
  fi
  sleep $SLEEP_DUR
done
