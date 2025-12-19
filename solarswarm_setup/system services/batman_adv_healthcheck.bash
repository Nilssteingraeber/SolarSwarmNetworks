#!/bin/bash
PATIENCE=5
WAIT=5

inspections=$PATIENCE

while [ 0 ]; do
    if [ $(sudo batctl n | awk "NR>2" | wc -l) -eq 0 ]; then
        inspections=$(($inspections-1))
        # echo Lost patience
        if [ $inspections -eq 0 ]; then
            # echo Restarting batman
            sudo systemctl restart batman_adv_setup.service
            inspections=$PATIENCE
        fi
    else
        # echo Found
        inspections=5
    fi
    sleep $WAIT
done
