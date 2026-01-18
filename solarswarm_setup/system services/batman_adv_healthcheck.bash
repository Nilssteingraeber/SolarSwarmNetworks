#!/bin/bash
PATIENCE=5
WAIT=5

inspections=$PATIENCE

while [ 0 ]; do
    if [ $(sudo batctl n | awk "NR>2" | wc -l) -eq 0 ]; then
        inspections=$(($inspections-1))
        # lost patience
        if [ $inspections -eq 0 ]; then
            # restart batman_adv_setup
            sudo systemctl restart batman_adv_setup.service
            inspections=$PATIENCE
        fi
    else
        # echo Found
        inspections=5
    fi
    sleep $WAIT
done
