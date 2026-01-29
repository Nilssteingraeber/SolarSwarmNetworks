#!/bin/bash
PATIENCE=5
WAIT=5

inspections=$PATIENCE

# two cases have been observed

while [ 0 ]; do
    if [ $(sudo batctl n | awk "NR>2" | wc -l) -eq 0 ]; then
        # no neighbors found (0 lines)
        if [ ! $? ]; then
            # batctl failed
            inspections=0
        else
            inspections=$(($inspections-1))
        fi

        if [ $inspections -eq 0 ]; then
            # lost all patience
            # restart batman_adv_setup
            sudo systemctl restart batman_adv_setup.service
            sleep 10
            # reset patience
            inspections=$PATIENCE
        fi
    else
        # at least one neighbor found
        # regain patience
        inspections=5
    fi
    sleep $WAIT
done
