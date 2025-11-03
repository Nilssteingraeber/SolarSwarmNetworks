#!/bin/bash
LEADER_CHECK_DELAY=60 # in seconds
LEADER_PATIENCE=30 # represents patience in minutes if LEADER_CHECK_DELAY is 60
# demotion after LEADER_PATIENCE failed pings -> LEADER_PATIENCE * LEADER_CHECK_DELAY seconds
MAX_MANAGER_COUNT=5
MANAGER_LIST=~/solarswarm_setup/docker/manager_list # alternatively in /tmp/
DEMOTION_LIST=~/solarswarm_setup/docker/demotion_list

while [ 0 ]; do
    # write newest state with names (hostnames) and count to manager_list
    echo "" > $MANAGER_LIST.tmp
    echo "" > $DEMOTION_LIST.tmp

    # check on managers
    for i in $(cat $MANAGER_LIST); do
        read hostname ip count <<< $i # assign "example_name 192.168.1.100 0" to vars
        if ping -W 1 -c $ip &>/dev/null; then # -W for timeout
            count=0
        else
            ((count++))
        fi

        if [ $count -le $LEADER_PATIENCE ]; then
            echo "$hostname $ip $count" >> $MANAGER_LIST.tmp
        else
            # leader lost all patience with manager
            echo "$hostname $ip" >> $DEMOTION_LIST.tmp
        fi
    done

    # try to demote hosts on demotion list
    for i in $(cat $DEMOTION_LIST); do
        read hostname ip <<< $i
        if ping -W 1 -c $ip &>/dev/null; then
            if ! sudo docker node demote $hostname; then
                # failed to demote -> remains on list
                echo "$hostname $ip" >> $DEMOTION_LIST.tmp
            else
                echo "Error: Failed to promote $hostname"
            fi
        fi
    done

    # promote new hosts if below maximum count
    if [ $(wc -l $MANAGER_LIST 2>/dev/null) -lt $MAX_MANAGER_COUNT ]; then
        for i in $(cat ~/solarswarm_setup/ssh_identities/names_with_ip); do
            read hostname ip <<< $i
            if grep $i $MANAGER_LIST &>/dev/null; then
                # already manager
                continue
            else
                if sudo docker node promote $hostname; then
                    echo "$hostname $ip 0" >> $MANAGER_LIST.tmp
                else
                    echo "Error: Failed to promote $hostname"
                fi
            fi
        done
    fi

    mv $MANAGER_LIST.tmp $MANAGER_LIST
    mv $DEMOTION_LIST.tmp $DEMOTION_LIST

    sleep $LEADER_CHECK_DELAY
done