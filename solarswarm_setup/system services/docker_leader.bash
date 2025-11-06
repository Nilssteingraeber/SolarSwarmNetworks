#!/bin/bash
LEADER_CHECK_DELAY=30 # in seconds
LEADER_PATIENCE=60 # LEADER_PATIENCE * LEADER_CHECK_DELAY total seconds
# demotion after LEADER_PATIENCE failed pings
MAX_MANAGER_COUNT=5
MANAGER_LIST=~/solarswarm_setup/docker/manager_list
DEMOTION_LIST=~/solarswarm_setup/docker/demotion_list
LABELS_TARGET=~/solarswarm_setup/rx/docker

while [ 0 ]; do
    # add labels and activate nodes
    # when a worker joins, it is paused and leaves a labels file (even if empty) 
    for file in $(ls $LABELS_TARGET | grep .labels); do
        hostname=$(echo $file | sed "s/.labels//")
        for label in $(cat file); do
            sudo docker node update --label-add $label $hostname &>/dev/null
        done
        sudo docker node update --availability active $hostname &>/dev/null
        rm $LABELS_TARGET/$file
    done

    # write newest state with names (hostnames) and count to manager_list
    echo "" > $MANAGER_LIST.tmp
    echo "" > $DEMOTION_LIST.tmp

    # check on managers
    for i in $(sudo docker node ls --filter "role=manager" --format {{.Hostname}} {{.Status}}); do
        read hostname status <<< $i # for instance, assign "example_name 0" to variables
        # status can be: Ready, Down, Unknown, or Disconnected
        if [ -f $MANAGER_LIST ]; then # check if manager_list exists
            count=$(awk -F " " '/$hostname/ {print $2}' $MANAGER_LIST)
        else
            count=0
        fi

        if [ $status == "Ready" ]; then # regain patience
            count=0 # TODO: secure quorum; might just demote immediately
        else
            ((count++))
        fi

        if [ $count -le $LEADER_PATIENCE ]; then
            echo "$hostname $count" >> $MANAGER_LIST.tmp
        else
            # leader lost all patience with manager
            echo "$hostname" >> $DEMOTION_LIST.tmp
        fi
    done

    # try to demote hosts on demotion list
    for i in $(cat $DEMOTION_LIST); do
        read hostname <<< $i
        if ! sudo docker node demote $hostname; then
            # failed to demote -> remains on list
            echo "$hostname" >> $DEMOTION_LIST.tmp
            echo "Error: Failed to demote $hostname"
        else
            sudo docker node rm $hostname
        fi
    done

    # promote new hosts if below maximum count
    for i in $(sudo docker node ls --filter "role=worker" \
        --filter "node.label=can_become_manager" \
        --format {{.Hostname}} {{.Status}} &>/dev/null
    ); do # for each healthy worker
        # update current managers
        managers=$(sudo docker node ls --filter "role=worker" --format {{.Hostname}} {{.Status}})
        if [ $(wc -l $managers) -lt $MAX_MANAGER_COUNT ]; then break; fi

        read hostname status <<< $i
        if [ $status == "Ready" ]; then
            if sudo docker promote $hostname &>/dev/null; then
                echo "$hostname 0" >> $MANAGER_LIST
            fi
        else
            echo "Error: Failed to promote $hostname"
        fi
    done
    # https://docs.docker.com/reference/cli/docker/node/ls/

    mv $MANAGER_LIST.tmp $MANAGER_LIST
    mv $DEMOTION_LIST.tmp $DEMOTION_LIST

    sleep $LEADER_CHECK_DELAY
done
