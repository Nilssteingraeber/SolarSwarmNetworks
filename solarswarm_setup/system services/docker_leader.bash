#!/bin/bash
LEADER_CHECK_DELAY=30 # in seconds
LEADER_PATIENCE=60 # LEADER_PATIENCE * LEADER_CHECK_DELAY total seconds
QUORUM_LOSS_TOLERANCE=30
# demotion after LEADER_PATIENCE failed pings
IDEAL_MANAGER_COUNT=3
SW_SETUP=/home/$MESH_IDENTITY/solarswarm_setup
MANAGER_LIST=$SW_SETUP/docker/manager_list
DEMOTION_LIST=$SW_SETUP/docker/demotion_list
LABELS_TARGET=$SW_SETUP/rx/docker
SSH_TIMEOUT="-o ConnectTimeout=1" # see service_helper.bash
HOST_CHECKING="-o StrictHostKeyChecking=no" # see service_helper.bash
LOG_OUT=/dev/stdout # see docker_init.bash
REMOVE_DUPLICATE_HOSTNAMES=true
# check_duplicate_hostnames() removes nodes using the same hostname as another node that is 'Ready'
# only does so if only one node has status 'Ready'
# otherwise a hostname is used my multiple hosts and manual intervention is required

# during testing, bravo frequently rejoined alfa's swarm
# 'sudo docker node ls' then showed multiple nodes using hostname bravo, but with different IDs
# the nodes were not removed when their host bravo left, but instead changed their status to 'Down'
# when bravo (the host) rejoined, it was given a new ID and the status 'Ready'
# commands using the then ambiguous hostname bravo failed

# function to check for and remove duplicate hostnames as hostnames are not unique in a swarm
check_duplicate_hostnames() {
    # optionally pass one hostname as argument to filter, i.e.: 'check_duplicate_hostnames alfa'
    hostnames=$(sudo docker node ls --filter "name=$1" --format "{{.Hostname}}")
    for name in $(cat $SW_SETUP/ssh_identities/names); do
        occurances=$(echo $hostnames | grep -E "$name " | wc -w)
        # hosts_ready=0; hosts_down=0; hosts_unknown=0; hosts_disconnected=0
        if [ $occurances -le 1 ]; then continue; fi
        if [ $occurances -gt 1 ]; then
            statuses=$(sudo docker node ls --filter "name=$name" --format "{{.ID}} {{.Status}}")
            # get number of IDs using the same hostname per status
            # output is assigned as words in a single line
            # grep returns a selection of lines which contain a certain word
            # therefore replace spaces with newline characters, then use grep
            # s to replace, g for global, '/ /\n/' to replace ' ' with '\n'
            hosts_ready=$(echo $statuses | sed "s/ /\n/g" | grep "Ready" | wc -w);
            hosts_down=$(echo $statuses | sed "s/ /\n/g" | grep "Down" | wc -w);
            hosts_unknown=$(echo $statuses | sed "s/ /\n/g" | grep "Unknown" | wc -w);
            hosts_disconnected=$(echo $statuses | sed "s/ /\n/g" | grep "Disconnected" | wc -w);
            echo "[docker_leader] Warning: Found $occurances nodes using hostname $name" >>$LOG_OUT
            echo "    Ready       : $hosts_ready" >>$LOG_OUT
            echo "    Down        : $hosts_down" >>$LOG_OUT
            echo "    Unknown     : $hosts_unknown" >>$LOG_OUT
            echo "    Disconnected: $hosts_disconnected" >>$LOG_OUT
            if [ $REMOVE_DUPLICATE_HOSTNAMES == true ] && [ $hosts_ready -eq 1 ]; then
                sudo docker node ls --filter "name=$name" --format "{{.ID}} {{.Status}}" | \
                while read id status; do
                    # note that it might take a few seconds for the leader to consider a host 'Down' after they left
                    # "Error response from daemon: [...] is not down and can't be removed"
                    
                    # if [ ! $status == "Ready" ]; then # unsure if 'Unknown' or 'Disconnected' count
                    if [ $status == "Down" ]; then
                        if sudo docker node rm -f $id &>/dev/null; then
                            echo "[docker_leader] Removed node $id with duplicate hostname $name and status $status"
                        fi
                    fi
                done
            elif [ $hosts_ready -gt 1 ]; then
                echo "[docker_leader] Error: Found $hosts_ready 'Ready' nodes using hostname $name. This means that multiple hosts were given the same hostname. If this error persists, please remove all but one of these nodes and set them up again with another name that is not already in use." >>$LOG_OUT
            fi
        fi
    done
}

while [ 0 ]; do
    # check if self is still leader and stop docker_leader.service if not
    state=$(sudo docker info --format '{{.Swarm.LocalNodeState}}')
    if [ ! -z $state ] && [ ! $state == "inactive" ]; then
        is_manager=$(sudo docker info | awk '/Is Manager:/ {print $3}' 2>/dev/null)
        if [ ! -z $is_manager ] && [ $is_manager == "true" ]; then
            manager_status=$(sudo docker node ls --filter "role=manager" --format "{{.Hostname}} {{.ManagerStatus}}" | grep "$MESH_IDENTITY" | awk '{print $2}')
            if [ ! -z $manager_status ] && [ ! $manager_status ==  "Leader" ]; then
                echo "[docker_leader] No longer leader. Stopping service..." >>$LOG_OUT
                sudo systemctl stop docker_leader.service
                exit 0
            fi
        fi
    fi

    designated_leader=$(head -1 $SW_SETUP/docker/leader)
    if ! check_leader $designated_leader; then designated_leader=""; fi

    # check for quorum loss
    state=$(sudo docker info --format '{{.Swarm.LocalNodeState}}' 2>/dev/null)
    if [ ! -z $state ] && [ $state == "error" ]; then
        echo "[docker_leader] Node state: error"
        # wait for cluster to fix itself
        error_time=0
        while [ $error_time -le $QUORUM_LOSS_TOLERANCE ]; do
            sleep 1
            ((error_time++))
        done

        state=$(sudo docker info --format '{{.Swarm.LocalNodeState}}')
        if [ ! -z $state ] && [ $state == "error" ] && [ $MESH_IDENTITY == $designated_leader ]; then
            echo "[docker_leader] Node state is still 'error'. Forcing new cluster as designated leader..."
            # eventually force new cluster, requiring all nods to rejoin
            # as this probably ends all running docker services on those nodes, critical robot functions (such as steering) should not run on a stack
            sudo docker swarm init --force-new-cluster --advertise-addr $MESH_IP
            echo "$(sudo docker swarm join-token -q worker 2>/dev/null) $MESH_IP:2377" > $WORKER_TOKEN_LOCATION
            sudo docker info | awk '/ClusterID/ {print 2}' >> $WORKER_TOKEN_LOCATION # Add ClusterID beneath token
            echo "[docker_init] Generated worker join token: $(head -1 $WORKER_TOKEN_LOCATION)" >>$LOG_OUT
            # sudo systemctl restart docker_leader.service
            # exit 0
        fi
    fi

    # imagine a cluster split into two parts, leaving the designated leader in a minority of managers
    # designated leader might start another cluster
    # if the other part with the majority has no loss of quorum, nodes won't attempt to join the new cluster
    # -> the temporary leader must (1) demote all managers, (2) remove all nodes other than himself, (3) leave the swarm forcefully
    # -> docker_init should exit when nodes become inactive as a result of being removed (further research or tests required)
    if [ ! -z $designated_leader ]; then designated_leader_ip=$(grep $leader $SW_SETUP/ssh_identities/names_with_ip | awk '{print $2}'); fi
    if [ ! -z $designated_leader_ip ] && [ ! $MESH_IDENTITY == $designated_leader ]; then
        if sudo docker node ls --filter "role=manager" --format "{{.Hostname}} {{.ManagerStatus}}" | grep "$designated_leader Unreachable"; then
            if ping -W 1 -c 1 $designated_leader_ip &>/dev/null; then
                echo "[docker_leader] Possibility of more than one clusters existing simultaneously detected."
                # designated leader is reachable but possibly not in same cluster
                # check if they are not in the same swarm and have different ClusterIDs
                worker_loc=/home/$designated_leader/solarswarm_setup/docker/worker_token
                if scp $SSH_TIMEOUT $HOST_CHECKING $designated_leader:$worker_loc $SW_SETUP/docker/cluster_check; then
                    own_cluster_id=$(head -2 $SW_SETUP/docker/worker_token | tail -1)
                    other_cluster_id=$(head -2 $SW_SETUP/docker/cluster_check | tail -1)
                    if [ ! -z $own_cluster_id ] && [ ! -z $other_cluster_id] && [ ! $own_cluster_id == $other_cluster_id ]; then
                        # more than one cluster
                        echo "[docker_leader] Self has ClusterID $own_cluster_id while designated leader has ClusterID $other_cluster_id."
                        if [ $purge_second_cluster == true ]; then
                            # demote managers (except self)
                            managers=$(sudo docker node ls --filter "role==manager" --format "{{.Hostname}}")
                            echo "[docker_leader] Current managers: $managers"
                            for manager in $managers; do
                                if [ -z $manager ] || [ $manager == $MESH_IDENTITY ]; then continue; fi
                                sudo docker node demote $manager
                            done

                            # remove nodes
                            hosts=$(sudo docker node ls --format "{{.Hostname}}")
                            echo "[docker_leader] Current hosts: $hosts"
                            for host in $hosts; do
                                if [ -z $host ] || [ $host == $MESH_IDENTITY ]; then continue; fi
                                sudo docker node rm $host
                            done

                            # leave
                            sudo docker swarm leave -f
                        fi
                    fi
                fi    
                if [ -f $SW_SETUP/docker/cluster_check ]; then rm $SW_SETUP/docker/cluster_check; fi
            fi
        fi
    fi

    
    # echo "[docker_leader] Starting loop..." >>$LOG_OUT
    # add labels and activate nodes
    # when a worker joins, it is paused and leaves a labels file (even if empty)
    echo "[docker_leader] Looking for .labels files..."
    for file in $(ls $LABELS_TARGET | grep .labels); do
        make_active=true
        hostname=$(echo $file | sed "s/.labels//")
        check_duplicate_hostnames $hostname # prevent ambiguous hostnames
        if [ ! -z $hostname ] && [ $hostname == $MESH_IDENTITY ]; then # skip own name
            echo "[docker_leader] Warning: Found labels file of node $hostname (own name)" >>$LOG_OUT
            continue
        elif grep -E "^$hostname$" $SW_SETUP/ssh_identities/names; then
            echo "[docker_leader] Found labels file of node $hostname" >>$LOG_OUT
        else # skip unknown name
            echo "[docker_leader] Warning: Found labels file of node $hostname (unknown name)" >>$LOG_OUT
            continue
        fi
        
        # sanitize file for better security
        # replace ';', '&', '\' and '|' with ''
        sed -i "s/;//g; s/&//g; s/\\\//g; s/|//g;" $LABELS_TARGET/$file
        
        # add labels to node
        for label in $(cat $LABELS_TARGET/$file); do
            echo "[docker_leader] Found label: $label"
            if ! sudo docker node update --label-add $label $hostname &>/dev/null; then
                echo "[docker_leader] Error: Failed to add label to node"
                make_active=false # make false if even one label could not be added
            fi 
        done
        
        # attempt to update availability
        if [ $make_active == true ]; then
            if sudo docker node update --availability active $hostname &>/dev/null; then
                rm $LABELS_TARGET/$file
                echo "[docker_leader] Activated node $hostname"
            else
                echo "[docker_leader] Error: Failed to activate node $hostname"
            fi
        fi
    done
    
    #####
    # TODO: remove echo, sleep, and continue below and test administering managers 
    echo "[docker_leader] Debug: Loop cycle done (skipped managers) - sleeping for $LEADER_CHECK_DELAY seconds" >>$LOG_OUT
    sleep $LEADER_CHECK_DELAY
    continue
    #####

    echo "[docker_leader] Checking on managers..."
    # write newest state with names (hostnames) and count to manager_list
    echo "" > $MANAGER_LIST.tmp
    echo "" > $DEMOTION_LIST.tmp # obsolete?

    # check on managers
    node_ls=$(sudo docker node ls --filter "role=manager" --format "{{.Hostname}} {{.Status}}")
    line_count=$(echo $node_ls | wc -l)
    for i in $(seq 1 $line_count); do
        read hostname status <<< $(echo $node_ls | head -$i | tail -1)
        # for instance, assign "example_name 0" to variables
        # status can be: Ready, Down, Unknown, or Disconnected
        if [ -f $MANAGER_LIST ]; then # check if manager_list exists
            host_count=$(grep -E "^$hostname [0-9]+\$" $MANAGER_LIST)
            if [ $(cat $host_count | wc -l) -ge 2 ]; then
                echo "[docker_leader] Warning: Found multiple entries of node $hostname in manager_list - first entry will be used" >>$LOG_OUT
            fi
            count=$(grep -E "^$hostname [0-9]+$" $MANAGER_LIST | awk '{print $2}' $MANAGER_LIST)
            # filter hostname, get second column
            echo "[docker_leader]" >>$LOG_OUT
            echo "[docker_leader] Manager in list: $hostname" >>$LOG_OUT
        else
            touch $MANAGER_LIST
            count=0
        fi

        if [ $status == "Ready" ]; then # regain patience
            count=0 # TODO: secure quorum; might just demote immediately
        else
            ((count++))
        fi

        if [ $count -le $LEADER_PATIENCE ]; then
            echo "$hostname $count" >> $MANAGER_LIST.tmp
            echo "[docker_leader] Added node $hostname to manager_list.tmp" >>$LOG_OUT
        else
            # leader lost all patience with manager
            echo "$hostname" >> $DEMOTION_LIST.tmp
            echo "[docker_leader] Added node $hostname to demotion_list.tmp" >>$LOG_OUT
        fi
    done
    
    # try to demote hosts on demotion list
    echo "[docker_leader] Demoting managers already on demotion_list..." >>$LOG_OUT
    for i in $(cat $DEMOTION_LIST); do
        read hostname <<< $i
        if ! sudo docker node demote $hostname; then
            # failed to demote -> remains on list
            echo "$hostname" >> $DEMOTION_LIST.tmp
            echo "[docker_leader] Error: Failed to demote node $hostname" >>$LOG_OUT
        else
            sudo docker node rm $hostname
            echo "[docker_leader] Demoted node $hostname" >>$LOG_OUT
        fi
    done

    # promote new hosts if below ideal count
    echo "[docker_leader] Checking for promotions..." >>$LOG_OUT
    node_ls=$(sudo docker node ls --filter "role=worker" \
        --filter "node.label=can_become_manager" \
        --format "{{.Hostname}} {{.Status}}" &>/dev/null) # get workers with label can_become_manager
    line_count=$(echo $node_ls | wc -l)
    for i in $(seq 1 $line_count); do
        # for each healthy worker: update current managers
        managers=$(sudo docker node ls --filter "role=manager" --format "{{.Hostname}} {{.Status}}")
        if [ $(echo $managers | wc -l) -ge $IDEAL_MANAGER_COUNT ]; then # enough managers
            break
        fi
        
        echo "[docker_leader] Attempting to promote node $hostname..." >>$LOG_OUT
        read hostname status <<< $(echo $node_ls | head -$i | tail -1)
        if [ $status == "Ready" ]; then
            if sudo docker promote $hostname &>/dev/null; then
                echo "$hostname 0" >> $MANAGER_LIST
                echo "[docker_leader] Promoted node $hostname and added it to manger_list" >>$LOG_OUT
            fi
        else
            echo "[docker_leader] Error: Failed to promote $hostname" >>$LOG_OUT
        fi
    done
    # https://docs.docker.com/reference/cli/docker/node/ls/

    mv $MANAGER_LIST.tmp $MANAGER_LIST
    mv $DEMOTION_LIST.tmp $DEMOTION_LIST
    
    echo "[docker_leader] Loop cycle done - sleeping for $LEADER_CHECK_DELAY seconds" >>$LOG_OUT
    sleep $LEADER_CHECK_DELAY
done
