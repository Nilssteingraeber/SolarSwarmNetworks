#!/bin/bash
echo "[docker_leader] Starting... ($(date +%T))" >>$LOG_OUT
LEADER_CHECK_DELAY=15 # in seconds
LEADER_PATIENCE=4
# minimum of LEADER_PATIENCE * LEADER_CHECK_DELAY total seconds (1,5 minutes by default)
QUORUM_LOSS_TOLERANCE=30
# demotion after LEADER_PATIENCE failed pings
IDEAL_MANAGER_COUNT=3
SW_SETUP=/home/$MESH_IDENTITY/solarswarm_setup
MANAGER_LIST=$SW_SETUP/docker/manager_list
RESET_PATIENCE=true # 
LABELS_TARGET=$SW_SETUP/rx/docker
SSH_TIMEOUT="-o ConnectTimeout=1" # see service_helper.bash
HOST_CHECKING="-o StrictHostKeyChecking=no" # see service_helper.bash
LOG_OUT=$SW_SETUP/logs/docker_leader.log # see docker_init.bash
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

remove_host_from_manager_list() {
    if [ ! -z $1 ]; then
        sed -i -E "/^$1/d" $MANAGER_LIST
    fi
}

check_leader() {
    leader=$1 # first (and only) argument is leader, i.e.: 'check_leader alfa' -> 'leader="alfa"' 
    if [ ! -z $leader ] && grep -E "^$leader$" $SW_SETUP/ssh_identities/names &>/dev/null; then
        return 0 # valid leader
    fi 
    return 1
}

source /etc/environment

echo "" > $MANAGER_LIST.tmp # new count for tracking patience with managers

echo "[docker_leader] Starting... ($(date +%T))" >>$LOG_OUT
while [ 0 ]; do
    # check if self is still leader and stop docker_leader.service if not
    state=$(sudo docker info --format '{{.Swarm.LocalNodeState}}')
    if [ ! -z $state ] && [ ! $state == "inactive" ]; then
        is_manager=$(sudo docker info | awk '/Is Manager:/ {print $3}' 2>/dev/null)
        if [ ! -z $is_manager ] && [ $is_manager == "true" ]; then
            manager_status=$(sudo docker node ls --filter "role=manager" --format "{{.Hostname}} {{.ManagerStatus}}" | grep "$MESH_IDENTITY" | awk '{print $2}')
            if [ $? == 0 ] && [ ! -z $manager_status ] && [ ! $manager_status ==  "Leader" ]; then
                # if self has quorum, but self is not Leader...
                echo "[docker_leader] No longer leader. Stopping service..." >>$LOG_OUT
                sudo systemctl stop docker_leader.service
                exit 0
            fi
        fi
    fi

    designated_leader=$(head -1 $SW_SETUP/docker/leader)
    if ! check_leader $designated_leader; then designated_leader=""; fi

    # check for quorum loss
    # TODO: change method of detecting error
    error=$(sudo docker info --format '{{.Swarm.Error}}' 2>/dev/null)
    if [ ! -z $error ]; then
        echo "[docker_leader] Swarm error: $error"
        # wait for cluster to fix itself
        error_time=0
        while [ $error_time -le $QUORUM_LOSS_TOLERANCE ]; do
            sleep 1
            ((error_time++))
        done

        # TODO: change method of detecting error
        error=$(sudo docker info --format '{{.Swarm.Error}}')
        if [ ! -z $error ] && [ $MESH_IDENTITY == $designated_leader ]; then
            echo "[docker_leader] Swarm error still persists. Forcing new cluster as designated leader..." >>$LOG_OUT
            # eventually force new cluster, requiring all nodes to rejoin
            # rejoining is handled by docker_init
            # as this probably ends all running docker services on those nodes, critical robot functions (such as steering) should not run on a stack
            sudo docker swarm init --force-new-cluster --advertise-addr $MESH_IP
            echo "$(sudo docker swarm join-token -q worker 2>/dev/null) $MESH_IP:2377" > $WORKER_TOKEN_LOCATION
            sudo docker info | awk '/ClusterID/ {print 2}' >> $WORKER_TOKEN_LOCATION # Add ClusterID beneath token
            echo "[docker_init] Generated worker join token: $(head -1 $WORKER_TOKEN_LOCATION)" >>$LOG_OUT
            # sudo systemctl restart docker_leader.service
            # exit 0
        fi
    fi

    # picture a cluster split into two parts, leaving the designated leader in a minority of managers
    # designated leader might start another cluster
    # if the other part with the majority has no loss of quorum, nodes won't attempt to join the new cluster
    # -> the temporary leader must (1) demote all managers, (2) remove all nodes other than himself, (3) leave the swarm forcefully
    # -> docker_init should exit when nodes become inactive as a result of being removed (further research or tests required)
    if [ ! -z $designated_leader ]; then designated_leader_ip=$(grep $leader $SW_SETUP/ssh_identities/names_with_ip | awk '{print $2}'); fi
    if [ ! -z $designated_leader_ip ] && [ ! $MESH_IDENTITY == $designated_leader ]; then
        if sudo docker node ls --filter "role=manager" --format "{{.Hostname}} {{.ManagerStatus}}" | grep "$designated_leader Unreachable"; then
            if ping -W 1 -c 1 $designated_leader_ip &>/dev/null; then
                echo "[docker_leader] Detected possibility of more than one clusters existing simultaneously."
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
                                sudo docker node rm $host -f
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
                echo "[docker_leader] Error: Failed to add label $label to node $hostname"
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
    # echo "[docker_leader] Debug: Loop cycle done (skipped managers) - sleeping for $LEADER_CHECK_DELAY seconds" >>$LOG_OUT
    # sleep $LEADER_CHECK_DELAY
    # continue
    #####

    echo "[docker_leader] Checking on managers..."

    # check on managers
    # demote unreachable hosts if leader has lost patience
    # error=$(sudo docker info --format '{{.Swarm.Error}}')
    if [ -z $error ]; then # TODO: change method of detecting errors
        managers=$(sudo docker node ls --filter "role=manager" --format "{{.Hostname}}:{{.ManagerStatus}}")
        echo $managers
    fi
    for line in $managers; do
        if [ ! -z $line ]; then
            read hostname status <<< $(echo $line | sed 's/:/ /')
            # check manager status
            if [ $status == "Leader" ] || [ $hostname == $MESH_IDENTITY ]; then
                continue
            fi
            if [ $status == "Unreachable" ]; then
                # add to list or update
                if ! grep -E "^$hostname [0]+$" $MANAGER_LIST; then # not on list yet
                    echo "$hostname 0" >> $MANAGER_LIST
                else # already on list
                    # filter for hostname, only take first occurance, then get count
                    count=$(grep -E "^$hostname [0-9]+$" $MANAGER_LIST | head -1 | awk '{print $2}')
                    new_count=$(($count+1))
                    if [ $new_count -ge $LEADER_PATIENCE ]; then
                        echo "[docker_leader] Lost patience with host $hostname... ($(date +%T))" >>$LOG_OUT
                        if sudo docker node demote $hostname && sudo docker node rm $hostname; then
                            # successfully removed from swarm
                            remove_host_from_manager_list $hostname
                            echo "[docker_leader] Demoted and removed host $hostname ($(date +%T))" >>$LOG_OUT
                        fi
                    else
                        sed -i "s/^$hostname $count$/$hostname $new_count/" $MANAGER_LIST # replace old count
                    fi
                fi
            else # Reachable ("role=manager" already filters workers)
                if [ $RESET_PATIENCE == true ]; then
                    remove_host_from_manager_list $hostname
                fi
            fi # check manager status
        fi
    done

    # promote new hosts if below ideal count
    echo "[docker_leader] Checking for promotions..." >>$LOG_OUT
    current_manager_count=""
    current_manager_count=$(sudo docker node ls --filter "role=manager" --format "{{.Hostname}}" | wc -w)
    if [ ! -z $current_manager_count ] && [ $current_manager_count -lt $IDEAL_MANAGER_COUNT ]; then
        echo "[docker_leader] Found only $current_manager_count managers though $IDEAL_MANAGER_COUNT are desired ($(date +%T))"
        node_ls=$(sudo docker node ls --filter "role=worker" \
            --filter "node.label=can_become_manager=true" \
            --format "{{.Hostname}}:{{.Status}}") # get workers with label can_become_manager=true
        for line in $node_ls; do
            read hostname status <<< $(echo $line | sed 's/:/ /')
            if [ $status == "Ready" ]; then
                if sudo docker node promote $hostname; then # check again
                    current_manager_count=$(sudo docker node ls --filter "role=manager" --format "{{.Hostname}}" | wc -w)
                fi
                if [ $current_manager_count -ge $IDEAL_MANAGER_COUNT ]; then
                    break
                fi
            fi
        done
    fi
    # https://docs.docker.com/reference/cli/docker/node/ls/
    
    echo "[docker_leader] Loop cycle done - sleeping for $LEADER_CHECK_DELAY seconds" >>$LOG_OUT
    sleep $LEADER_CHECK_DELAY
done
