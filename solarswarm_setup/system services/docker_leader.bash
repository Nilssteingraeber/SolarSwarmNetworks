#!/bin/bash
source /etc/environment
LEADER_CHECK_DELAY=15 # in seconds
LEADER_PATIENCE=4
# minimum of LEADER_PATIENCE * LEADER_CHECK_DELAY total seconds (1,5 minutes by default)
QUORUM_LOSS_TOLERANCE=30
# demotion after LEADER_PATIENCE failed pings
IDEAL_MANAGER_COUNT=1
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

check_duplicate_hostnames() {
    # check for and remove duplicats of hostname given as parameter
    # return number of remaining duplicates or -1 if something goes wrong
    if [ -z $1 ]; then
        echo "[docker_leader] Check for duplicate hostnames failed: No hostname was provided" >>$LOG_OUT
        return -1
    fi

    occurances=$(sudo docker node ls --filter "name=$1" --format "{{.Hostname}}" | wc -w)
    if [ -z $occurances ]; then # docker node ls failed
        return -1
    elif [ $occurances -le 1 ]; then # no duplicates
        return 0
    fi

    # at least one duplicate
    nodes_ready=0; nodes_down=0; nodes_unknown=0; nodes_disconnected=0
    if [ $occurances -gt 1 ]; then
        statuses=$(sudo docker node ls --filter "name=$1" --format "{{.ID}}:{{.Status}}:{{.ManagerStatus}}")
        # get number of IDs using the same hostname per status
        for status in $statuses; do
            read node_id status m_status <<< $(echo $status | sed 's/:/ /g')
            if [ -z $node_id ] || [ -z $status ]; then continue; fi # skip empty
            if [ $status == "Ready" ]; then
                ((nodes_ready++))
            elif [ $status == "Down" ]; then
                ((nodes_down++))
            elif [ $status == "Unknown" ]; then
                ((nodes_unknown++))
            elif [ $status == "Disconnected" ]; then
                ((nodes_disconnected++))
            fi
        done
        echo "[docker_leader] Warning: Found $occurances nodes using hostname $name" >>$LOG_OUT
        echo "    Ready       : $nodes_ready" >>$LOG_OUT
        echo "    Down        : $nodes_down" >>$LOG_OUT
        echo "    Unknown     : $nodes_unknown" >>$LOG_OUT
        echo "    Disconnected: $nodes_disconnected" >>$LOG_OUT
        if [ $REMOVE_DUPLICATE_HOSTNAMES == true ] && [ $nodes_ready -le 1 ]; then
            # case: one Ready -> remove all others
            # case: none Ready -> remove all
            for status in $statuses; do
                read node_id status m_status <<< $(echo $status | sed 's/:/ /g')
                echo $node_id $status $m_status
                if [ -z $node_id ] || [ $status == "Ready" ]; then continue; fi # skip Ready
                if [ ! -z $m_status ] && [ ! $m_status == "Reachable" ]; then sudo docker node demote $id; fi # demote if manager
                sudo docker node rm -f $node_id
            done
        elif [ $nodes_ready -gt 1 ]; then
            echo "[docker_leader] Error: Found $nodes_ready 'Ready' nodes using hostname $name. This means that multiple hosts were given the same hostname. If this error persists, please remove all but one of these nodes and set them up again with another name that is not already in use." >>$LOG_OUT
        fi
    fi
    duplicates=$(sudo docker node ls --filter "name=$1" --format "{{.Hostname}}" | wc -w)
    if [ ! -z $duplicates ]; then
        ((duplicates--))
        return $duplicates
    else
        return -1
    fi
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

check_for_health() { # this is a copy from docker_init.bash
    # see check_for_healtch() in docker_init.bash for more details
    if sudo journalctl -u docker -n10 --since "10s ago" | grep -q "No entries" ; then
        return 0 # healthy
    elif ! sudo journalctl -u docker -n10 --since "10s ago" --grep="swarm does not have a leader" | grep -q "No entries"; then
        # note: this was only tested on a manager in a swarm consisting only of two managers
        return 1 # loss of quorum
    elif ! sudo journalctl -u docker -n10 --since "10s ago" --grep="no route to host" | grep -q "No entries"; then
        return 2 # isolated
    elif ! sudo journalctl -u docker -n10 --since "10s ago" --grep="connection refused" | grep -q "No entries"; then
        return 3 # manager left cluster
    elif ! sudo journalctl -u docker -n10 --since "10s ago" --grep="authentication handshake failed" | grep -q "No entries"; then
        return 4 # manager joined different cluster
    else 
        return 255 # unknown case
    fi
    # note: 'var=$(check_for_health)' returns the content of journalctl, not the return code
}

echo "" > $MANAGER_LIST.tmp # new count for tracking patience with managers

echo "[docker_leader] Starting... ($(date +'%D %T'))" >>$LOG_OUT
while [ 0 ]; do
    # update designated leader
    designated_leader=$(head -1 $SW_SETUP/docker/leader)
    if ! check_leader $designated_leader; then designated_leader=""; fi

    # check if self is still leader and stop docker_leader.service if not
    state=$(sudo docker info --format '{{.Swarm.LocalNodeState}}')
    if [ ! -z $state ] && [ ! $state == "inactive" ]; then
        is_manager=$(sudo docker info | awk '/Is Manager:/ {print $3}' 2>/dev/null)
        if [ ! -z $is_manager ] && [ $is_manager == "true" ]; then
            manager_status=$(sudo docker node ls --filter "role=manager" --format "{{.Hostname}} {{.ManagerStatus}}" | grep "$MESH_IDENTITY" | awk '{print $2}')
            if [ $? == 0 ] && [ ! -z $manager_status ] && [ ! $manager_status ==  "Leader" ] && [ ! -z $designated_leader ] && [ ! $MESH_IDENTITY == $designated_leader ]; then
                # if self has quorum, but self is not Leader nor designated leader...
                echo "[docker_leader] No longer leader. Stopping service..." >>$LOG_OUT
                sudo systemctl stop docker_leader.service
                exit 0
            fi
        fi
    fi

    # check for quorum loss
    check_for_health
    code=$? # code 1 for loss of quorum
    if [ ! -z $code ] && [ $code -eq 1 ]; then
        echo "[docker_leader] Detected loss of quorum - management tasks not possible ($(date +'%D %T'))"
        # wait for cluster to fix itself
        error_time=0
        while [ $error_time -le $QUORUM_LOSS_TOLERANCE ]; do
            sleep 5
            error_time=$((error_time + 5))
            check_for_health
            code=$? # check again
            if [ ! -z $code ] && [ ! $code -eq 1 ]; then break; fi
        done

        if [ ! -z $code ] && [ $code -eq 1 ] && [ $MESH_IDENTITY == $designated_leader ]; then
            echo "[docker_leader] Loss of quorum still persists. Forcing new cluster as designated leader... ($(date +'%D %T'))" >>$LOG_OUT
            # eventually force new cluster, requiring all nodes to rejoin
            # rejoining is handled by docker_init
            # as this probably ends all running docker services on those nodes, critical robot functions (such as steering) should not run as a service
            sudo docker swarm init --force-new-cluster --advertise-addr $MESH_IP
            echo "$(sudo docker swarm join-token -q worker 2>/dev/null) $MESH_IP:2377" >$WORKER_TOKEN_LOCATION
            cluster_id=$(sudo docker info | awk '/ClusterID/ {print $2}')
            echo $cluster_id >>$WORKER_TOKEN_LOCATION # add ClusterID beneath token
            echo "[docker_init] Generated worker join token: $(head -1 $WORKER_TOKEN_LOCATION)" >>$LOG_OUT
            # sudo systemctl restart docker_leader.service
            # exit 0
        fi
    fi

    # designated must always run docker_leader - even if not leader
    # designated leader must init new swarm if nevessary, but is no longer responsible for managerer management
    if [ ! -z $designated_leader ] && [ $MESH_IDENTITY == $designated_leader ]; then
        manager_status=$(sudo docker node ls --filter "name=$MESH_IDENTITY" --format {{.ManagerStatus}})
        if [ $? ] && [ ! -z $manager_status ] && [ ! $manager_status == "Leader" ]; then
            echo "[docker_leader] Designated leader is no longer leader - sleeping early... ($(date +%T))" >>$LOG_OUT
            sleep $LEADER_CHECK_DELAY
            continue
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
                    if [ ! -z $own_cluster_id ] && [ ! -z $other_cluster_id ] && [ ! $own_cluster_id == $other_cluster_id ]; then
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
                            sudo systemctl stop docker_leader.service
                            exit 0
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

        nodes_in_cluster=$(sudo docker node ls --format "{{.Hostname}}:{{.Availability}}:{{.ManagerStatus}}")
        if [ $? ]; then # some security measures
            nodes_with_hostname=(echo $nodes_in_cluster | sed 's/ /\n/g' | grep -E "^$hostname:[A-Za-z]+:[A-Za-z]*$" 2>/dev/null)
            occurances=$()

            # check if hostname exists
            if [ $occurances -eq 0 ]; then # hostname not in cluster -> possibly old labels file
                rm $LABELS_TARGET/$file
                continue
            elif [ $occurances -gt 1 ]; then # duplicate hostnames
                continue
            fi

            # prevent update of nodes that are not on pause or a manager
            read c_hostname c_availability c_manager_status <<< $(echo $nodes_with_hostname | sed 's/:/ /g')
            if [ ! -z $c_availability ] && [ $c_availability == "Active" ]; then # is not paused
                rm $LABELS_TARGET/$file
                continue
            elif [ ! -z $c_manager_status ]; then # is manager
                rm $LABELS_TARGET/$file
                continue
            fi
        fi

        check_duplicate_hostnames $hostname # prevent ambiguous hostnames
        if [ ! $? ]; then continue; fi # possibly duplicate hostnames
        
        # workers can not know who is current leader and send labels to all managers
        # check if node already has labels
        if [ $AVOID_ADDING_LABELS_AGAIN == true ]; then
            present_labels=$sudo docker node inspect $hostname --format {{.Spec.Labels}})
            # from testing: "map[]" if none, else "map[KEY:VALUE]"
            if [ $? ] && echo $present_labels | grep ":"; then
                rm $LABELS_TARGET/$file
                continue
            fi
        fi

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

    # echo "[docker_leader] Checking on managers..."

    # check on managers
    # demote unreachable hosts if leader has lost patience
    managers=$(sudo docker node ls --filter "role=manager" --format "{{.Hostname}}:{{.ManagerStatus}}")
    if [ $? ]; then
        for line in $managers; do
            if [ ! -z $line ]; then
                read hostname status <<< $(echo $line | sed 's/:/ /')+
                # check manager status
                if [ $status == "Leader" ] || [ $hostname == $MESH_IDENTITY ] || [ $hostname == $designated_leader ]; then
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
                                echo "[docker_leader] Demoted and removed host $hostname ($(date +'%D %T'))" >>$LOG_OUT
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
    fi

    # promote new hosts if below ideal count
    # note: it would be better to modify this in the future so that an uneven manager count is avoided
    echo "[docker_leader] Checking for promotions..." >>$LOG_OUT
    current_manager_count=""
    current_manager_count=$(sudo docker node ls --filter "role=manager" --format "{{.Hostname}}" | wc -w)
    if [ $? ] && [ ! -z $current_manager_count ] && [ $current_manager_count -lt $IDEAL_MANAGER_COUNT ]; then
        echo "[docker_leader] Found only $current_manager_count managers though $IDEAL_MANAGER_COUNT are desired ($(date +'%D %T'))""
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
