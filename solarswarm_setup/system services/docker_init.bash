#!/bin/bash
source /etc/environment # load permanent envrionment variables such as MESH_IDENTITY and MESH_IP
SW_SETUP=/home/$MESH_IDENTITY/solarswarm_setup # absolute path to solarswarm_setup/
SW_RUN=/home/$MESH_IDENTITY/solarswarm_run # absolute path to solarswarm_run/
    # note: 'solarswarm_run' instead of 'solarswarm_run/' so the path can be extended, i.e.: $SW_RUN/docker-compose.yaml
STACK_NAME="SolarSwarm" # determines what the stack (definition of docker services should be called)
LEADER_TARGET=$SW_SETUP/docker # only for local host!
LEADER_LOCATION=$SW_SETUP/docker/leader # path to leader file - only for local host!
WORKER_TOKEN_LOCATION=$SW_SETUP/docker/worker_token # only for local host!
MANAGER_TOKEN_LOCATION=$SW_SETUP/docker/manager_token # only for local host!
    # ^ $SW_SETUP contains MESH_IDENTITY!
    # scp will look for the wrong home directory on different hosts
LABELS_LOCATION=$SW_SETUP/docker/$MESH_IDENTITY.labels
IGNORE_LABELS_MISSING=false
HEALTHCHECK_DELAY=10
UNKNOWN_ERROR_TOLERANCE=3 # how many unknown errors are allowed before a node checks for cluster change
SSH_TIMEOUT="-o ConnectTimeout=1" # see service_helper.bash
HOST_CHECKING="-o StrictHostKeyChecking=no" # see service_helper.bash
LOG_OUT=$SW_SETUP/logs/docker_init.log # also defined in docker_leader.bash and rx_copy.bash


if [ -z $MESH_IDENTITY ]; then # check if name is set
    echo "Error: MESH_IDENTITY has not been set"
    exit 1
fi

if [ -z $MESH_IP ]; then # check if ip is set
    echo "Error: MESH_IDENTITY has not been set"
    exit 1
fi

check_leader() {
    leader=$1 # first (and only) argument is leader, i.e.: 'check_leader alfa' -> 'leader="alfa"' 
    if [ ! -z $leader ] && grep -E "^$leader$" $SW_SETUP/ssh_identities/names &>/dev/null; then
        return 0 # valid leader
    fi 
    return 1
}

generate_worker_token() {
    echo "$(sudo docker swarm join-token -q worker 2>/dev/null) $MESH_IP:2377" > $WORKER_TOKEN_LOCATION
    cluster_id=$(sudo docker info | awk '/ClusterID/ {print $2}')
    echo $cluster_id >>$WORKER_TOKEN_LOCATION # add ClusterID beneath token
    echo "[docker_init] Generated worker join token: $(head -2 $WORKER_TOKEN_LOCATION | tail -1)" >>$LOG_OUT
}

add_own_labels() {
    echo "[docker_init] Looking for .labels file..."
    labels_target="$SW_SETUP/docker/$MESH_IDENTITY.labels"
    if [ ! -f $labels_target ]; then
        echo "[docker_init] Found no .labels file for self"
        successful=false
    else
        # sanitize file for better security
        # replace ';', '&', '\' and '|' with ''
        sed -i "s/;//g; s/&//g; s/\\\//g; s/|//g;" $labels_target
        # add labels to node
        successful=true
        for label in $(cat $labels_target); do
            label=$(echo $label | sed 's/"//g') # value `"test"` would become `\"test\"`, as shown by docker node inspect
            echo "[docker_leader] Found label: $label"
            if ! sudo docker node update --label-add $label $MESH_IDENTITY &>/dev/null; then
                echo "[docker_leader] Error: Failed to add label $label to self"
                successful=false # make false if even one label could not be added
            fi 
        done
    fi
    if [ $successful == true ]; then return 0; else return 1; fi
}

deploy_stack() {
    echo "[docker_init] Deploying stack..."
    echo "[docker_init] Debug: Skipping"
    if [ 0 ]; then # TODO: change to 1 when testing is done
        echo ""
        # TODO: stack deploy command below
        # sudo docker stack deploy -c $SW_RUN/docker-compose.yaml -c $SW_RUN/docker-stack.yaml $STACK_NAME
    fi
}

check_for_health() {
    # this solution uses simple arguments that are shown in the man pages for journalctl
    # this solution assumes that the listed cases always produce entries containing the expressions used to filter
    # it is unclear how this might behave if a worker reaches some managers, but not all
    # it is unclear how this might behave if a worker reaches managers without quorum
    
    # -u docker to select unit, -n10 to get last 10 entries, --since to get recent entries, --grep to filter error messages
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
        # note: -1 is returned as 255
    fi
    # note: 'var=$(check_for_health)' returns the content of journalctl, not the return code
}

# check if docker is active - was inactive during some tests on charlie after boot
if [ ! $(sudo systemctl is-active docker) == "active" ]; then sudo systemctl restart docker; fi

echo "[docker_init] Starting... ($(date +'%D %T'))" >>$LOG_OUT
state=$(sudo docker info --format '{{.Swarm.LocalNodeState}}')
if [ -z $state ]; then
    echo "" >/dev/null
elif [ $state == "inactive" ]; then
    while [ 0 ]; do
        # determine designated leader
        if [ -f $LEADER_LOCATION ]; then
            echo "[docker_init] Found local leader file"
            # read potential leader name (head -1 for first line only)
            leader=$(head -1 $LEADER_LOCATION)
            if check_leader $leader; then break; fi # leader accepted
        else
            echo "[docker_init] Asking hosts for leader file..." >>$LOG_OUT
            # go through list of hosts using a for-each loop
            # for i in $(cat ...names_with_ip) would read word by word, not line by line
            # so instead, the line count is used to cut the list to one line containing only: host ip
            line_count=$(cat $SW_SETUP/ssh_identities/names_with_ip | wc -l)
            for i in $(seq 1 $line_count); do
                # read host and ip from single i-th line
                read host ip <<< $(head -$i $SW_SETUP/ssh_identities/names_with_ip | tail -1)
                # skip self or host without public key
                if [ $host == $MESH_IDENTITY ] || [ ! -f ~/.ssh/$host.pub ]; then continue; fi
                echo "[docker_init] Connecting to $host $ip..." >>$LOG_OUT
                # check if host is reachable (timeout 1 second (-W) and ping count limited to 1 (-c))
                if ping -W 1 -c 1 $ip &>/dev/null; then
                    echo "[docker_init] Reached $host" >>$LOG_OUT
                    remote_leader_location=/home/$host/solarswarm_setup/docker/leader
                    if scp $SSH_TIMEOUT $HOST_CHECKING $host:$remote_leader_location $LEADER_TARGET; then
                        # got leader file from host
                        echo "[docker_init] Found leader file from $host" >>$LOG_OUT
                        # read potential leader name (head -1 for first line only)
                        leader=$(head -1 $LEADER_LOCATION)
                        if check_leader $leader; then break 2; fi
                        # 1 is for loop and 2 is while loop above
                    fi
                else
                    echo "[docker_init] Failed to reach $host" >>$LOG_OUT
                fi
            done
            echo "[docker_init] Could not get leader file from any hosts - trying again..." >>$LOG_OUT
        fi
        sleep 2
    done

    # finalize leader
    if [ ! -z $leader ]; then
        LEADER=$leader
        LEADER_IP=$leader_ip
        LABELS_TARGET=/home/$LEADER/solarswarm_setup/rx/docker
        echo "[docker_init] Final leader: $LEADER" >>$LOG_OUT
    else
        echo "[docker_init] Error: Expected leader - exiting..." >>$LOG_OUT
        exit 1
    fi

    # sudo hostname $MESH_IDENTITY # set hostname to make managing swarm easier
    # hostname should be set with the 'ssh' option of the 'service_helper.bash' script

    # check if self is leader
    if [ ! -z $LEADER ] && [ $LEADER == $MESH_IDENTITY ]; then # is leader
        # init swarm and produce join token
        sudo docker swarm init --advertise-addr $MESH_IP
        add_own_labels
        generate_worker_token
        line_count=$(sudo docker stack ls | wc -l)
        if [ $? == 0 ] && [ $line_count -eq 1 ]; then deploy_stack; fi
        if sudo systemctl restart docker_leader.service; then
            echo "[docker_init] Restarted docker_leader.service"
        fi
        # 2377 is the standard port
        echo "[docker_init] Leaving leader branch" >>$LOG_OUT
    else # is worker
        # check if <name>.labels exists or can be ignored
        # if ignored, this node has to be unpaused manually from a manager or it will not start any services
        # to do this, use: sudo docker node update --availability active <this node's hostname, node id or ip>
        # note: placing an empty <name>.labels file is enough for the leader to activate this node after joining
        #       providing labels is only required to run swarm services with labels as constraints 
        if [ ! -f $LABELS_LOCATION ] && [ $IGNORE_LABELS_MISSING == false ]; then
            echo "[docker_init] Error: Found no $MESH_IDENTITY.labels in '$SW_SETUP/docker/' - exiting..."
            exit 1
        fi
        # attempt to join with present token
        joining=true
        while [ $joining == true ]; do
            if [ -f $WORKER_TOKEN_LOCATION ]; then # token found
                if ! sudo docker swarm join \
                    --availability pause \
                    --advertise-addr $MESH_IP \
                    --token $(head -1 $WORKER_TOKEN_LOCATION); then
                    echo "[docker_init] Error: Failed to join - removing worker join token" >>$LOG_OUT
                    rm $WORKER_TOKEN_LOCATION
                    continue # remove token and get another until join is successful
                fi
                # joined successfully
                # worker is paused until manager or leader updates it
                state=$(sudo docker info --format '{{.Swarm.LocalNodeState}}')
                if [ $state == "pending" ] || [ $state == "active" ]; then
                    joining=false
                    # effectievly a break statement
                    # is used in case a reason to stop joining is introduced later
                    echo "[docker_init] Joined swarm with state $state" >>$LOG_OUT
                fi
            else # elif [ ! -f $WORKER_TOKEN_LOCATION ]; then
                # try to get token from leader
                echo "[docker_init] Asking hosts for worker join token..." >>$LOG_OUT
                host=$LEADER
                worker_loc=/home/$host/solarswarm_setup/docker/worker_token
                if ! scp $SSH_TIMEOUT $HOST_CHECKING $LEADER:$worker_loc $SW_SETUP/docker/worker_token; then
                    line_count=$(cat $SW_SETUP/ssh_identities/names_with_ip | wc -l)
                    for i in $(seq 1 $line_count); do
                        worker_loc=/home/$host/solarswarm_setup/docker/worker_token
                        read host ip <<< $(head -$i $SW_SETUP/ssh_identities/names_with_ip | tail -1)
                        if [ $host == $MESH_IDENTITY ] || [ ! -f ~/.ssh/$host.pub ]; then continue; fi
                        # skip self, but note that leader will be contacted twice
                        echo "[docker_init] Connecting to $host $ip..." >>$LOG_OUT
                        if ping -W 1 -c 1 $ip &>/dev/null; then
                            echo "[docker_init] Reached $host" >>$LOG_OUT
                            if scp $SSH_TIMEOUT $HOST_CHECKING $host:$worker_loc $SW_SETUP/docker/; then
                                echo "[docker_init] Found worker join token from $host" >>$LOG_OUT
                                break
                            fi
                        else
                            echo "[docker_init] Failed to reach $host" >>$LOG_OUT
                        fi
                    done
                    echo "[docker_init] Could not get worker join token from any hosts" >>$LOG_OUT
                fi
            fi
            sleep 5
        done
        
        # send labels to leader to be labelled and activated
        touch $LABELS_LOCATION
        while [ 0 ]; do
            manager_count=$(sudo docker info --format {{.Swarm.Managers}} 2>/dev/null)
            if [ ! -z $manager_count ]; then
                manager_ip_addresses=$(sudo docker info 2>/dev/null | grep "Manager Addresses" -A 1 2>/dev/null | tail -1 | sed 's/ //g' | sed 's/:2377//g')
                if [ ! -z $manager_ip_addresses ]; then
                    count_sent=0
                    for ip_address in $manager_ip_addresses; do
                        read hostname ip_address <<< $(grep -E "^.+ $(echo $ip_address | sed 's/./\./g')$" $SW_SETUP/ssh_identities/names_with_ip)
                        if [ -z $hostname ]; then continue; fi
                        labels_target="/home/$hostname/solarswarm_setup/docker/"
                        if scp $SSH_TIMEOUT $HOST_CHECKING $LABELS_LOCATION $hostname:$labels_target; then ((count_sent++));fi
                    done
                    required_count=(($manager_count/2 + 1))
                    if [ $count_sent -ge $(($required_count)) ]; then
                        break
                    else
                        echo "[docker_init] Could not send labels file to at least $required_count managers ($count_sent/$manager_count)"
                    fi # enough when more than half of all managers received labels
                fi
            fi
            sleep 5
        done
        echo "[docker_init] Leaving worker branch"
    fi # if is leader or worker
fi # if state is 'inactive'

echo "[docker_init] Monitoring local node state..."
unknown_error_count=0 # for check_for_health to detect loss of quorum as worker who can still reach managers
while [ 0 ]; do
    state=$(sudo docker info --format '{{.Swarm.LocalNodeState}}')
    if [ -z $state ]; then
        echo "[docker_init] Node state: unknown" #>/dev/null
    else
        echo "[docker_init] Node state: $state" #>/dev/null
        if [ $state == "inactive" ]; then
            sudo systemctl stop docker_leader.service 2>/dev/null
            exit 1
        elif [ $state == "error" ]; then
            sudo systemctl stop docker_leader.service 2>/dev/null
            sudo docker swarm leave -f
            exit 1
        elif [ $state == "locked" ]; then
            echo "" >/dev/null
        elif [ $state == "active" ]; then
            # check if self became manager and possibly leader
            is_manager=$(sudo docker info | awk '/Is Manager:/ {print $3}' 2>/dev/null)
            if [ $? ] && [ ! -z $is_manager ] && [ $is_manager == "true" ]; then
                # renew join token
                cluster_id=$(sudo docker info | awk '/ClusterID/ {print $2}')
                token_cluster_id=$(head -2 $WORKER_TOKEN_LOCATION | tail -1)
                if [ ! $cluster_id == $token_cluster_id ]; then
                    # token is from different cluster
                    generate_worker_token
                elif ! grep $MESH_IP $WORKER_TOKEN_LOCATION &>/dev/null; then
                    # token is from different manager -> might not work if they are unreachable
                    generate_worker_token
                fi

                # check if docker_leader must be started
                manager_status=$(sudo docker node ls --filter "role=manager" --format "{{.Hostname}} {{.ManagerStatus}}" | grep "$MESH_IDENTITY" | awk '{print $2}')
                if [ $? ] && [ ! -z $manager_status ] && [ $manager_status ==  "Leader" ]; then
                    # is leader
                    if [ ! $(sudo systemctl is-active docker_leader.service) == "active" ]; then
                        sudo systemctl restart docker_leader.service
                    fi
                fi
            fi # if manager

            check_for_health
            code=$?
            # code 0: self is healthy
            # code 1: self is manager without quorum
            # code 2: self is isolated from managers (internal or external causes)
            # code 3: manager left cluster
            # code 4: manager joined different cluster
            # code -1/255: unkown case
            
            # echo $code

            # update designated leader
            designated_leader=$(head -1 $SW_SETUP/docker/leader)
            if ! check_leader $designated_leader; then designated_leader=""; fi

            # check for swarm errors (no reachable managers or loss of quorum)
            # while error is present, repeatedly get newest join token and ClusterID of designated leader
            # if ClusterID has changed, designated leader has forced a new cluster to solve quorum loss
            # if so, join new cluster
            if [ ! -z $code ] && [ ! -z $designated_leader ] && [ $code -ne 0 ] && \
                    [ ! $MESH_IDENTITY == $designated_leader ]; then
                # only if not designated leader
                if [ $code -eq 255 ] && [ $unknown_error_count -lt $UNKNOWN_ERROR_TOLERANCE ]; then
                    ((unknown_error_count++))
                else
                    # only if not unknown error or if unknown error keeps occuring
                    # this is done this way because behaviour was not tested for a worker who can reach a manager without quorum

                    # get newest token from designated leader and compare ClusterID
                    echo "[docker_init] Healthcheck code: $code ($(date +'%D %T'))" >>$LOG_OUT
                    host=$designated_leader
                    worker_loc=/home/$host/solarswarm_setup/docker/worker_token

                    # get token
                    tries=5
                    echo "[docker_init] Asking designated leader $host for worker join token... ($tries times)" >>$LOG_OUT
                    while [ $tries -gt 0 ]; do
                        if scp $SSH_TIMEOUT $HOST_CHECKING $host:$worker_loc $SW_SETUP/docker/cluster_check; then
                            # got newest token -> compare ClusterID
                            cluster_id=$(head -2 $WORKER_TOKEN_LOCATION | tail -1 )
                            new_cluster_id=$(head -2 $SW_SETUP/docker/cluster_check | tail -1 )
                            if [ ! -z $cluster_id ] && [ ! -z $new_cluster_id ] && [ ! $cluster_id == $new_cluster_id ]; then
                                # join new cluster
                                echo "[docker_init] Found new cluster to join. Leaving cluster with ClusterID $cluster_id" >>$LOG_OUT
                                sudo docker swarm leave -f
                                mv $SW_SETUP/docker/cluster_check $WORKER_TOKEN_LOCATION
                                if sudo docker swarm join \
                                        --availability pause \
                                        --advertise-addr $MESH_IP \
                                        --token $(head -1 $WORKER_TOKEN_LOCATION); then
                                    echo "[docker_init] Node joined new cluster with ClusterID $new_cluster_id" >>$LOG_OUT
                                    break
                                fi
                            fi
                        fi
                        ((tries--))
                        sleep 5
                    done # while trying to copy join token with ClusterID
                    sleep 5
                fi
            fi # if is not designated leader
        fi # if state is active
    fi # if state is empty
    sleep $HEALTHCHECK_DELAY
done

