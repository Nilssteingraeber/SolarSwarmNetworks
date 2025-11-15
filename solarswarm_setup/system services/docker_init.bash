#!/bin/bash
source /etc/environment
SW_SETUP=/home/$MESH_IDENTITY/solarswarm_setup
LEADER_TARGET=$SW_SETUP/docker # only for local host!
LEADER_LOCATION=$SW_SETUP/docker/leader # only for local host!
WORKER_TOKEN_LOCATION=$SW_SETUP/docker/worker_token # only for local host!
MANAGER_TOKEN_LOCATION=$SW_SETUP/docker/manager_token # only for local host!
    # ^ $SW_SETUP contains MESH_IDENTITY!
    # scp will look for the wrong home directory on different hosts
LABELS_LOCATION=$SW_SETUP/docker/$MESH_IDENTITY.labels
IGNORE_LABELS_MISSING=false
HEALTHCHECK_DELAY=10
SSH_TIMEOUT="-o ConnectTimeout=1" # see service_helper.bash
HOST_CHECKING="-o StrictHostKeyChecking=no" # see service_helper.bash
LOG_OUT=/dev/stdout # also defined in docker_leader.bash and rx_copy.bash

# leave swarm if already in one
sudo docker swarm leave -f

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
    if [ ! -z $leader ] && [ $(grep $leader $SW_SETUP/ssh_identities/names) ]; then
        # leader contains valid name
        # get respective ip
        leader_ip=$(grep $leader $SW_SETUP/ssh_identities/names_with_ip | awk '{print $2}');
        failed_retries=0
        RETRIES=5
        for j in $(seq $RETRIES); do # attempt to reach leader j times
            if ! ping -W 1 -c 1 $leader_ip &>/dev/null; then
                echo "[docker_init] Failed attempt $j to reach leader $leader" >>$LOG_OUT
                ((failed_retries++))
            else
                break
            fi
        done
        
        if [ $failed_retries -lt $RETRIES ]; then
            echo "[docker_init] Reached leader $leader" >>$LOG_OUT
            return 0 # success
        else
            echo "[docker_init] Failed to reach leader - removing file..." >>$LOG_OUT
            rm $LEADER_LOCATION
        fi
    else
        echo "[docker_init] Leader file contains invalid name - removing file..." >>$LOG_OUT
        rm $LEADER_LOCATION
    fi
    return 1
}

while [ 0 ]; do
    if [ -f $LEADER_LOCATION ]; then
        echo "[docker_init] Found local leader file"
    	# read potential leader name (head -1 for first line only)
        leader=$(head -1 $LEADER_LOCATION)
        if check_leader $leader; then break; fi
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
    LEADER=$leader # TODO: what if leader loses connection?
    LEADER_IP=$leader_ip
    LABELS_TARGET=/home/$LEADER/solarswarm_setup/rx/docker
    echo "[docker_init] Final leader: $LEADER" >>$LOG_OUT
else
    echo "[docker_init] Error: Expected leader - exiting..." >>$LOG_OUT
    exit 1
fi

sudo hostname $MESH_IDENTITY # set hostname to make managing swarm easier

if [ -z $MESH_IDENTITY ] || [ -z $MESH_IP ]; then
    echo "[docker_init] Error: Identity was not set - exiting..." >>$LOG_OUT
    exit 1
fi

# check if self is leader
if [ ! -z $LEADER ] && [ $LEADER == $MESH_IDENTITY ]; then # is leader
    sudo docker swarm init --advertise-addr $MESH_IP
    echo "$(sudo docker swarm join-token -q worker 2>/dev/null) $MESH_IP:2377" > $WORKER_TOKEN_LOCATION
    echo "[docker_init] Generated worker join token: $(cat $WORKER_TOKEN_LOCATION)" >>$LOG_OUT
    sudo systemctl restart docker_leader.service
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
                --token $(cat $WORKER_TOKEN_LOCATION); then
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
            if ! scp $SSH_TIMEOUT $HOST_CHECKING $LEADER:$worker_loc $SW_SETUP/docker/; then
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
    while ! scp $SSH_TIMEOUT $HOST_CHECKING $LABELS_LOCATION $LEADER:$LABELS_TARGET; do
        echo "[docker_init] Failed to provide labels to leader $LEADER - trying again in 5 seconds" >>$LOG_OUT
        sleep 5
    done
    echo "[docker_init] Leaving worker branch" >>$LOG_OUT
fi


echo "[docker_init] Monitoring local node state..." >>$LOG_OUT
while [ 0 ]; do
    state=$(sudo docker info --format '{{.Swarm.LocalNodeState}}')
    # TODO?: log state
    if [ $state == "inactive" ]; then
        echo "[docker_init] Node state: inactive" #>/dev/null
        exit 1
    elif [ $state == "error" ]; then
        echo "[docker_init] Node state: error" #>/dev/null
    elif [ $state == "locked" ]; then
        echo "[docker_init] Node state: locked" #>/dev/null
    else
        echo "[docker_init] Node state: active" #>/dev/null
    fi
    sleep $HEALTHCHECK_DELAY
done
