#!/bin/bash
LEADER_LOCATION=~/solarswarm_setup/docker/leader
WORKER_TOKEN_LOCATION=~/solarswarm_setup/docker/worker_token
MANAGER_TOKEN_LOCATION=~/solarswarm_setup/docker/manager_token
LABELS_LOCATION=~/solarswarm_setup/docker
LABELS_TARGET=~/solarswarm_setup/rx/docker
HEALTHCHECK_DELAY=10
SSH_TIMEOUT=1

while [ ! -f ~/solarswarm_setup/docker/leader ]; do
    for i in $(cat ssh_identities/names_with_ip); do
        read host ip <<< $i
        if [ $host == $MESH_IDENTITY ]; then continue; fi # skip self
        echo "Connecting to $host $ip..."
        if ping -c 1 $ip; then
            echo "Reached $host"
            if sudo scp -o ConnectTimeout=$SSH_TIMEOUT $host:$LEADER_LOCATION $LEADER_LOCATION; then
                break
            fi
        else
            echo "Failed to reach $host"
        fi
    done
done

# read leader from leader file
if [ -f ~/solarswarm_setup/docker/leader ]
    LEADER=$(head -1 ~/solarswarm_setup/docker/leader) # TODO: what if leader loses connection?
fi

sudo hostname $MESH_IDENTITY # set hostname to make managing swarm easier

source /etc/environment
if [ -z $MESH_IDENTITY ] || [ -z $MESH_IP ]; then
    exit 1
fi

# check if self is leader
if [ $LEADER == $MESH_IDENTITY ]; then # is leader
    sudo docker swarm init --advertise $MESH_IP
    echo "$(sudo docker swarm join-token -q worker) $MESH_IP:2377" > $WORKER_TOKEN_LOCATION
    sudo systemctl restart docker_leader.service
    # 2377 is the standard port
else # is worker
    joining=true
    while [ $joining == true ]; then
        if [ -f $WORKER_TOKEN_LOCATION ] && [ sudo docker join \
            --availability pause \
            --advertise-addr $MESH_IP \
            --token "$(cat $WORKER_TOKEN_LOCATION)"
        ]; then
            # joined successfully
            # is paused until manager or leader updates it
            state=$(sudo docker info --format '{{.Swarm.LocalNodeState}}')
            if [ $state == "pending" ] or [ $state == "active"]; then
                joining=false
            fi
        elif [ ! -f $WORKER_TOKEN_LOCATION ]; then
            # try to get token from leader
            if [ ! sudo scp -o ConnectTimeout=$SSH_TIMEOUT $LEADER:$WORKER_TOKEN_LOCATION $WORKER_TOKEN_LOCATION ]; then
                for i in $(cat ssh_identities/names_with_ip); do
                    read host ip <<< $i
                    if [ $host == $MESH_IDENTITY ]; then continue; fi # skip self
                    echo "Connecting to $host $ip..."
                    if ping -c 1 $ip; then
                        echo "Reached $host"
                        sudo scp -o ConnectTimeout=$SSH_TIMEOUT $host:$WORKER_TOKEN_LOCATION $WORKER_TOKEN_LOCATION
                    else
                        echo "Failed to reach $host"
                    fi
                done
            fi
        fi
    fi

    # send labels to leader to be labelled and activated
    while ! sudo scp -o ConnectTimeout=$SSH_TIMEOUT $LABELS_LOCATION $LEADER:$LABELS_TARGET &>/dev/null; do
        sleep 5
    done
fi



while [ 0 ]; do
    state=$(sudo docker info --format '{{.Swarm.LocalNodeState}}')
    # TODO: log state
    if [ $state == "inactive" ]; then
        exit 1
    elif [ $state == "error" ]; then
        
    elif [ $state == "locked" ]; then

    fi
    sleep $HEALTHCHECK_DELAY
done
