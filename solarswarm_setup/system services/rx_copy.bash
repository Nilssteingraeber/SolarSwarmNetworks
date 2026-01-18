#!/bin/bash
source /etc/environment
COPY_TO_SSH=true
COPY_TO_DOCKER=true
AUTO_UPDATE_SSH=true
# SWARM_MODE=true
SW_SETUP=/home/$MESH_IDENTITY/solarswarm_setup
LOG_OUT=/dev/stdout # see docker_init.bash

LEADER_LOC=$SW_SETUP/rx/docker/leader
LEADER_TARGET=$SW_SETUP/docker/leader

check_leader() {
    leader=$1 # first (and only) argument is leader, i.e.: 'check_leader alfa' -> 'leader="alfa"' 
    if [ ! -z $leader ] && grep -E "^$leader$" $SW_SETUP/ssh_identities/names &>/dev/null; then
        return 0 # valid leader
    fi 
    return 1
}

while [ 0 ]; do
    # rx/ssh/
    # update ~/.ssh/ as hosts can not directy copy into this directory
    if [ $COPY_TO_SSH == true ]; then
        if [ -d ~/.ssh/ ] && [ -d $SW_SETUP/rx/ssh/ ]; then
            echo "[rx_copy] Checking '$SW_SETUP/rx/ssh/'" >>$LOG_OUT
            changed=false
            # move names, names_with_ip, config, and public keys to ssh_identities/
            if [ -f $SW_SETUP/rx/ssh/names ]; then
                mv $SW_SETUP/rx/ssh/names $SW_SETUP/ssh_identities/names
                echo "[rx_copy] Moved 'names' to '$SW_SETUP/ssh_identities/names'" >>$LOG_OUT
                changed=true
            fi

            if [ -f $SW_SETUP/rx/ssh/names_with_ip ]; then
                mv $SW_SETUP/rx/ssh/names_with_ip $SW_SETUP/ssh_identities/names_with_ip
                echo "[rx_copy] Moved 'names_with_ip' to '$SW_SETUP/ssh_identities/names_with_ip'" >>$LOG_OUT
                changed=true
            fi

            if [ -f $SW_SETUP/rx/ssh/config ]; then
                mv $SW_SETUP/rx/ssh/config $SW_SETUP/ssh_identities/config
                echo "[rx_copy] Moved 'config' to '$SW_SETUP/ssh_identities/config'" >>$LOG_OUT
                changed=true
            fi
            
            for name in $($SW_SETUP/ssh_identities/names); do
                if [ ! -z $name ] && [ -d $SW_SETUP/rx/ssh/ ] && [ ! -z $SW_SETUP/rx/ssh/$name.pub] && [ ! $name == $MESH_IDENTITY ]; then
                    mv $SW_SETUP/rx/ssh/$name.pub $SW_SETUP/ssh_identities/keys/$name.pub
                    echo "[rx_copy] Moved '$name.pub' to '$SW_SETUP/ssh_identities/keys/$name.pub'" >>$LOG_OUT
                    changed=true
                fi
            done

            if [ ! -z $MESH_IP ] && [ $changed == true ] && [ $AUTO_UPDATE_SSH == true ]; then
                cp $SW_SETUP/ssh_identities/config ~/.ssh/config
                sudo sed -i "s/own_name/$MESH_IDENTITY/" ~/.ssh/config # replace own_name with MESH_IDENTITY
                echo "[rx_copy] Copied 'config' to '~/.ssh/config'" >>$LOG_OUT
            fi
        fi
    fi

    # rx/docker/
    if [ $COPY_TO_DOCKER == true ]; then
        if [ -f $LEADER_LOC ] && check_leader $(head -1 $LEADER_LOC); then
            mv $LEADER_LOC $LEADER_TARGET
        fi
    fi
done