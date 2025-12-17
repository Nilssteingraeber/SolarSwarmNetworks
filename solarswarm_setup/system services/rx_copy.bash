#!/bin/bash
source /etc/environment
COPY_TO_SSH=true
AUTO_UPDATE_SSH=true
# SWARM_MODE=true
SW_SETUP=/home/$MESH_IDENTITY/solarswarm_setup
LOG_OUT=/dev/stdout # see docker_init.bash

# update ~/.ssh/ as hosts can not directy copy into this directory
if [ $COPY_TO_SSH == true ]; then
    if [ -d ~/.ssh/ ] && [ -d $SW_SETUP/rx/ssh/ ]; then
        echo "[rx_copy] Checking '$SW_SETUP/rx/ssh/'" >>$LOG_OUT
        changed=false
        # move names, names_with_ip, config, and public keys to ssh_identities/
        if [ -f $SW_SETUP/rx/ssh/names ]; then
            mv $SW_SETUP/rx/ssh/names $SW_SETUP/ssh_identities/
            echo "[rx_copy] Moved 'names' to '$SW_SETUP/ssh_identities/'" >>$LOG_OUT
            changed=true
        fi

        if [ -f $SW_SETUP/rx/ssh/names_with_ip ]; then
            mv $SW_SETUP/rx/ssh/names_with_ip $SW_SETUP/ssh_identities/
            echo "[rx_copy] Moved 'names_with_ip' to '$SW_SETUP/ssh_identities/'" >>$LOG_OUT
            changed=true
        fi

        if [ -f $SW_SETUP/rx/ssh/config ]; then
            mv $SW_SETUP/rx/ssh/config $SW_SETUP/ssh_identities/
            echo "[rx_copy] Moved 'config' to '$SW_SETUP/ssh_identities/'" >>$LOG_OUT
            changed=true
        fi
        
        for name in $($SW_SETUP/ssh_identities/names); do
            if [ ! -z $name ] && [ -d $SW_SETUP/rx/ssh/ ] && [ ! -z $SW_SETUP/rx/ssh/$name.pub] && [ ! $name == $MESH_IDENTITY ]; then
                mv $SW_SETUP/rx/ssh/$name.pub $SW_SETUP/ssh_identities/keys/
                echo "[rx_copy] Moved '$name.pub' to '$SW_SETUP/ssh_identities/keys/'" >>$LOG_OUT
                changed=true
            fi
        done

        if [ ! -z $MESH_IP ] && [ $changed == true ] && [ $AUTO_UPDATE_SSH == true ]; then
            cp $SW_SETUP/ssh_identities/config ~/.ssh/
            sudo sed -i "s/own_name/$MESH_IP/" ~/.ssh/config # replace own_name with MESH_IP
            echo "[rx_copy] Copied 'config' to '~/.ssh/'" >>$LOG_OUT
        fi
    fi
fi

# copy 
# if [ $SWARM_MODE == true ]; then
#     if [ -f $SW_SETUP/rx/docker/worker_token ]; then
#         mv $SW_SETUP/rx/docker/worker_token $SW_SETUP/docker
#     fi

#     if [ -f $SW_SETUP/rx/docker/manager_token ]; then
#         mv $SW_SETUP/rx/docker/manager_token $SW_SETUP/docker
#     fi
# fi
