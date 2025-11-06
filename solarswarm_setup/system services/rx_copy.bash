#!/bin/bash
COPY_TO_SSH=true
AUTO_UPDATE_SSH=true
# SWARM_MODE=true
SW_SETUP=~/solarswarm_setup/

# update ~/.ssh/ as hosts can not directy copy into this directory
if [ $COPY_TO_SSH == true ]; then
    if [ -d ~/.ssh/ ] && [ -d $SW_SETUP/rx/ssh/ ]; then
        changed=false
        # move names, names_with_ip, config, and public keys to ssh_identities/
        if [ -f $SW_SETUP/rx/ssh/names ]; then
            mv $SW_SETUP/rx/ssh/names $SW_SETUP/ssh_identities/
            changed=true
        fi

        if [ -f $SW_SETUP/rx/ssh/names_with_ip ]; then
            mv $SW_SETUP/rx/ssh/names_with_ip $SW_SETUP/ssh_identities/
            changed=true
        fi

        if [ -f $SW_SETUP/rx/ssh/config ]; then
            mv $SW_SETUP/rx/ssh/config $SW_SETUP/ssh_identities/
            changed=true
        fi

        for pub_key in $(ls $SW_SETUP/rx/ssh/ | grep .pub); do
            if [ ! $pub_key == $MESH_IDENTITY.pub ]; then
                mv $SW_SETUP/rx/ssh/$pub_key $SW_SETUP/ssh_identities/keys/
                changed=true
            fi
        done

        # copy all to .ssh/ if anything has changed
        # done after everything was moved so ssh_identities/ and .ssh/ are synchronized
        if [ $changed == true ] && [ $AUTO_UPDATE_SSH == true ]; then
            # sudo cp $SW_SETUP/ssh_identities/names ~/.ssh/
            # sudo cp $SW_SETUP/ssh_identities/names_with_ip ~/.ssh/
            sudo cp $SW_SETUP/ssh_identities/config ~/.ssh/
            sudo cp $SW_SETUP/ssh_identities/keys/*.pub ~/.ssh/ # all public keys
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