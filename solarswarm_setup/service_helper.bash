#!/bin/bash
# v0.9.2
SSH_TIMEOUT="-o ConnectTimeout=1"
# change if too short (was sufficient to connect to github.com in a connection test)
# ConnectTimeout could be set in '/etc/ssh/ssh_config' instead, but it is easier to find here
# note: docker_init.bash and docker_leader.bash also set this variable locally

HOST_CHECKING="-o StrictHostKeyChecking=no"
# allow ssh connection to hosts without strict authorization (susceptible to MIT attacks)
# change StrictHostKeyChecking to "yes" or make variable empty for more security if a CA was set up
# StrictHostKeyChecking could be set in '/etc/ssh/ssh_config' instead, but it is easier to find here
# note: docker_init.bash and docker_leader.bash also set this variable locally

SW_RUN=~/solarswarm_run
SW_SETUP=~/solarswarm_setup # where solarswarm_setup is located (~ is suggested)
cd $SW_SETUP # requires parent directory 'solarswarm_setup' to be placed in '~'

source /etc/environment

if [ -z $1 ]; then # print help if no arguments were passed
    echo """    Correct usage:
        bash service_helper [options] [additional options]
    Options:
        status
          - show service status, presence of files, and envs
        wlandev [wireless interface]
          - set WLANDEV to given wlan device
        ssh
          - generate ssh keys with a name
        send <keys | hosts> <everybody | hostname> [register]
          - copy keys or hosts (names, names_with_ip, config) onto all
            available hosts
          - add all local public keys to a hosts' 'authorized_keys'
            with option 'register'
        collect <logs | keys> <everybody | hostname>
          - collect files from hosts (currently only logs)
          - copies solarswarm_run/logs/ and solarswarm_setup/logs/
          - target is solarswarm_setup/rx/logs/<run | setup>/<hostname>/
        register
          - add local public keys to 'authorized_keys'
    	setup
          - copy files, make scripts executable, reload systemd,
    	  - enable timers and restart all services
    	cleanup [with_keys]
          - remove services and files (except ssh keys and hosts)
          - remove all ssh keys if 'with_keys' is given
    	restart [valid service]
          - restart all or a specific service (or timer)
    	stop [valid service]
          - stop all or a specific service (or timer)
    	enable [valid service]
          - enable all or a specific service (or timer)
    	disable [valid service]
          - disable all or a specific service (or timer)
        managed
          - return from ad-hoc to managed mode (required for internet
            access)
    Notes:
        <argument>                for required argument
        [argument]                for optional argument
        argument | argument       for choice of arguments

        - 'All' services excludes docker_leader as that service should
          generally be startet by the docker_init service if a file
          'leader' with the host's name in it is inside 'docker/'.
        - The name provided for ssh must be in the files 'names' and
          match with a name and ip in 'names_with_ip' and config.
          Ensure that no two maschines use the same name or ip.
        - Use send option while all target robots are reachable to
          update changes to 'name', 'names_with_ip' and 'config' or
          to share one's own a new public key.
        - Internet access is only possible while in 'managed' mode.
        - Ensure that 'system services/batman_adv_setup.bash' uses
          the correct wireless interface to set up ad-hoc mode. To
          disable ad-hoc, use the 'managed' option.
        - Enabled services and timers start automatically after boot.
          A service can be disabled and active at the same time.
    """
    exit 1
fi

echo_valid_service() { # to avoid redundancy
    echo """Unknown service name. Use any of:
    batman_adv_setup
    batman_adv_healthcheck
    iw_dump
    docker_leader (special, should not be started/enabled manually)
    docker_init
    rx_copy
    """
}

send_to_hosts() { # copy files to all available ssh hosts' rx/
    # args: <keys | hosts> <everybody | specific_name> [register]
    source /etc/environment # reload envs
    if [ -z $MESH_IDENTITY ]; then # check if name is set
        echo "Error: MESH_IDENTITY has not been set"
        exit 1
    fi

    if [ -z $MESH_IP ]; then # check if ip is set
        echo "Error: MESH_IDENTITY has not been set"
        exit 1
    fi
    
    sudo hostname $MESH_IDENTITY # set hostname
    
    if [ ! -z ] && [ $3 == "keys" ]; then
        echo "Which keys should be sent? (own/all) "
        read which_keys
    fi
    if [ ! $which_keys == "own" ] && [ ! $which_keys == "all" ] || [ -z $which_keys ]; then
        echo "Error: Choose either 'own' or 'all'"
        exit 1
    fi

    if [ $2 == "everybody" ]; then
        # go through list of hosts using a for-each loop
        # for i in $(cat ...names_with_ip) would read word by word, not line by line
        # so instead, the line count is used to cut the list to one line containing only: host ip
        line_count=$(cat ssh_identities/names_with_ip | wc -l)
        for i in $(seq 1 $line_count); do
            # read host and ip from single i-th line
            read host ip <<< $(head -$i ssh_identities/names_with_ip | tail -1)
            # skip empty hosts and self
            if [ -z $host ] || [ -z $ip ]; then continue; fi
            if [ ! -z $host ] && [ $host == $MESH_IDENTITY ]; then continue; fi
            echo "Connecting to $host $ip..."
            # check if host is reachable (timeout 1 second (-W) and ping count limited to 1 (-c))
            if ping -W 1 -c 1 $ip &>/dev/null; then # ping once with 1s timeout
                echo "Reached $host"
                remote_setup=/home/$host/solarswarm_setup
                
                if [ ! -z $1 ] && [ $1 == "keys" ]; then # copy public keys
                    if [ $which_keys == "all" ]; then
                        scp $SSH_TIMEOUT $HOST_CHECKING ssh_identities/keys/*.pub \
                            $host:$remote_setup/rx/ssh/ # copy public keys
                        if [ ! -z $4 ] && [ $4 == "register" ]; then # add to authorized_keys of host
                            # register own key key first to possibly not require a password for every other key
                            ssh-copy-id $SSH_TIMEOUT $HOST_CHECKING -f -i ssh_identities/keys/$MESH_IDENTITY.pub $host@$ip
                            for pub_key in $(ls ssh_identities/keys/); do
                                if [ pub_key == MESH_IDENTITY.pub ]; then continue; fi
                                ssh-copy-id $SSH_TIMEOUT $HOST_CHECKING -f -i ssh_identities/keys/$pub_key $host@$ip
                                # note: all keys in 'ssh_identities/keys/' are currently trusted
                            done
                        fi # TODO: test if register finally works
                    elif [ $which_keys == "own" ]; then
                        scp $SSH_TIMEOUT $HOST_CHECKING ssh_identities/keys/*.pub \
                            $host:$remote_setup/rx/ssh/ # copy public key
                        if [ ! -z $4 ] && [ $4 == "register" ]; then # add to authorized_keys of host
                            ssh-copy-id $SSH_TIMEOUT $HOST_CHECKING -f -i ssh_identities/keys/$MESH_IDENTITY.pub $host@$ip
                        fi
                    fi
                elif [ ! -z $1 ] && [ $1 == "hosts" ]; then # copy names, names_with_ip, and config to all reachable hosts
                    scp $SSH_TIMEOUT $HOST_CHECKING ssh_identities/names \
                        $host:$remote_setup/ssh_identities/rx/ssh/
                    scp $SSH_TIMEOUT $HOST_CHECKING ssh_identities/names_with_ip \
                        $host:$remote_setup/ssh_identities/rx/ssh/
                    scp $SSH_TIMEOUT $HOST_CHECKING ssh_identities/config \
                        $host:$remote_setup/ssh_identities/rx/ssh/
                else
                    echo "Error: No valid option"
                    exit 1
                fi
            else
                echo "Failed to reach $host"
            fi
        done
    else # with specific name
        if [ -z $2 ] || ! $(grep -E "^$2$" ssh_identities/names &>/dev/null); then
            echo "Error: Expected 'everybody' or a known name after 'keys' or 'hosts'"
            exit 1
        fi
        read host ip <<< $(grep -E "^$2 " ssh_identities/names_with_ip &>/dev/null)
        echo "Connecting to $host $ip..."
        # check if host is reachable (timeout 1 second (-W) and ping count limited to 1 (-c))
        if ping -W 1 -c 1 $ip &>/dev/null; then # ping once with 1s timeout
            echo "    Reached $host"
            remote_run=/home/$host/solarswarm_run
            remote_setup=/home/$host/solarswarm_setup

            if [ ! -z $1 ] && [ $1 == "keys" ]; then # copy public keys
                if [ $which_keys == "all" ]; then
                    scp $SSH_TIMEOUT $HOST_CHECKING ssh_identities/keys/*.pub \
                        $host:$remote_setup/ssh_identities/rx/ssh/ # copy public keys
                    if [ ! -z $4 ] && [ $4 == "register" ]; then # add to authorized_keys of host
                        # register own key key first to possibly not require a password for every other key
                        ssh-copy-id $SSH_TIMEOUT $HOST_CHECKING -f -i ssh_identities/keys/$MESH_IDENTITY.pub $host@$ip
                        for pub_key in $(ls ssh_identities/keys/); do
                            if [ pub_key == MESH_IDENTITY.pub ]; then continue; fi
                            ssh-copy-id $SSH_TIMEOUT $HOST_CHECKING -f -i ssh_identities/keys/$pub_key $host@$ip
                            # note: currently all keys in 'ssh_identities/keys/' are trusted
                        done
                    fi
                elif [ $which_keys == "own" ]; then
                    scp $SSH_TIMEOUT $HOST_CHECKING ssh_identities/keys/$MESH_IDENTITY.pub \
                        $host:$remote_setup/ssh_identities/rx/ssh/ # copy public key
                    if [ ! -z $4 ] && [ $4 == "register" ]; then # add to authorized_keys of host
                        ssh-copy-id $SSH_TIMEOUT $HOST_CHECKING -f -i ssh_identities/keys/$MESH_IDENTITY.pub $host@$ip
                    fi
                fi
            elif [ ! -z $1 ] && [ $1 == "hosts" ]; then # copy names, names_with_ip, and config to all reachable hosts
                scp $SSH_TIMEOUT $HOST_CHECKING ssh_identities/names \
                    $host:$remote_setup/ssh_identities/rx/ssh/
                scp $SSH_TIMEOUT $HOST_CHECKING ssh_identities/names_with_ip \
                    $host:$remote_setup/ssh_identities/rx/ssh/
                scp $SSH_TIMEOUT $HOST_CHECKING ssh_identities/config \
                    $host:$remote_setup/ssh_identities/rx/ssh/
            else
                echo "Error: No valid option"
                exit 1
            fi
        else
            echo "    Failed to reach $host"
            exit 1
        fi
    fi
}

collect() { # copy files from all reachable ssh hosts to local rx/
    source /etc/environment # reload envs
    if [ -z $MESH_IDENTITY ]; then # check if name is set
        echo "Error: MESH_IDENTITY has not been set"
        exit 1
    fi

    if [ -z $MESH_IP ]; then # check if ip is set
        echo "Error: MESH_IDENTITY has not been set"
        exit 1
    fi
    
    sudo hostname $MESH_IDENTITY # set hostname

    if [ $2 == "everybody" ]; then
        line_count=$(cat ssh_identities/names_with_ip | wc -l)
        for i in $(seq 1 $line_count); do
            # read host and ip from single i-th line
            read host ip <<< $(head -$i ssh_identities/names_with_ip | tail -1)
            # skip empty hosts and self
            if [ -z $host ] || [ -z $ip]; then continue; fi
            if [ ! -z $host ] && [ $host == $MESH_IDENTITY ]; then continue; fi
            echo "Connecting to $host $ip..."
            # check if host is reachable (timeout 1 second (-W) and ping count limited to 1 (-c))
            if ping -W 1 -c 1 $ip &>/dev/null; then # ping once with 1s timeout
                echo "    Reached $host"
                remote_run=/home/$host/solarswarm_run
                remote_setup=/home/$host/solarswarm_setup

                if [ ! -z $1 ] && [ $1 == "logs" ]; then # copy logs    
                    if [ ! -d rx/logs/run/$host ]; then mkdir rx/logs/run/$host; fi
                    if [ ! -d rx/logs/setup/$host ]; then mkdir rx/logs/setup/$host; fi
                    scp -r $SSH_TIMEOUT $HOST_CHECKING $host:$remote_run/logs/* rx/logs/run/$host # note: this will also copy .gitignore
                    scp -r $SSH_TIMEOUT $HOST_CHECKING $host:$remote_setup/logs/* rx/logs/setup/$host
                elif [ ! -z $1 ] && [ $1 == "keys" ]; then # copy keys
                    scp $SSH_TIMEOUT $HOST_CHECKING $host:$remote_setup/ssh_identities/keys/*.pub \
                        ssh_identities/rx/ssh/ # copy public keys
                    echo "Note: For other hosts to be able to use their private keys, you must add their public keys to your '~/.ssh/authorized_keys'."
                    echo "You can use 'bash service_helper.bash' to add all local keys to your '~/.ssh/authorized_keys'"
                fi
            else
                echo "    Failed to reach $host"
            fi
        done
    else # with specific name
        if [ -z $2 ] || ! $(grep -E "^$2$" ssh_identities/names &>/dev/null); then
            echo "Error: Expected 'everybody' or a known name after 'logs'"
            exit 1
        fi
        read host ip <<< $(grep -E "^$2 " ssh_identities/names_with_ip &>/dev/null)
        echo "Connecting to $host $ip..."
        # check if host is reachable (timeout 1 second (-W) and ping count limited to 1 (-c))
        if ping -W 1 -c 1 $ip &>/dev/null; then # ping once with 1s timeout
            echo "    Reached $host"
            remote_run=/home/$host/solarswarm_run
            remote_setup=/home/$host/solarswarm_setup

            if [ ! -z $1 ] && [ $1 == "logs" ]; then # copy logs
                if [ ! -z $2 ] && [ $2 == $MESH_IDENTITY ]; then
                    # explicitly collect own logs
                    if [ ! -d rx/logs/run/$MESH_IDENTITY ]; then mkdir rx/logs/run/$MESH_IDENTITY; fi
                    if [ ! -d rx/logs/setup/$MESH_IDENTITY ]; then mkdir rx/logs/setup/$MESH_IDENTITY; fi
                    cp -r $SW_RUN/logs/* rx/logs/run/$MESH_IDENTITY/
                    cp -r logs/* rx/logs/setup/$MESH_IDENTITY/
                else
                    if [ ! -d rx/logs/run/$2 ]; then mkdir rx/logs/run/$host; fi
                    if [ ! -d rx/logs/setup/$2 ]; then mkdir rx/logs/setup/$host; fi
                    scp -r $SSH_TIMEOUT $HOST_CHECKING $2:$remote_run/logs/* rx/logs/run/$2
                    scp -r $SSH_TIMEOUT $HOST_CHECKING $2:$remote_setup/logs/* rx/logs/setup/$2
                fi
            elif [ ! -z $1 ] && [ $1 == "keys" ]; then # copy public keys
                scp $SSH_TIMEOUT $HOST_CHECKING $host:$remote_setup/ssh_identities/keys/*.pub \
                    ssh_identities/rx/ssh/
            else
                echo "Error: No valid option"
                exit 1
            fi
        else
            echo "    Failed to reach $host"
        fi
    fi
}

register_locally() {
    source /etc/environment # reload envs
    if [ -z $MESH_IDENTITY ]; then # check if name is set
        echo "Error: MESH_IDENTITY has not been set"
        exit 1
    fi

    if [ -z $MESH_IP ]; then # check if ip is set
        echo "Error: MESH_IDENTITY has not been set"
        exit 1
    fi
    
    sudo hostname $MESH_IDENTITY # set hostname

    for pub_key in $(ls ssh_identities/keys/); do
        ssh-copy-id $SSH_TIMEOUT $HOST_CHECKING -f -i ssh_identities/keys/$pub_key $MESH_IDENTITY@localhost
        # note: currently all keys in 'ssh_identities/keys/' are trusted
    done
}

echo "Warning: This script is executed with root priviliges and uses
    the files present in 'system services/' to configure services on
    this machine. As this poses a security risk, make sure the content
    of the *.bash files and service_helper.bash script's content was
    not altered. Only files from 'system services/' and their
    respective service names should be used."
echo "Continue? y/N "
read continue
if [ -z $continue ] || [ ! $continue == "y" ]; then
    exit 0
fi

if [ $1 == "status" ]; then
    if ping -W 1 -c 1 google.com; then internet=yes; else internet=no; fi
    echo """
    You are logged in as: $(whoami)
    Hostname: $(hostname)
    Connection to internet: $internet
    Docker Swarm Leader: 
    Current environment variables:
        WLANDEV=$WLANDEV
        MESH_IDENTITY=$MESH_IDENTITY
        MESH_IP=$MESH_IP
    Active services: (active | inactive (unknown are also "inactive"))
        batman_adv_healthcheck   $(sudo systemctl is-active batman_adv_healthcheck.service)
        batman_adv_setup         $(sudo systemctl is-active batman_adv_setup.service)
        docker_leader            $(sudo systemctl is-active docker_leader.service)
        docker_init              $(sudo systemctl is-active docker_init.service)
        iw_dump                  $(sudo systemctl is-active iw_dump.service)
        rx_copy                  $(sudo systemctl is-active rx_copy.service)
    Enabled services: (enabled | disabled | not-found)
        batman_adv_healthcheck   $(sudo systemctl is-enabled batman_adv_healthcheck.service)
        batman_adv_setup         $(sudo systemctl is-enabled batman_adv_setup.service)
        docker_leader            $(sudo systemctl is-enabled docker_leader.service)
        docker_init              $(sudo systemctl is-enabled docker_init.service)
        iw_dump                  $(sudo systemctl is-enabled iw_dump.service)
        rx_copy                  $(sudo systemctl is-enabled rx_copy.service)
    Files:
        ~/.ssh/                  $(if [ -d ~/.ssh/ ]; then echo exists; fi)
        ~/.ssh/config            $(if [ -d ~/.ssh/ ] && [ -f ~/.ssh/config ]; then echo exists; fi)
        ~/.ssh/MESH_IDENTITY     $(if [ -d ~/.ssh/ ] && [ -f ~/.ssh/$MESH_IDENTITY ]; then echo exists; fi)
    Services: (.service and .timer files in /etc/systemd/system/)
        batman_adv_healthcheck   $(if [ -f /etc/systemd/system/batman_adv_healthcheck.service ]; then echo exists; fi)
        batman_adv_setup         $(if [ -f /etc/systemd/system/batman_adv_setup.service ]; then echo exists; fi)
        docker_leader            $(if [ -f /etc/systemd/system/docker_leader.service ]; then echo exists; fi)
        docker_init              $(if [ -f /etc/systemd/system/docker_init.service ]; then echo exists; fi)
        iw_dump                  $(if [ -f /etc/systemd/system/iw_dump.service ]; then echo exists; fi)
        rx_copy                  $(if [ -f /etc/systemd/system/rx_copy.service ]; then echo exists; fi)
    Services: (.bash scripts in /usr/local/bin)
        batman_adv_healthcheck   $(if [ -f /usr/local/bin/batman_adv_healthcheck.bash ]; then echo exists; fi)
        batman_adv_setup         $(if [ -f /usr/local/bin/batman_adv_setup.bash ]; then echo exists; fi)
        docker_leader            $(if [ -f /usr/local/bin/docker_leader.bash ]; then echo exists; fi)
        docker_init              $(if [ -f /usr/local/bin/docker_init.bash ]; then echo exists; fi)
        iw_dump                  $(if [ -f /usr/local/bin/iw_dump.bash ]; then echo exists; fi)
        rx_copy                  $(if [ -f /usr/local/bin/rx_copy.bash ]; then echo exists; fi)
    """
elif [ $1 == "wlandev" ]; then
    if [ -z $2 ]; then
        echo "Input your wireless interface (leave empty to use wlp0s20f3) "
        read wlandev
        if [ -z $wlandev ]; then
            wlandev=wlp0s20f3
        fi
    else
        wlandev=$2
    fi
    if [[ $wlandev == *['!'@#\$%^\&*\(\)+]* ]]; then
        echo "Error: Input contains illegal characters"
        exit 1
    fi
    # export WLANDEV=$wlandev
    sudo sed -i "/WLANDEV=/d" /etc/environment # delete old
    echo "WLANDEV=\"$wlandev\"" | sudo tee -a /etc/environment > /dev/null # append new
    echo "Exported WLANDEV=$wlandev"
    # export HOST_MAC
    if [ ! -z $wlandev ]; then
        # list interfaces; only show line containing $wlandev and line after that; get mac from second line
        host_mac=$(ip link | grep -A 1 $wlandev | awk '/link\/ether/ {print $2}')
        sudo sed -i "/HOST_MAC=/d" /etc/environment # delete old
        echo "HOST_MAC=\"$host_mac\"" | sudo tee -a /etc/environment > /dev/null # append new
        echo "Exported HOST_MAC=$host_mac"
    fi
    source /etc/environment
elif [ $1 == "send" ]; then
    if [ ! -z $2 ] && [ $2 == "keys" ]; then
        send_to_hosts keys $3 $4
    elif [ ! -z $2 ] && [ $2 == "hosts" ]; then
        send_to_hosts hosts $3
    else
        echo "Error: Choose either keys (public keys) or hosts (config, names, names_with_ip) to send"
        exit 1    
    fi
elif [ $1 == "collect" ]; then
    if [ ! -z $2 ] && [ $2 == "logs" ]; then
        collect logs $3
    else
        echo "Error: Only logs can be collected currentrly"
        exit 1    
    fi
elif [ $1 == "register" ]; then
    register_locally
elif [ $1 == "ssh" ]; then
    if [ ! -d ~/.ssh ]; then # check if .ssh/ exists
        mkdir ~/.ssh
    fi

    if [ -z $2 ]; then # ask for name
        echo "Input a name from 'names' "
        read name
    else
        name=$2
    fi

    generate=true
    if [ -z $name ]; then # no name
        echo "Error: No name was given"
        exit 1
    elif ! grep -Fxq "$name" ssh_identities/names; then # verify name
        # -F for fixed strings, -x to match whole line, -q for quiet
        echo "Error: Name not in 'names'"
        exit 1
    else
        if [ -f ssh_identities/$name ]; then # check if private key with name exists
            echo "Error: Private key already exists for this name"
            echo "Delete? y/N"
            read delete
            if [ $delete == "y" ]; then # delete existing key pair
                sudo rm ~/.ssh/$name &>/dev/null
                sudo rm ~/.ssh/$name.pub &>/dev/null
                sudo rm ssh_identities/$name &>/dev/null
                sudo rm ssh_identities/keys/$name.pub &>/dev/null
                echo "Deleted existing keys to given name"
            else
                generate=false
            fi
        elif [ -f ssh_identities/keys/$name.pub ]; then # check if public key with name exists
            echo "Error: Public key already exists for this name (likely already in use)"
            echo "Check if this name is in use before deleting 'ssh_identities/keys/$name.pub'" 
            echo "manually and trying again."
            echo "To delete use 'rm ssh_identities/keys/$name.pub'"
            exit 1
        fi
    fi

    # get ip from name to set envs
    mesh_ip=$(grep "$name" ssh_identities/names_with_ip | grep -oE "[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+")
    if [ -z $mesh_ip ]; then # no matching ip
        echo "Error: Found no valid ip address in 'names_with_ip'"
        exit 1
    else
        sudo sed -i "/MESH_IP=/d" /etc/environment # remove previous MESH_IP, if set
        echo "MESH_IP=\"$mesh_ip\"" | sudo tee -a /etc/environment > /dev/null # append new
        echo "Exported MESH_IP=\"$mesh_ip\""
	
	    sudo hostname $name
        sudo sed -i "/MESH_IDENTITY=/d" /etc/environment # remove previous MESH_IP, if set
        echo "MESH_IDENTITY=\"$name\"" | sudo tee -a /etc/environment > /dev/null # append new
        echo "Exported MESH_IDENTITY=\"$name\""
        source /etc/environment # load envs

        sudo chown -R $(whoami) ~/.ssh/
        touch ~/.ssh/known_hosts
        if [ $generate == true ]; then
            if ssh-keygen -f ~/.ssh/$name -N "" -C "$name@$name" -q; then # generate pair of keys
                # if sudo is used, fingerprint is generated for root@<hostname>
                # -C to replace comment root@<hostname> with $name@$name
                # -N to pass no passphrase
                echo "Generated '~/.ssh/$name' and '~/.ssh/$name.pub'"
            else
                echo "Error: Failed to generate keys"
            fi
            
            if cp ~/.ssh/$name.pub ssh_identities/keys/ 2>/dev/null \
                && cp ~/.ssh/$name ssh_identities/; then
                echo "Copied keys to 'ssh_identities/$name' and 'ssh_identities/keys/$name.pub'"
            else
                echo "Error: Failed to copy keys from ~/.ssh/ to ssh_identities/"
            fi
        fi

        echo "Copy hosts (file 'config') and ALL public keys in ssh_identities/keys/ except own to '~/.ssh'? (Y/n/config only)"
        echo "Warning: This will overwrite existing files"
        read move_to_ssh
        if [ $move_to_ssh == "n" ] || [ $move_to_ssh == "N" ]; then
            echo "Done"
        else
            if [ -f ~/.ssh/config ] && [ ! -f ~/.ssh/config.bak ]; then
                # make backup if config was previously present
                echo "Found existing config in '~/.ssh/'"
                echo "Creating backup of config..."
                mv ~/.ssh/config ~/.ssh/config.bak
                # prevent other hosts from being lost
                # otherwise would have to remove "identities" to prevent duplicates (lazy
                # solution, though if any hosts are present that are not also robots, it
                # is unlikely they can be reached anyway)
            fi
            
            if [ -z $move_to_ssh ] || [ $move_to_ssh == "config" ] || [ $move_to_ssh == "config only" ]; then
                cp ssh_identities/config ~/.ssh/
                sudo sed -i "s/own_name/$MESH_IDENTITY/" ~/.ssh/config # replace own_name with MESH_IDENTITY
            else
                cp ssh_identities/config ~/.ssh/
                sudo sed -i "s/own_name/$MESH_IDENTITY/" ~/.ssh/config
                cp ssh_identities/$MESH_IDENTITY ~/.ssh/
                for key in $(ls ssh_identities/keys/); do
                    if [ ! -z $key ] && [ ! $key == "$name.pub" ]; then
                        cp ssh_identities/keys/$key ~/.ssh/
                    fi
                done
            fi
            echo "Copied files"
        fi

        # Change hostname permanently (required for Docker)
        hostname=$(sudo head -1) # get old hostname
        echo "Changing $hostname to $MESH_IDENTITY permanently so robot can be identified easily in a swarm..."
        echo "To undo this, use 'sudo hostnamectl set-hostname <old hostname>'"
        # sudo sed -i "s/$hostname/$MESH_IDENTITY/" /etc/hostname # replace old hostname
        # sudo sed -i "s/\t$hostname/$MESH_IDENTITY/g" /etc/hosts
        sudo hostnamectl set-hostname $MESH_IDENTITY
        echo "Open a new terminal to see the changes"
    fi
elif [ $1 == "setup" ]; then
    if [ -z $WLANDEV ] || [ -z $MESH_IDENTITY ] || [ -z $MESH_IP ]; then
        echo "Error: Set all variables with options 'wlandev' and 'ssh' before setup"
        exit 1
    fi
    if [ ! $(id -u $MESH_IDENTITY) ]; then
        echo "Error: The given user does not exist"
        exit 1
    fi
    echo "Starting setup..."
    echo "Creating log files for services in 'logs/'..."
    touch logs/batman_adv_setup.log
    touch logs/batman_adv_healthcheck.log
    touch logs/iw_dump.log
    touch logs/docker_init.log
    touch logs/docker_leader.log
    touch logs/rx_copy.log
    
    echo "Copying service scripts..."
    sudo cp system\ services/*.bash /usr/local/bin/
    sudo chmod +x /usr/local/bin/batman_adv_setup.bash
    sudo chmod +x /usr/local/bin/batman_adv_healthcheck.bash
    sudo chmod +x /usr/local/bin/iw_dump.bash
    sudo chmod +x /usr/local/bin/docker_leader.bash
    sudo chmod +x /usr/local/bin/docker_init.bash
    sudo chmod +x /usr/local/bin/rx_copy.bash
    
    echo "Copying unit files and replacing placeholders..."
    for service in $(ls system\ services/ | grep .service); do
        cp system\ services/$service system\ services/$service.tmp # create copy
        sed -i "s/own_name/$MESH_IDENTITY/g" system\ services/$service.tmp # replace placeholder
        sudo mv system\ services/$service.tmp /etc/systemd/system/$service # move unit file
    done
    sudo cp system\ services/*.timer /etc/systemd/system/ 2>/dev/null # move timers (if any exist)
    
    sudo systemctl daemon-reload
    
    echo "Enabling services..."
    sudo systemctl enable batman_adv_setup.service
    sudo systemctl enable batman_adv_healthcheck.service
    sudo systemctl enable iw_dump.service
    sudo systemctl enable docker_init.service
    sudo systemctl enable rx_copy.service
    
    if [ -d ~/.ssh/ ] && [ -f ~/.ssh/authorized_keys ] && [ ! -f ~/.ssh/authorized_keys.bak ]; then
        cp ~/.ssh/authorized_keys ~/.ssh/authorized_keys.bak # create backup if none exists already
    fi
    echo "Done"
elif [ $1 == "cleanup" ]; then
    echo "Starting cleanup..."
    echo "Stopping services..."
    sudo systemctl stop batman_adv_healthcheck.service
    sudo systemctl stop batman_adv_setup.service
    sudo systemctl stop iw_dump.service
    sudo systemctl stop docker_leader.service
    sudo systemctl stop docker_init.service
    sudo systemctl stop rx_copy.service
    
    echo "Disabling services..."
    sudo systemctl disable batman_adv_healthcheck.service
    sudo systemctl disable batman_adv_setup.service
    sudo systemctl disable iw_dump.servie
    sudo systemctl disable docker_leader.service
    sudo systemctl disable docker_init.service
    sudo systemctl disable rx_copy.service
    
    echo "Removing unit files..."
    sudo rm -f /etc/systemd/system/batman_adv_setup.service
    sudo rm -f /etc/systemd/system/batman_adv_healthcheck.service
    sudo rm -f /etc/systemd/system/iw_dump.service
    sudo rm -f /etc/systemd/system/docker_leader.service
    sudo rm -f /etc/systemd/system/docker_init.service
    sudo rm -f /etc/systemd/system/rx_copy.service
    
    echo "Removing service scripts..."
    sudo rm -f /usr/local/bin/batman_adv_setup.bash
    sudo rm -f /usr/local/bin/batman_adv_healthcheck.bash
    sudo rm -f /usr/local/bin/iw_dump.bash
    sudo rm -f /usr/local/bin/docker_leader.bash
    sudo rm -f /usr/local/bin/docker_init.bash
    sudo rm -f /usr/local/bin/rx_copy.bash
    if [ ! -z $2 ] && [ $2 == "with_keys" ]; then
        echo "Removing keys from '.ssh/'..."
        if [ -f ~/.ssh/authorized_keys ]; then
            sudo rm ~/.ssh/authorized_keys
        fi
        if [ -f ~/.ssh/authorized_keys.bak ]; then
            mv ~/.ssh/authorized_keys.bak ~/.ssh/authorized_keys # restore backup
        fi
        rm ssh_identities/keys/
    fi
    echo "Done"
elif [ $1 == "restart" ]; then
    if [ -z $2 ]; then
	    sudo systemctl restart batman_adv_setup.service
	    sudo systemctl restart batman_adv_healthcheck.service
        sudo systemctl restart iw_dump.service
        # sudo systemctl restart docker_leader.service
        sudo systemctl restart docker_init.service
    elif [ $2 == "batman_adv_setup" ]; then
    	sudo systemctl restart batman_adv_setup.service
    elif [ $2 == "batman_adv_healthcheck" ]; then
    	sudo systemctl restart batman_adv_healthcheck.service
    elif [ $2 == "iw_dump" ]; then
        sudo systemctl restart iw_dump.service
    elif [ $2 == "docker_leader" ]; then
        sudo systemctl restart docker_leader.service
    elif [ $2 == "docker_init" ]; then
        sudo systemctl restart docker_init.service
    elif [ $2 == "rx_copy" ]; then
        sudo systemctl restart rx_copy.service
    else
        echo_valid_service
    fi
elif [ $1 == "stop" ]; then
    if [ -z $2 ]; then
	    sudo systemctl stop batman_adv_setup.service
	    sudo systemctl stop batman_adv_healthcheck.service
        sudo systemctl stop iw_dump.service
        sudo systemctl stop docker_leader.service
        sudo systemctl stop docker_init.service
        sudo systemctl stop rx_copy.service
        sudo docker swarm leave -f
    elif [ $2 == "batman_adv_setup" ]; then
    	sudo systemctl stop batman_adv_setup.service
    elif [ $2 == "batman_adv_healthcheck" ]; then
    	sudo systemctl stop batman_adv_healthcheck.service
    elif [ $2 == "iw_dump" ]; then
        sudo systemctl stop iw_dump.service
    elif [ $2 == "docker_leader" ]; then
        sudo systemctl stop docker_leader.service
    elif [ $2 == "docker_init" ]; then
        sudo systemctl stop docker_init.service
        sudo docker swarm leave -f
    elif [ $2 == "rx_copy" ]; then
        sudo systemctl stop rx_copy.service
    else
        echo_valid_service
    fi
elif [ $1 == "enable" ]; then
    if [ -z $2 ]; then
	    sudo systemctl enable batman_adv_setup.service
	    sudo systemctl enable batman_adv_healthcheck.service
        sudo systemctl enable iw_dump.service
        # sudo systemctl enable docker_leader.service
        sudo systemctl enable docker_init.service
        sudo systemctl enable rx_copy.service
    elif [ $2 == "batman_adv_setup" ]; then
    	sudo systemctl enable batman_adv_setup.service
    elif [ $2 == "batman_adv_healthcheck" ]; then
    	sudo systemctl enable batman_adv_healthcheck.service
    elif [ $2 == "iw_dump" ]; then
    	sudo systemctl enable iw_dump.service
    elif [ $2 == "rx_copy" ]; then
        sudo systemctl enable rx_copy.service
    else
        echo_valid_service
    fi
elif [ $1 == "disable" ]; then
    if [ -z $2 ]; then
	    sudo systemctl disable batman_adv_setup.service
	    sudo systemctl disable batman_adv_healthcheck.service
        sudo systemctl disable iw_dump.service
        sudo systemctl disable docker_leader.service
        sudo systemctl disable docker_init.service
        sudo systemctl disable rx_copy.service
    elif [ $2 == "batman_adv_setup" ]; then
    	sudo systemctl disable batman_adv_setup.service
    elif [ $2 == "batman_adv_healthcheck" ]; then
    	sudo systemctl disable batman_adv_healthcheck.service
    elif [ $2 == "iw_dump" ]; then
    	sudo systemctl disable iw_dump.service
    elif [ $2 == "docker_leader" ]; then
        sudo systemctl disable docker_leader.service
    elif [ $2 == "docker_init" ]; then
        sudo systemctl disable docker_init.service
    elif [ $2 == "rx_copy" ]; then
        sudo systemctl disable rx_copy.service
    else
        echo_valid_service
    fi
elif [ $1 == "managed" ]; then
    echo "Enable firewall? (Y/n)"
    read enable_fw
    if [ $enable_fw != "n" ] && [ $enable_fw != "N" ]; then
        sudo ufw enable
    fi

    sudo systemctl stop batman_adv_setup.service 2>/dev/null
    sudo systemctl stop batman_adv_healthcheck.service 2>/dev/null
    sudo systemctl stop iw_dump.service 2>/dev/null
    sudo systemctl stop docker_leader.service 2>/dev/null
    sudo systemctl stop docker_init.service 2>/dev/null

    sudo ip addr del "$MESH_IP/24" dev bat0
    sudo ip addr del "$MESH_IP/24" dev $WLANDEV

    sudo ip link set bat0 down 2>/dev/null
    sudo batctl if del $WLANDEV 2>/dev/null
    sudo ip link set $WLANDEV down
    sudo iwconfig $WLANDEV mode managed
    sudo ip link set $WLANDEV up
    echo "Done"
fi
