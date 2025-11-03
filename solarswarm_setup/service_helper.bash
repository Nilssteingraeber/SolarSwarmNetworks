#!/bin/bash
# v0.9.2
SSH_TIMEOUT=1 # change if too short (was sufficient to connect to github.com in a connection test)

SW_SETUP=~/solarswarm_setup/
cd $SW_SETUP # requires parent directory 'solarswarm_setup' to be placed in '~'

source /etc/environment

if [ -z $1 ]; then
    echo """    Correct usage:
        bash service_helper [options] [additional options]
    Options:
        status
            show service status, presence of files, and envs
        wlandev [wireless interface]
            set WLANDEV to given wlan device
        ssh
            generate ssh keys with a name
        send <keys | hosts>
            copy keys or hosts (names, names_with_ip,
            config) onto all available hosts
    	setup
            copy files, make scripts executable, reload systemd,
    	    enable timer, and restart all services
    	cleanup
            remove services and files (except ssh keys and hosts)
    	restart [batman_adv_setup | batman_adv_healthcheck | iw_dump]
            restart all or a specific service (or timer)
    	stop [batman_adv_setup | batman_adv_healthcheck | iw_dump]
            stop all or a specific service (or timer)
    	enable [batman_adv_setup | batman_adv_healthcheck | iw_dump]
            enable all or a specific service (or timer)
    	disable [batman_adv_setup | batman_adv_healthcheck | iw_dump]
            disable all or a specific service (or timer)
        managed
            return from ad-hoc to managed mode (required for internet
            access)
    Notes:
        - The name provided for ssh must be in the files 'names' and
          match with a name and ip in 'names_with_ip' and config.
          Ensure that no two maschines use the same name or ip.
        - Use send option while all target robots are reachable to
          update changes to 'name', 'names_with_ip' and 'config' or
          to share one's own a new public key.
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
    iw_dump"""
}

send_to_hosts() { # copy files to all available ssh hosts
    source /etc/environment # reload envs
    if [ -z $MESH_IDENTITY ]; then # check if name is set
        echo "Error: MESH_IDENTITY has not been set"
        exit 1
    fi

    if [ -z $MESH_IP ]; then # check if ip is set
        echo "Error: MESH_IDENTITY has not been set"
        exit 1
    fi

    name_count=$(wc -l < ssh_identities/names) # get number of hosts
    # note: check for correct line breaks in names and names_with_ip using `cat -A <file>`
    # at the end of any line (including the last) should be a '$'
    # the lists were read as one line when typed on Windows and had to be replaced before
    i=1
    while [ $i -le $name_count ]; do
        host=$(head -$i ssh_identities/names | tail -1) # cut single host name
        host_ip=$(grep "$host" ssh_identities/names_with_ip | grep -oE "[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+")
        # -o 
        echo "Connecting to $host $host_ip..."
        if [ ping -c 1 $host_ip ]; then
            echo "Reached $host"
            if [ $1 == "keys" ]; then # copy public keys
                sudo scp -o ConnectTimeout=$SSH_TIMEOUT ssh_identities/keys/*.pub $host:$SW_SETUPssh_identities/rx/ssh/
                # host must have rx_copy.service active to copy it to their .ssh directory
            elif [ $1 == "hosts" ]; then # copy names, names_with_ip, and config to all reachable hosts
                sudo scp -o ConnectTimeout=$SSH_TIMEOUT ssh_identities/names $host:$SW_SETUPssh_identities/rx/ssh/
                sudo scp -o ConnectTimeout=$SSH_TIMEOUT ssh_identities/names_with_ip $host:$SW_SETUPssh_identities/rx/ssh/
                sudo scp -o ConnectTimeout=$SSH_TIMEOUT ssh_identities/config $host:$SW_SETUPssh_identities/rx/ssh/
            fi
        else
            echo "Failed to reach $host"
        fi
        ((i++))
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
if [ -z $continue ] || [ $continue == "N" ] || [ $continue == "n" ]; then
    exit 0
fi

if [ $1 == "status" ]; then
    if ping -W 1 -c 1 google.com; then internet=yes; else internet=no; fi
    echo """
You are logged in as: $(whoami)
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
    iw_dump (timer)          $(sudo systemctl is-active iw_dump.timer)
    rx_copy                  $(sudo systemctl is-active rx_copy.service)
Enabled services: (enabled | disabled | not-found)
    batman_adv_healthcheck   $(sudo systemctl is-enabled batman_adv_healthcheck.service)
    batman_adv_setup         $(sudo systemctl is-enabled batman_adv_setup.service)
    docker_leader            $(sudo systemctl is-enabled docker_leader.service)
    docker_init              $(sudo systemctl is-enabled docker_init.service)
    iw_dump (timer)          $(sudo systemctl is-enabled iw_dump.timer)
    rx_copy                  $(sudo systemctl is-enabled rx_copy.service)
Files:
    ~/.ssh/                  $(if [ -d ~/.ssh/ ]; then echo exists; fi)
    ~/.ssh/config            $(if [ -d ~/.ssh/ ] && [ -f ~/.ssh/config ]; then echo exists; fi)
    ~/.ssh/MESH_IDENTITY     $(if [ -d ~/.ssh/ ] && [ -f ~/.ssh/$MESH_IDENTITY ]; then echo exists; fi)
    /tmp/iw_dump/            $(if [ -d /tmp/iw_dump/ ]; then echo exists; fi)
    /tmp/iw_dump/iw_dump.txt $(if [ -d /tmp/iw_dump/ ] && [ -f /tmp/iw_dump/iw_dump.txt ]; then echo exists; fi)
Services: (.service and .timer files in /etc/systemd/system/)
    batman_adv_healthcheck   $(if [ -f /etc/systemd/system/batman_adv_healthcheck.service ]; then echo exists; fi)
    batman_adv_setup         $(if [ -f /etc/systemd/system/batman_adv_setup.service ]; then echo exists; fi)
    docker_leader            $(if [ -f /etc/systemd/system/docker_leader.service ]; then echo exists; fi)
    docker_init              $(if [ -f /etc/systemd/system/docker_init.service ]; then echo exists; fi)
    iw_dump (timer)          $(if [ -f /etc/systemd/system/iw_dump.timer ]; then echo exists; fi)
    rx_copy                  $(if [ -f /etc/systemd/system/rx_copy.service ]; then echo exists; fi)
Services: (.bash scripts in /usr/local/bin)
    batman_adv_healthcheck   $(if [ -f /usr/local/bin/batman_adv_healthcheck.bash ]; then echo exists; fi)
    batman_adv_setup         $(if [ -f /usr/local/bin/batman_adv_setup.bash ]; then echo exists; fi)
    docker_leader            $(if [ -f /usr/local/bin/docker_leader.bash ]; then echo exists; fi)
    docker_init              $(if [ -f /usr/local/bin/docker_init.bash ]; then echo exists; fi)
    iw_dump (timer)          $(if [ -f /usr/local/bin/iw_dump.bash ]; then echo exists; fi)
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
    sudo sed -i "/WLANDEV=/d" /etc/environment
    echo "WLANDEV=\"$wlandev\"" | sudo tee -a /etc/environment > /dev/null
    echo "Exported WLANDEV=$wlandev"
    source /etc/environment
elif [ $1 == "send" ]; then
    if [ $2 == "keys"]; then
        send_to_hosts keys
    elif [ $2 == "hosts" ]; then
        send_to_hosts hosts
    else
        echo "Error: Choose either keys (public keys) or hosts (config, names, names_with_ip) to send"
        exit 1    
    fi
elif [ $1 == "ssh" ]; then
    if [ ! -d ~/.ssh ]; then # check if .ssh/ exists
        sudo mkdir ~/.ssh
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
        if sudo ls ~/.ssh/ | grep -Fq "$name"; then # check if private key with name exists
            echo "Error: Private key already exists for this name"
            echo "Delete? y/N"
            read delete
            if [ $delete == "y" ]; then # delete existing key pair
                rm ssh_identities/$name
                rm ssh_identities/keys/$name.pub
                sudo rm ~/.ssh/$name
                sudo rm ~/.ssh/$name.pub
                echo "Deleted existing keys to given name"
            else
                generate=false
            fi
        elif sudo ls ~/.ssh/ | grep -F "$name.pub"; then # check if public key with name exists
            echo "Error: Public key already exists for this name (likely already in use)"
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

        sudo sed -i "/MESH_IDENTITY=/d" /etc/environment # remove previous MESH_IP, if set
        echo "MESH_IDENTITY=\"$name\"" | sudo tee -a /etc/environment > /dev/null # append new
        echo "Exported MESH_IDENTITY=\"$name\""
        source /etc/environment # load envs

        if [ generate == true ]; then
            # use local repostories of keys in case scp (used in send option) can not overwrite
            sudo ssh-keygen -R -f ssh_identities/$name -N "" # generate pair of keys
            # -R to replace old keys
            # -N to pass no passphrase
            mv ssh_identities/$name.pub ssh_identities/keys/ # move public key
            echo "Generated 'ssh_identities/$name' and 'ssh_identities/keys/$name.pub'"
        fi

        if [ -f ~/.ssh/config ] && [ ! -f ~/.ssh/config.backup ]; then # make backup if config was previously present
            sudo mv ~/.ssh/config ~/.ssh/config.bak
            echo "Found existing config in '~/.ssh/'"
            echo "Creating backup of config..."
            # prevent other hosts from being lost
            # otherwise would have to remove "identities" to prevent duplicates (lazy
            # solution, though if any hosts are present that are not also robots, it
            # is unlikely they can be reached anyway)
        fi

        echo "Copy hosts and ALL local keys to '~/.ssh'? (Y/n)"
        read move_to_ssh
        if [ $move_to_ssh == "n" ] || [ $move_to_ssh == "N" ]; then
            echo "Done"
        else
            sudo cp ssh_identities/config ~/.ssh/
            sudo cp ssh_identities/$MESH_IDENTITY ~/.ssh/
            sudo cp ssh_identities/keys/* ~/.ssh/
            echo "Copied files"
        fi
    fi
elif [ $1 == "setup" ]; then
    if [ -z $WLANDEV ] || [ -z $MESH_IDENTITY ] || [ -z $MESH_IP ]; then
        echo "Error: Set all variables with options 'wlandev' and 'ssh' before setup"
        exit 1
    fi
    echo "Starting setup..."
    mkdir /tmp/iw_dump
    sudo cp system\ services/*.bash /usr/local/bin/
    sudo chmod +x /usr/local/bin/batman_adv_setup.bash
    sudo chmod +x /usr/local/bin/batman_adv_healthcheck.bash
    sudo chmod +x /usr/local/bin/iw_dump.bash
    
    sudo cp system\ services/*.service /etc/systemd/system/
    sudo cp system\ services/*.timer /etc/systemd/system/
    
    sudo systemctl daemon-reload
    sudo systemctl restart batman_adv_setup.service
    sudo systemctl restart batman_adv_healthcheck.service
    
    sudo systemctl enable batman_adv_setup.service
    sudo systemctl enable batman_adv_healthcheck.service
    sudo systemctl enable --now iw_dump.timer
    echo "Done"
elif [ $1 == "cleanup" ]; then
    echo "Starting cleanup..."
    sudo systemctl stop batman_adv_healthcheck.service
    sudo systemctl stop batman_adv_setup.service
    
    sudo systemctl disable batman_adv_healthcheck.service
    sudo systemctl disable batman_adv_setup.service
    sudo systemctl disable --now iw_dump.timer 1> /dev/null
    
    sudo rm -f /etc/systemd/system/batman_adv_setup.service
    sudo rm -f /etc/systemd/system/batman_adv_healthcheck.service
    sudo rm -f /etc/systemd/system/iw_dump.service
    sudo rm -f /etc/systemd/system/iw_dump.timer
    sudo rm -f /usr/local/bin/batman_adv_setup.bash
    sudo rm -f /usr/local/bin/batman_adv_healthcheck.bash
    rm -rf /tmp/iw_dump/
    echo "Done"
elif [ $1 == "restart" ]; then
    if [ -z $2 ]; then
	    sudo systemctl restart batman_adv_setup.service
	    sudo systemctl restart batman_adv_healthcheck.service
        sudo systemctl start iw_dump.timer
    elif [ $2 == "batman_adv_setup" ]; then
    	sudo systemctl restart batman_adv_setup.service
    elif [ $2 == "batman_adv_healthcheck" ]; then
    	sudo systemctl restart batman_adv_healthcheck.service
    elif [ $2 == "iw_dump" ]; then
        sudo systemctl restart iw_dump.timer
    else
        echo_valid_service
    fi
elif [ $1 == "stop" ]; then
    if [ -z $2 ]; then
	    sudo systemctl stop batman_adv_setup.service
	    sudo systemctl stop batman_adv_healthcheck.service
        sudo systemctl stop iw_dump.timer
    elif [ $2 == "batman_adv_setup" ]; then
    	sudo systemctl stop batman_adv_setup.service
    elif [ $2 == "batman_adv_healthcheck" ]; then
    	sudo systemctl stop batman_adv_healthcheck.service
    elif [ $2 == "iw_dump" ]; then
        sudo systemctl stop iw_dump.timer # useless?
    else
        echo_valid_service
    fi
elif [ $1 == "enable" ]; then
    if [ -z $2 ]; then
	    sudo systemctl enable batman_adv_setup.service
	    sudo systemctl enable batman_adv_healthcheck.service
        sudo systemctl enable iw_dump.timer
    elif [ $2 == "batman_adv_setup" ]; then
    	sudo systemctl enable batman_adv_setup.service
    elif [ $2 == "batman_adv_healthcheck" ]; then
    	sudo systemctl enable batman_adv_healthcheck.service
    elif [ $2 == "iw_dump" ]; then
    	sudo systemctl enable iw_dump.timer 1> /dev/null
    else
        echo_valid_service
    fi
elif [ $1 == "disable" ]; then
    if [ -z $2 ]; then
	    sudo systemctl disable batman_adv_setup.service
	    sudo systemctl disable batman_adv_healthcheck.service
    elif [ $2 == "batman_adv_setup" ]; then
    	sudo systemctl disable batman_adv_setup.service
    elif [ $2 == "batman_adv_healthcheck" ]; then
    	sudo systemctl disable batman_adv_healthcheck.service
    elif [ $2 == "iw_dump" ]; then
    	sudo systemctl disable iw_dump.timer 1> /dev/null
    else
        echo_valid_service
    fi
elif [ $1 == "managed" ]; then
    sudo systemctl stop batman_adv_setup.service 2>/dev/null
    sudo systemctl stop batman_adv_healthcheck.service 2>/dev/null
    sudo systemctl stop iw_dump.timer 2>/dev/null

    sudo ip addr del "$MESH_IP/24" dev bat0
    sudo ip addr del "$MESH_IP/24" dev $WLANDEV

    sudo ip link set bat0 down 2>/dev/null
    sudo batctl if del $WLANDEV 2>/dev/null
    sudo ip link set $WLANDEV down
    sudo iwconfig $WLANDEV mode managed
    sudo ip link set $WLANDEV up
fi
