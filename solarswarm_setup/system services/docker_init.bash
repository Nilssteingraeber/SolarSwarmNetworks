#!/bin/bash
WORKER_TOKEN_LOCATION=~/solarswarm_setup/docker/worker_token
MANAGER_TOKEN_LOCATION=~/solarswarm_setup/docker/manager_token

sudo docker swarm init --advertise $ADV_IP
    echo $ADV_IP > "$DIR/swarm_tokens/ip"
    sudo docker swarm join-token -q worker > "$DIR/swarm_tokens/worker_token"
    sudo docker swarm join-token -q manager > "$DIR/swarm_tokens/manager_token"
    echo """$MAX_MANAGERS
    1""" > "$DIR/swarm_tokens/manager_count"
Leader
    löscht nach Boot altes Token
    ein Roboter wird jedes Mal manuell als Leader konfiguriert
    (nach neustart nicht mehr Leader)
    docker_leader.service kopiert `leader` jede Minute auf alle erreichbaren Hosts (Sonst meldet sich)
    docker init
        docker_leader.service zählt manager und worker
        pingt regelmäßig hosts
        wenn erreichbar, scp managager_token oder worker_token
        scp nach rx/docker/
        5 x manager_token, sonst worker_token (prüfen, ob erfolgreich?)

Sonst
    löscht nach Boot altes Token und `leader`
    wartet auf neue `leader`-Datei und fragt
    versucht regelmäßig, von hosts 
    rx_copy.service verschiebt Tokens von rx/docker/ nach docker/
    docker_init.service sucht in docker/ nach einem tokens
    betritt entweder als 