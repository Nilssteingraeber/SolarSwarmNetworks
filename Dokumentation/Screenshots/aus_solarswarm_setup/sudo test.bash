#!/bin/bash
# getestet auf alfa

# scp ssh_identities/names bravo:~/solarswarm_setup/ssh_identities/
# gelingt mit bash
# gelingt nicht mit sudo bash

# sudo ls
# scp ssh_identities/names bravo:~/solarswarm_setup/ssh_identities/
# gelingt mit bash
# erfordert Passwort mit sudo -u alfa

sudo -u alfa ls
scp ssh_identities/names bravo:~/solarswarm_setup/ssh_identities/
# gelingt mit bash
# erfordert Passwort mit sudo
# gelingt mit sudo -u alfa
