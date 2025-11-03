# System
## Voraussetzungen
Dieses Verzeichnis beinhaltet alle notwendigen Dateien für das Einrichten eines Roboters. Ein Roboter ist ein WLAN-fähiger Maschine mit Linux (Ubuntu) als Betriebssystem. Wegen der Anforderungen an dieses Projekt werden andere Distributionen oder ältere Versionen als Ubuntu 24.04 nicht getestet. Der Roboter oder Mock-Roboter sollte WLAN-fähig sein, eine aktuelle Docker Installation besitzen und für das Setup eine Internetverbindung haben. Für den Betrieb in einem Ad-hoc-Netzwerk wird zu dem eine Netzwerkkarte benötigt, die den Ad-hoc Modus (IBSS) unterstützt. Geprüft werden kann die verfügbare mit dem Befehl `iw list | grep -A 10 "Supported interface modes"`.

Es sollte bekannt sein, welche Roboter bereits welche Namen und IP-Adressen, die in `ssh_identities/names_with_ip` aufgeführt werden, verwenden. Für eine gute Organisation wird empfohlen, nach dem Einrichten des Nutzers (siehe unten) die Maschine mit Nutzernamen und entsprechender IP-Adresse zu beschriften und eine Liste mit vergebenen Namen zu pflegen.

> Hinweis: Das Verzeichnis `solarswarm_setup` muss in in `~` bzw. `/home/<Nutzername>/` liegen, da bereitgestellte Skripte und Services dort nach Dateien suchen oder sie ablegen.

Zum Einrichten des Ad-hoc Modus' werden zwei Kernel-Module (standardmäig vorinstalliert) und drei zusätzliche Tools benötigt, sowie ein weiteres zum Hosten eines SSH-Servers. Darunter ist `avahi-autoipd` optional und wird nur verwendet, wenn `batman_adv_setup.bash` manuell ausgeführt wird und keine statische IP-Adresse festgelegt wurde. Mehr dazu später.
```bash
    sudp apt-get update
    sudo apt-get install iw
    sudo apt-get install batctl
    sudo apt-get install avahi-autoipd
    # sudo apt-get install openssh-client # usually pre-installed
    sudo apt-get install openssh-server
```

Um Schlüssel zur Anmeldung verwenden zu können, muss in `/etc/ssh/sshd_config` die Zeile `#PubkeyAuthentification yes` gefunden und das "#" entfernt werden.
```bash
    sudo nano /etc/ssh/sshd_config # delete '#' in editor
    
    # or to remove automatically:
    sudo sed -i -e "s/#PubkeyAuthentification/PubkeyAuthentification/" /etc/ssh/sshd_config
```

## Nutzer einrichten
***Nutzername und IP-Adresse.*** Der Datei `ssh_identities/names_with_ip` sind vorgegebene Namen und IP-Adressen zu entnehmen. Die Namen entsprechenden dem kleingeschriebenen NATO-Alphabet (alfa, bravo, charlie, ...) und die IP-Adressen einer aufsteigenden Aufzählung von IPv4-Adressen beginnen mit 192.168.1.1. Die Listen `names` und `names_with_ip` können beliebig um eindeutige Namen ohne Sonderzeichen ergänzt werden, sollten aber stets auf allen Robotern synchron und konfliktfrei sein (neue öffentliche SSH-Schlüssel und SSH-Hostsmüssen mit anderen Robotern geteilt, siehe weiter unten). Wegen der in `system services/batman_adv_setup.bash` angegebenen Subnetzmaske 255.255.255.0 sind nur IP-Adressen von 192.168.1.1 bis 192.168.1.254 zuweisbar. Diese IP-Adressen werden dem virtuellen Netzwerk-Interface bat0 hinzugefügt und gelten daher nur im Ad-hoc Netzwerk. Im Managed Mode (Verbindung zu einem Access Point) haben sie keine Bedeutung. Diese Nutzernamen und statischen IP-Adressen werden verwendet, um (a) leichter Roboter auseinanderhalten zu können, (b) SSH einzurichten mit vordefinierten Hosts zum Austausch von Docker Swarm Join-Tokens, sowie Erweiterbarkeit und (c) zuverlässige Advertise-Adressen für Docker Swarm zu haben.

***Passwort.*** Einheitliche Passwörter wie "solarswarm-a" für den Nutzer `alfa` können dabei helfen, anderen Personen die Arbeit mit verschiedenen Robotern zu erleichtern. Ungeachtet dessen, ob individuelle Passwörter, ein Schema oder schlicht das selbe Passwort für alle Roboter eingesetzt werden, sollte gewährleistet sein, dass spätere Studierende oder Personal des Instituts für Elektromobilität an der Hochschule Bochum unbeschwert an den Robotern arbeiten können.

> Hinweis: Da nicht immer alle Roboter gleichzeitig eingeschaltet sind, ersparen wir uns automatische Updates dieser Listen. Mit `service_helper.bash send <hosts|keys>` können ... an alle **eingeschalteten** und **erreichbaren** Roboter, die auf der **lokalen** Liste `names_with_ip` vorhanden sind.

Mit Root-Rechten, füge einen Sudo-Nutzer (auch "Systemverwalter") mit einem Username aus`ssh_identities/names` hinzu, der noch nicht verwendet wird. Dies kann in den Systemeinstellungen unter "Benutzer" oder in der Konsole erledigt werden:
```bash
    sudo adduser <Username>
    sudo adduser <Username> sudo
```
Referenz: https://help.ubuntu.com/community/RootSudo#Allowing_other_users_to_run_sudo

Um den Betrieb der Roboter angenehmer zu machen, ist es ratsam, "Automatische Anmeldung" für diesen Nutzer einzuschalten. Der Toggel dafür befindet sich ebenfalls in den Systemeinstellungen unter "Benutzer". Lösungen für Roboter ohne Verwendung einer graphische Benutzeroberfläsche variieren je nach Display Manager und werden hier nicht aufgeführt, da sie nicht alle rekonstruiert werden können. Dennoch wird mit dieser Anmerkung auf Beispiele gängiger Lösung verwiesen:
- Gnome Desktop Manager (GDM): https://help.gnome.org/admin/system-admin-guide/stable/login-automatic.html.en
- Light Desktop Manager (lightdm): https://askubuntu.com/a/51087
- Headless/Server: https://ostechnix.com/ubuntu-automatic-login/

## Variablen und Labels einrichten
Drei wichtige Umgebungsvariablen müssen Systemweit definiert werden und Systemstarts überstehen: WLANDEV, MESH_IDENTITY und MESH_IP. MESH_IDENTITY und MESH_IP entsprechen dem bestimmten Nutzernamen und der dazugehörigen IP-Adresse. WLANDEV entspricht einem gültigen, WLAN-fähigen Netzwerk-Interface (alle einsehbar mit `ip link`). Wlan-Interfaces fangen üblicherweise mit "wlan" oder "wl" an (üblich ist "wlan0"). Die Intel NUCs verwenden die Bezeichnung "wlp0s20f3".

`service_helper.bash` hilft dabei, die Umgebungsvariablen zu setzen. WLANDEV kann mit `service_helper.bash wlandev [WLANDEV]` und MESH_IDENTITY und -IP mit `service_helper.bash ssh [Nutzername]` angegeben werden. Ohne optionalen Parameter wird eine Eingabe verlangt. MESH_IP wird aus `names_with_ip` vom Namen abgeleitet. `ssh` generiert zusätzlich ein SSH-Schlüsselpaar für Nutzername und IP.

## SSH einrichten und verwalten
Für SSH relevante Dateien sind in `ssh_identities/` enthalten und werden hier erklärt. Das Verzeichnis `keys` enthält alle öffentlichen Schlüssel *anderer* Roboter oder Hosts. Die Datei `config` enthält alle SSH-Hosts und muss um weitere Hosts ergänzt werden, wenn `names` und `names_with_ip` neue Einträge erhalten. MESH_IDENTITY wird für Host, User und die Schlüsselnamen und MESH_IP für HostName verwendet:
```
    Host example_name
            HostName 196.168.1.100
            User example_name
            IdentityFile ~/.ssh/example_name.pub
```

Namen und IP-Adressen in `config`, `names` und `names_with_ip` müssen sich stets decken. Hosts sollten in `~/ssh_identities/` und in `~/.ssh/` ihren privaten Schlüssel (für alfa beispielsweise `alfa`) und in `~/ssh_identities/keys` und in `~/.ssh` alle gültigen öffentlichen Schlüssel aller anderen Hosts (`bravo.pub`, `charlie`, ...) haben. Der private und öffentliche Schlüssel liefen im Setup-Verzeichnis an verschiedenen Stellen, da `keys/` als Ziel von `service_helper.bash send keys` gedacht ist und Dateien darin überschrieben werden können. Auch werden Schlüssel an zwei Stellen aufbewahrt, um den Austausch später leichter anpassen zu können.

Mit `service_helper.bash send <keys | hosts>` können entweder alle Schlüssel (Endung `.pub`)aus `ssh_identities/keys/` oder `config`, `names` und `names_with_ip` aus `ssh_identities/` auf alle **derzeitig erreichbaren** Hosts kopiert.

## service_helper.bash und Services ### UNFINISHED/TODO ###

### clipboard
Ausführen von `service_helper.bash`... ohne Parameter Anleitung... zuerst mit `ip link` WLANDEV herausfinden und mit `service_helper.bash wlandev` festlegen... Im Voraus Identität für den Roboter festlegen... mit `service_helper.bash ssh` festlegen, um MESH_IDENTITY und MESH_IP festzulegen...
`service_helper.bash setup`

Das Verzeichnis `system services/` beinhaltet drei Services: *batman_adv_setup* richtet Batman-Adv ein und tritt einem Ad-hoc Netzwerk bei. Wenn `enabled`, wird Batman-Adv nach Boot des Systems eingerichtet. *batman_adv_healthcheck* prüft, ob Batman-Adv Nachbarn sieht und startet *batman_adv_setup* neu, sollte ca. 25 Sekunden lang kein Nachbar gesichtet worden sein. *iw_dump* wird benötigt, um automatisiert Informationen zu Nachbarn im Netzwerk zu ermitteln und sie in Containern verfügbar zu machen. 

`service_helper.bash` hilt dabei, die Services auf einem Linux-System einzurichten. Das Skript informiert über gültige Parameter, wenn es ohne welche ausgeführt wird (etwa mit `bash service_helper.bash`). Zu beachten ist, dass das Skript für die meisten Befehle Root-Rechte benötigt. Daher vor der Ausführung sollte sichergestellt werden, dass nichts an diesem Skript oder den Services verändert wurde, was dem System schaden könnte.

> Hinweis: *batman_adv_setup* und *iw_dump* erfordern das Anlegen einer Umgebungsvariablen `WLANDEV`, welche ein gültiges Wlan-Interface nennen sollte. Mit `ip link` lassen sich Netzwerk-Interfaces anzeigen. Wlan-Interfaces fangen üblicherweise mit "wlan" oder "wl" an (üblich ist "wlan0"). Die Intel NUCs verfügen über "wlp0s20f3". Mit `sudo bash service_helper.bash wlandev` kann für eine angegebene Bezeichnung eine permanente Umgebungsvariable in `/etc/environment` angelegt werden. Sie wird überschrieben, falls bereits eine hinterlegt wurde.


## Dockerd Labels festlegen
Docker Swarm Services sollen später nur auf bestimmten Rechnern laufen, um leistungsschwächere Roboter nicht zu überlasten, auf systemspezifische Eingenschaften wie CPU-Architekturen zu berücksichtigen oder Daten auf dem Roboter vorauszusetzen. Beispielsweise ist es nur sinnig für einen Roboter, einen Service zum Ausliefern von Kartendaten zu starten, wenn er diese Daten lokal hat.

## Docker Join-Tokens austauschen
Problem: Wie viele Manager gibt es im Schwarm? Sind sie außer Reichweite oder ausgeschaltet?

Lösungsansatz: Roboter holen sich nach Start Tokens von anderen Hosts und treten als Worker bei. Der Leader promotiert bis zu 5 Manager und pingt diese regelmäßig an. Wenn sie länger nicht erreicht werden können, werden sie auf eine demotion-Liste verschoben und bei nächster Gelegenheit demotet und von der demotion-Liste entfernt. Ein anderer Worker wird währenddessen promotet.

Was ist node? ID, IP, MAC, ...?
docker node <promote | demote> <node>
docker node ls --filter node.label=<label>=<value>

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

# Weitere Roboter hinzufügen