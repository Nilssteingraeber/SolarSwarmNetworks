# Einrichtung des Systems
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

## Nutzer einrichten
***Nutzername und IP-Adresse.*** Der Datei `ssh_identities/names_with_ip` sind vorgegebene Namen und IP-Adressen zu entnehmen. Die Namen entsprechenden dem kleingeschriebenen NATO-Alphabet (alfa, bravo, charlie, ...) und die IP-Adressen einer aufsteigenden Aufzählung von IPv4-Adressen beginnen mit 192.168.1.1. Die Listen `names` und `names_with_ip` können beliebig um eindeutige Namen ohne Sonderzeichen ergänzt werden, sollten aber stets auf allen Robotern synchron und konfliktfrei sein (neue öffentliche SSH-Schlüssel und SSH-Hostsmüssen mit anderen Robotern geteilt, siehe weiter unten). Wegen der in `system services/batman_adv_setup.bash` angegebenen Subnetzmaske 255.255.255.0 sind nur IP-Adressen von 192.168.1.1 bis 192.168.1.254 zuweisbar. Diese IP-Adressen werden dem virtuellen Netzwerk-Interface bat0 hinzugefügt und gelten daher nur im Ad-hoc Netzwerk. Im Managed Mode (Verbindung zu einem Access Point) haben sie keine Bedeutung. Diese Nutzernamen und statischen IP-Adressen werden verwendet, um (a) leichter Roboter auseinanderhalten zu können, (b) SSH einzurichten mit vordefinierten Hosts zum Austausch von Docker Swarm Join-Tokens, sowie Erweiterbarkeit und (c) zuverlässige Advertise-Adressen für Docker Swarm zu haben.

***Passwort.*** Einheitliche Passwörter wie "solarswarm-a" für den Nutzer `alfa` können dabei helfen, anderen Personen die Arbeit mit verschiedenen Robotern zu erleichtern. Ungeachtet dessen, ob individuelle Passwörter, ein Schema oder schlicht das selbe Passwort für alle Roboter eingesetzt werden, sollte gewährleistet sein, dass spätere Studierende oder Personal des Instituts für Elektromobilität an der Hochschule Bochum unbeschwert an den Robotern arbeiten können.

> Hinweis: Da nicht immer alle Roboter gleichzeitig eingeschaltet sind, ersparen wir uns automatische Updates dieser Listen. Mit `service_helper.bash send <hosts|keys>` können Hosts aus `ssh_identities/` und öffentliche Schlüssel aus `ssh_identities/keys/` an alle **eingeschalteten** und **erreichbaren** Roboter, die auf der **lokalen** Liste `names_with_ip` vorhanden sind, geschickt werden. Sie werden auf dem Zielhost in `rx/` abgelegt und von dem Service rx_copy geprüft und an die richtige Stelle verschoben.

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

## Variablen einrichten
Drei wichtige Umgebungsvariablen müssen Systemweit definiert werden und Systemstarts überstehen: WLANDEV, MESH_IDENTITY und MESH_IP. MESH_IDENTITY und MESH_IP entsprechen dem bestimmten Nutzernamen und der dazugehörigen IP-Adresse. WLANDEV entspricht einem gültigen, WLAN-fähigen Netzwerk-Interface (einsehbar mit `ip link`). Wlan-Interfaces fangen üblicherweise mit "wlan" oder "wl" an (üblich ist "wlan0"). Die Intel NUC Mini-PCs verwenden die Bezeichnung "wlp0s20f3".

`service_helper.bash` hilft dabei, die Umgebungsvariablen zu setzen. WLANDEV kann mit `service_helper.bash wlandev [WLANDEV]` und MESH_IDENTITY und -IP mit `service_helper.bash ssh [Nutzername]` gesetzt werden. Ohne optionalen Parameter wird eine Eingabe verlangt. MESH_IP wird aus `ssh_identities/names_with_ip` vom Namen abgeleitet. `ssh` generiert zusätzlich ein SSH-Schlüsselpaar für Nutzername und IP.

## SSH einrichten und verwalten
Für SSH relevante Dateien sind in `ssh_identities/` enthalten und werden hier erklärt. Das Unterverzeichnis `keys/` enthält alle öffentlichen Schlüssel bekannter Roboter oder Hosts. Die in `ssh_identities/` enthaltene Datei `config` enthält alle SSH-Hosts und muss um weitere Hosts ergänzt werden, wenn `names` und `names_with_ip` neue Einträge erhalten. Ohne einen gültigen Eintrag in `config` können die Skripte den Host nicht erreichen, da sie Hostnamen beispielsweise für scp verwenden. MESH_IDENTITY wird für Host, User und die Schlüsselnamen und MESH_IP für HostName verwendet:
```
Host example_name
        HostName 196.168.1.100
        User example_name
        IdentityFile ~/.ssh/example_name.pub
```

> Hinweis: Die in `config` definierten Hosts gelten nur für SSH. Um diese Hosts auch für andere Programme gültig zu machen, müssen sie in `/etc/hosts` inkludiert werden. In diesem Beispiel ist eine Zeile `196.168.1.100 example_name` erforderlich.

Namen und IP-Adressen in `config`, `names` und `names_with_ip` müssen sich stets decken. Hosts sollten in `~/ssh_identities/` und in `~/.ssh/` ihren privaten Schlüssel (für alfa beispielsweise `alfa`) und in `~/ssh_identities/keys` und in `~/.ssh` alle gültigen öffentlichen Schlüssel aller anderen Hosts (`bravo.pub`, `charlie`, ...) haben. Der private und öffentliche Schlüssel liefen im Setup-Verzeichnis an verschiedenen Stellen, da `keys/` als Ziel von `service_helper.bash send keys` gedacht ist und Dateien darin überschrieben werden können. Auch werden Schlüssel an zwei Stellen aufbewahrt, um den Austausch später leichter anpassen zu können.

Mit `service_helper.bash send <keys | hosts>` können entweder alle Schlüssel (Endung `.pub`) aus `ssh_identities/keys/` oder `config`, `names` und `names_with_ip` aus `ssh_identities/` auf alle **derzeit erreichbaren** Hosts kopiert werden.

## Docker Node Labels
Docker Swarm Services sollen später nur auf bestimmten Rechnern laufen, um leistungsschwächere Roboter nicht zu überlasten, systemspezifische Eingenschaften wie CPU-Architekturen zu berücksichtigen und Daten auf dem Roboter vorauszusetzen. Beispielsweise ist es nur sinnvoll für einen Roboter, einen Service zum Ausliefern von Kartendaten zu starten, wenn er eine lokale Kopie dieser Daten besitzt.

Welche Labels eine Docker Node (ein einzelner Host in dem Schwarm) haben muss, um einen Swarm Service replizieren zu können, wird in `~/solarswarm_run/docker-compose.yaml` festgelegt. Die von uns verwendeten Labels werden in `docker/labels_usage.md` aufgelistet. Hier ein verkürztes Beispiel für einen Service:
```yaml
fibonacci_example:
    # ...
    deploy:
        mode: replicated
        replicas: 2
        max_replicas_per_node: 1
        placement:
            constraint:
            - node.labels.ram_ge_8gb # <- das Label ram_ge_8gb muss vorhanden sein
            - node.labels.architecture=x86_64 # <- das Label architecture muss den Wert x86_64 haben
```
> Hinweis: Werden mehrere Constraints eingetragen, werden die Wahrheitswerte aller Einträge mit einem logischen UND verbunden. Dieser Service startet nur auf Nodes, die beide Labels (mit den selben Werten) besitzen.

Labels können einer Node erst nach Beitritt in einen Schwarm und nur durch einen Manager zugewiesen werden. Damit eine Node selbst verkündet, welche Labels sie von einem Manager zugewiesen bekommt, soll sie eine Datei mit ihren gewünschten Labels bereitstellen. Dazu muss in `docker/` eine Datei, benannt nach dem Hostnamen mit der Endung `.labels` (etwa `bravo.logs`), erstellt und zeilenweise mit Labels gefüllt werden. `docker/example_name.labels` bietet ein Beispiel. `docker/labels_usage.md` einhält weitere Hinweise für die Vergabe von Labels und sollte um benutzerdefinierte Labels erweitert werden, damit sie stets als Katalog dienen kann. Dabei ist zu bedenken, dass die Kataloge der Roboter, die bereits eingerichtet wurden, aktualisiert werden.
Sobald ein Roboter den Systemd Service *docker_init* (Skript `system\ services/docker_init.bash`) startet und beitritt, wird seine Availability auf "drain" gestzt, wordurch er keine Services repliziert. Er versucht wiederholt, seine Labels-Datei an die Leader-Node zu senden, welche für eine valide Labels-Datei die darin stehenden Labels dem Roboter verleiht und sine Availability auf "active" setzt. Dadurch ist er integriert und kann Services replizieren. Eine Konstante in `docker_init.bash` kann verändert werden, um trotz fehlender Labels-Datei beizutreten:
```bash
IGNORE_LABELS_MISSING=false # falls true, wird eine leere <Hostname>.labels erzeugt, insofern keine vorhanden ist
```

## Systemd Services und service_helper.bash
Das Skript `service_helper.bash` übernimmt viele repetative Aufgaben und hilft beim Einrichten, Warten und Entfernen wichtiger Dateien, Variablen und Systemd Services. Die Ausführung erfolgt (von diesem Verzeichnis aus) `bash service_helper.bash` oder `./service_helpeer.bash`. Da viele Programmaufrufe innerhalb des Skripts administrative Rechte benötigen, wird anfangs eine Warnung angezeigt und zu einer Bestätigung der Ausführung aufgerufen. Für die Sicherheit ist zu beachten, dass das Passwort eines Sudo-Users nur einmal abgefragt wird und alle weiteren Programme, die vorneran ein `sudo` verzeichnen, ohne weitere Abfragen ausgeführt werden. So kann jede fremde Zeile Code mit Sudo-Rechten ausgeführt werden, wenn das Skript mit diesen Rechten ausgeführt wird.

Das Skript gibt ohne übergebene Parameter eine Übersicht der möglichen Optionen und Parameter aus. Optionen und Parameter werden durch Leerzeichen voneinenader und von dem Programmnamen vor der Auführung angegeben. Zunächst ein paar Beispiele:
```bash
bash service_helper.bash wlandev wlp0s20f3 # setzt WLANDEV
bash service_helper.bash ssh bravo # MESH_IDENTITY auf bravo und MESH_IP auf 192.168.1.2
bash service_helper.bash setup # bereitet Systemd Services vor
bash service_helper.bash status # zeigt an, welche Variablen und Dateien vorhanden sind
```

## Bestimmen eines Leaders
Der Leader (Anführer) in einem Docker Swarm ist derjenige Manager, der den Schwarm initiiert hat. Er stellt Join-Tokens aus, mit denen andere Hosts als Worker- oder Manager-Nodes beitreten können. Fällt ein Leader aus und wird das Quorum dadurch nicht zerstört, wird ein weiterer Manager zum Leader ernannt. Für unseren Schwarm übernimmt der Leader zusätzliche Aufgauben wie das Zuweisen der Node Labels nach Beitritt oder das Ernennen und Entfernen von Managern zur Einhaltung einer in `docker_leader.bash` als Konstante `IDEAL_MANAGER_COUNT` angegebenen Anzahl. Nur der Leader darf den Systemd Service *docker_leader* ausführen.

Ob ein Roboter Leader sein soll, muss er bereits vor dem Start von *docker_init* wissen. Dazu soll er im Verzeichnis `docker/` eine Datei `leader` mit seinem eigenen Namen erhalten. Beispielsweise erhält Host alfa eine Datei `leader`, die nur "alfa" beinhaltet. Alle weiteren Roboter versuchen nach dem Start von *docker_init*, den Leader zu finden und seinem Schwarm beizutreten. Dazu werden `leader` und `worker_token` vom Leader oder anderen erreichbaren Hosts geholt.

## Logging
Einige Systemd Services haben eigene Konstanten `LOG_OUT`, welche das Ziel für Logs bestimmt. Diese Konstante kann je Service angepasst werden. Generell ist für Logs von Programmen in `solarswarm_setup` `solarswarm_setup/logs` und Docker Services in `solarswarm_run` `solarswarm_run/logs/` vorgesehen.

Mit `bash service_helper.bash collect logs <all | hostname>` werden die Logs aus `solarswarm_setup/logs` und `solarswarm_run/logs` von allesn erreichbaren Hosts in `~/solarswarm_setup/rx/logs/` des lokalen Hosts kopiert. `rx/logs/` hat die Unterverzeichnisse `setup/` und `run/`, in denen Verzeichnisse mit dem Hostnamen der Quelle angelegt und befüllt werden.

v----- TODO -----v

# Nutzung
## Starten
## Problembehandlung
## Beenden
## Sammeln der Logs

# Erweiterung
Wie erweitern? Wie neue Systemd Services, ROS2 Nodes oder Docker Services hinzufüen?
Kopie von custom_interfaces in solarswarm_run/ros_ws synchronisieren und neu bauen
## Hinzufügen neuer Hostnamen und IP-Adressen
## Integrieren eines Systemd Services
Speicherort für Unit-File und Bash-Skript
Integrieren in services_helper.bash
    Hilfe erweitern
    If-elif-Struktur erweitern (erst Struktur ohne Blöcke zeigen)
## Hinzufügen neuer Docker Services oder ROS2 Nodes
Verweis auf `~/solarswarm_run/Anleitung\ zur\ Erweiterung.md`
## Hinzufügen neuer Docker Node Labels


# Zwischenablage
Ausführen von `service_helper.bash`... ohne Parameter Anleitung... zuerst mit `ip link` WLANDEV herausfinden und mit `service_helper.bash wlandev` festlegen... Im Voraus Identität für den Roboter festlegen... mit `service_helper.bash ssh` festlegen, um MESH_IDENTITY und MESH_IP festzulegen...
`service_helper.bash setup`

Das Verzeichnis `system services/` beinhaltet drei Services: *batman_adv_setup* richtet Batman-Adv ein und tritt einem Ad-hoc Netzwerk bei. Wenn `enabled`, wird Batman-Adv nach Boot des Systems eingerichtet. *batman_adv_healthcheck* prüft, ob Batman-Adv Nachbarn sieht und startet *batman_adv_setup* neu, sollte ca. 25 Sekunden lang kein Nachbar gesichtet worden sein. *iw_dump* wird benötigt, um automatisiert Informationen zu Nachbarn im Netzwerk zu ermitteln und sie in Containern verfügbar zu machen. 

`service_helper.bash` hilt dabei, die Services auf einem Linux-System einzurichten. Das Skript informiert über gültige Parameter, wenn es ohne welche ausgeführt wird (etwa mit `bash service_helper.bash`). Zu beachten ist, dass das Skript für die meisten Befehle Root-Rechte benötigt. Daher vor der Ausführung sollte sichergestellt werden, dass nichts an diesem Skript oder den Services verändert wurde, was dem System schaden könnte.

> Hinweis: *batman_adv_setup* und *iw_dump* erfordern das Anlegen einer Umgebungsvariablen `WLANDEV`, welche ein gültiges Wlan-Interface nennen sollte. Mit `ip link` lassen sich Netzwerk-Interfaces anzeigen. Wlan-Interfaces fangen üblicherweise mit "wlan" oder "wl" an (üblich ist "wlan0"). Die Intel NUCs verfügen über "wlp0s20f3". Mit `sudo bash service_helper.bash wlandev` kann für eine angegebene Bezeichnung eine permanente Umgebungsvariable in `/etc/environment` angelegt werden. Sie wird überschrieben, falls bereits eine hinterlegt wurde.

--- Docker Join-Tokens austauschen ---
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

--- Weitere Roboter hinzufügen ---