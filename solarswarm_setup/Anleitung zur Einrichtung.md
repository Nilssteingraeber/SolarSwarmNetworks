# Einrichtung des Systems
## Voraussetzungen
Ein Roboter ist ein WLAN-fähiger Maschine mit Linux (Ubuntu) als Betriebssystem. Wegen der Anforderungen an dieses Projekt werden andere Distributionen oder ältere Versionen als Ubuntu 24.04 nicht getestet. Der Roboter oder Mock-Roboter sollte WLAN-fähig sein, eine aktuelle Docker Installation besitzen und für das Setup eine Internetverbindung haben. Für den Betrieb in einem Ad-hoc-Netzwerk wird zu dem eine Netzwerkkarte benötigt, die den Ad-hoc Modus (IBSS) unterstützt. Geprüft werden kann die verfügbare mit dem Befehl `iw list | grep -A 10 "Supported interface modes"`.

## Nutzer einrichten
Öffne die Einstellungen über die App-Suche oder die Icons in der oberen, rechten Ecke und navigiere zu Users und schaue nach, ob der Roboter einen Nutzer mit einem Namen aus dem NATO-Alphabet (kleingeschrieben) hat. Falls ja, konfiguriere den Nutzer so, dass er (a) Administratorrechte hat und (b) nach Systemstart automatisch angemeldet wird. Fall nein, lege einen neuen Nutzer mit einem Namen aus der untenstehenden Liste an. Dieser sollte (a) und (b) erfüllen. Als Passwort wird geraten, **solarswarm-#** zu nehmen, wobei "#" der Anfangsbuchstabe (klein) des gewählten Namen ist.

Beispiel: Nutzername `zulu` mit Passwort `solarswarm-z`

> Hinweis: `echo` wird nur als `ecco` akzeptiert, um Verwechselung und Konflikte zu vermeiden.

> Hinweis: Tausche dich mit anderen Personen aus und stelle sicher, dass der gewählte Name noch nicht vergeben wurde. Vergebene Namen müssen steets eindeutig sein und Roboter sollten nicht mehr als einen Nutzer mit einem Namen aus dem NATO-Alphabet erhalten. Dieser Name wird als Bezeichner für den Roboter (auch "Host") verwendet. Bestenfalls werden die Roboter mit ihrem Namen beklebt, damit sie leichter aueinander zu halten sind.

NATO-Alphabet:
| | | | | | | |
|------|-------|---------|-------|------|---------|------|
| alfa | bravo | charlie | delta | ecco | foxtrot | golf |
| hotel | india | juliett | kilo | lima | mike | november |
| oscar | papa | quebec | romeo | sierra | tango | uniform |
| victor | whiskey | xray | yankee | zulu | | |

## Notwendige Programme und Docker Images installieren
Kopiere aus dem Projektverzeichnis die Verzeichnisse `solarswarm_setup/` und `solarswarm_run/`. Führe dazu folgende Befehle in einer Konsole aus:
```bash
git clone https://github.com/Nilssteingraeber/SolarSwarmNetworks.git`
sudo copy -R SolarSwarmNetworks/solarswarm_setup/ ~/solarswarm_setup
sudo copy -R SolarSwarmNetworks/solarswarm_run/ ~/solarswarm_run
```

Führe folgende Befehle in einer Konsole aus:
```bash
sudo apt-get update
sudo apt-get install iw
sudo apt-get install batctl
sudo apt-get install avahi-autoipd
sudo apt-get install openssh-server
```

> Tipp: Über die Pfeiltasten (hoch und runter) kannst du vergangene Befehle wieder auswählen, um sie nicht erneut tippen zu müssen.

Stelle sicher, dass Docker installiert ist. Gebe `which docker` in die Konsole ein. Wird anschließend ein Pfad (etwa "/usr/bin/docker") ausgegeben, ist Docker bereits installiert. Wird kein Pfad ausgegeben, folge bitte dem offiziellen Installationsguide, um Docker zu installieren: https://docs.docker.com/engine/install/ubuntu/

Führe folgende Befehle in einer Konsole aus:
```bash
cd ~/solarswarm_run
sudo docker compose build # kann etwas dauern, erfordert eine Internetverbindung
cd ~/solarswarm_setup
```

## Internetverbindung trennen
Ab hier muss WLAN eingeschaltet, der Roboter aber mit keinem Access Point verbunden sein. Gehe dazu mit der Maus in die Eistellungen, rufe die Einstellungen zu deinem momentanen WlLAN Access Point und trenne die Verbindung. Konfiguriere alle gemerkten Access Points so, dass sich der Roboter nicht automatisch mit ihnen verbinden kann.

## Variablen einrichten
1. Führe `ip link` in einer Konsole aus, um existierende Netzwerk-Interfaces aufzulisten. Je Interface werden zwei Zeilen nach folgendem Schema ausgegeben:
    ```
    1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN mode DEFAULT group default qlen 1000
        link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    ```
    Hier steht `lo` für den Namen des Interfaces. Suche nach dem Namen für ein Wlan-Interface. Diese fangen üblicherweise mit `wlan` oder `wl` an (üblich ist `wlan0`). Die Intel NUC Mini-PCs verwenden die Bezeichnung `wlp0s20f3`.
2. Navigiere in das Setup-Verzeichnis in einer Konsole mit `cd ~/solarswarm_setup/`.
3. Führe in dieser Konsole `bash service_helper.bash wlandev <Wlan-Interface>` aus (ersetzte `<>` und Ihhalt mit dem genauen Name des Wlan-Interfaces). Das Skript `service_helper.bash` gibt zu Beginn immer eine Warnung aus und erfordert eine Bestätigung durch eine Eingabe von "y". Beim ersten Aufruf und nach längerer Inaktivität in der Konsole wird das Passwort des Nutzers erfordert. 
4. Führe in dieser Konsole `bash service_helper.bash ssh <Nutzername>` aus. Bestätige erneut, wenn davor gewarnt wird, dass Schlüssel überschrieben werden könnten.

## Docker Node Labels
Docker Swarm Services sollen später nur auf bestimmten Rechnern laufen, um leistungsschwächere Roboter nicht zu überlasten, systemspezifische Eingenschaften wie CPU-Architekturen zu berücksichtigen und Daten auf dem Roboter vorauszusetzen. Beispielsweise ist es nur sinnvoll für einen Roboter, einen Service zum Ausliefern von Kartendaten zu starten, wenn er eine lokale Kopie dieser Daten besitzt.

1. Gib in der Konsole mit `cat ~/solarswarm_setup/docker/labels_usage.md` eine Liste mit laut lokalem Stand verfügbaren Labels aus.
2. Gib in der Konsole mit `sudo docker info` Informationen zum Roboter aus. In den unteren Zeilen werden Architektur, die Anzahl der CPU-Kerne und der maximale Arbeitsspeicher ausgegeben.
3. Erstelle eine Datei mit deinem Nutzernamen plus der Dateiendung `.labels` als Dateiname. Führe dazu `touch ~/solarswarm_setup/docker/<Nutzername>.labels` in der Konsole aus. Öffne die Datei mit einem Texteditor, indem du entweder (a) mit der Maus im Dateiexplorer von deinem *persönlichen Bereich* nach `solarswarm_setup`, dann nach `docker` gehst und mit Rechtsclick die Datei mit einem Programm deiner Wahl öffnest (für Ubuntu 24.04 genügt ein Doppelclick für den Standard-Texteditor) oder (b) mit `nano ~/solarswarm_setup/docker/<Nutzername>.labels` (ersetze "#" mit deinem Nutzernamen).
4. Schreibe zutreffende Labels in die Datei und trenne sie entweder mit einem Leerzeichen oder einem Zeilenumbruch (empfohlen). Wird die Datei leer gelassen, kann der Roboter später manche Services nicht replizieren. Um unsichtbare Zeichenfehler zu vermeiden, baue zum Schluss einen zusätzlichen Zeilenumbruch ein. Beispielhafter Inhalt einer Labels-Datei:
    ```
    architecture=x86_64
    cpus=8
    ram_ge4
    ram_ge8
    ram_ge16
    has_gpu

    ```
    > Hinweis: Hat ein Roboter beispielsweise 16 GB RAM, sollte er neben dem Label `ram_ge16` auch die Labels `ram_ge8` und `ram_ge4` erhalten.
5. Speichere die Datei entweder (a) mit der Tastenkombination `Strg + S` oder (b) mit der Tastenkombination `Strg + X`, gefolgt von `Y`, gefolgt von `Enter`.

## Systemd Services und service_helper.bash
Führe folgende Befehle in der Konsole aus, um notwendige Systemd Services einzurichten und ihren Status zu überprüfen:
```bash
bash service_helper.bash setup
bash service_helper.bash status
```
Lasse den letzten Parameter aus, wenn die restlichen Funktionen des Skripts `service_helper.bash` erfahren möchtest. Nach dem Setup sind fast alle Services "enabled", aber nicht gestartet. Somit tun sie nichts erst nichts bis sie gestartet oder der Roboter neugestartet wird.

## SSH einrichten und Schlüssel austauschen neu.
Für den Austausch von Datein müssen der SSH-Client konfiguriert und SSH-Schlüssel zwischen Robotern ausgetauscht werden:
1. Öffne in der Konsole mit `sudo nano /etc/system/ssh/sshd_config` die globale Konfigurationsdatei für SSH. Suche mit den Pfeiltasten nach einer Zeile mit `PubkeyAuthentication`. Falls vorhanden, entferne das "#" und ersetze "no" mit "yes". Speichere anschließend mit der Tastenkombination `Strg + X`, gefolgt von `Y`, gefolgt von `Enter`.
2. Starte SSH mit `sudo systemctl restart ssh.service` neu.
3. Stelle auf diesem und anderen Robotern, die ihre Schlüssel austauschen sollen, mit `bash service_helper.bash status` und `bash service_helper.bash restart <Service>` sicher, dass zumindest die Services `batman_adv_setup` und `rx_copy` aktiv sind.
4. Schicke mit `bash service_helper.bash send keys` alle lokal vorhandenen öffentlichen SSH-Schlüssel im Setup-Verzeichnis an alle im Mesh **erreichbaren** Roboter. Falls ein Roboter den eigenen Schlüssel noch nicht besitzt, wird das Passwort des empfangenden Roboters abgefragt. Wenn du beispielsweise auf `zulu` bist und das Skript deinen öffentlichen Schlüssel an `alfa` schicken möchte, musst du das Passwort vom Nutzer `alfa` auf dem Ziel-Roboter (hier **solarswarm-a**) angeben.

## Bestimmen eines Leaders
Der Leader (Anführer) in einem Docker Swarm ist derjenige Manager, der den Schwarm initiiert hat. Er stellt Join-Tokens aus, mit denen andere Hosts als Worker- oder Manager-Nodes beitreten können. Für unseren Schwarm übernimmt der Leader zusätzliche Aufgauben. Nur der Leader darf den Systemd Service *docker_leader* ausführen.

Ob ein Roboter Leader sein soll, muss er bereits vor dem Start von *docker_init* wissen. Dazu benötigt er im Verzeichnis `docker/` eine Datei `leader` mit seinem eigenen Namen. **Falls der einzurichtende Roboter** Leader werden soll, führe den Befehl `echo $MESH_IDENTITY > ~/solarswarm_setup/docker/leader` auf diesem Roboter aus. Falls zuvor ein anderer Roboter Leader war, entferne **auf diesem Roboter** mit `rm ~/solarswarm_setup/docker/leader` die Leader-Datei.

Ein Beispielszenario: Host `alfa` erhält eine Datei `leader`, die nur den Namen `alfa` beinhaltet. Vor einem weiteren Start des Schwarms soll jedoch `zulu` der neue Leader werden. Dazu wird die Leader-Datei auf `alfa` gelöscht und eine neue Leader-Datei mit dem Inhalt `zulu` auf `zulu` erstellt. `zulu` bleibt Leader, bis die Datei auf `zulu` wieder gelöscht wird.