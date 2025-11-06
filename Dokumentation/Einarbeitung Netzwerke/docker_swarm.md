# Einarbeitung in Docker Swarm
## Konzept
Um später zu zeigen, dass unser Netzwerk funktioniert, wollen wir mehrere Knoten zusammenarbeiten lassen und Aufgaben verteilen. Bei Ausfällen einzelner Knoten soll gewährleistet sein, dass wichtige Aufgaben von anderen Knoten übernommen werden. Docker bietet dazu den Swarm Modus an, um Cluster Management und Features zum Orchestrieren von Services zu realisieren. Ein Schwarm besteht aus mehreren Docker hosts im Swarm Modus, welche manager, worker oder beide sein können. Manager verwalten Knoten im Schwarm und erteilen Aufgaben. Worker führen diese in Form von `swarm services` aus. `Services` oder Dienste bestehen aus einzelnen `Tasks` oder Aufgaben. In einem Schwarm wird ein optimaler Zustand definiert, den Docker versucht einzuhalten. Für uns relevant ist zuerst nur, dass der Schwarm beim Ausfall eines Knoten seine Tasks auf andere Knoten verteilt, damit die davon abhängigen Services weiter funktionieren.  
Referenz: https://docs.docker.com/engine/swarm/key-concepts/

## Docker init und join
Ein Schwarm wird mit `docker swarm init` gestartet. `init` gibt eine Bezeichnung des erstellten Knoten und zwei Join-Tokens aus. Diese Tokens werden anschließend benötigt, um weitere Worker oder Manager hinzuzufügen. Mit `docker swarm leave` kann der Schwarm verlassen werden, wobei der letzte Knoten `--force` oder `-f` verwenden muss, da der Docker Daemon es sonst verhindert. Die offizielle Docker Dokumentation bietet eine Übersicht der Optionen, sowie ihrer Standardwerte. Hier ist eine Auswahl von Optionen, die für uns zum Testen relevant sein könnten:
- `--listen-addr <IP-Adresse[:Port]>` spezifiziert eine IP-Adresse (und Port), auf der Worker Nachrichten von Managern lauschen. Statt einer IP-Adresse kann ein Netzwerkinterface angegeben werden.
- `--advertise-addr <IP-Adresse[:Port]>` spezifiziert eine IP-Adresse (und Port), mit der Manager werben. Statt einer IP-Adresse kann ein Netzwerk Interface angegeben werden.
- `--max-snapshots <Anzahl>` ist standardmäßig 0. `Anzahl` gibt an, wie viele alte Raft snapshots behalten werden. Sie halten den Zustand eines Clusters hält und sind zum Debuggen oder Wiederherstellen eines Clusters nützlich.
- `--snapshot-interval <Integer>` ist standardmäßig 10000. `Interger` gibt an, nach wie vielen Raft-Logs ein neuer Raft snapshot gemacht wird. 
- `--availability <active/pause/drain>` legt die Verfügbarkeit nach dem Beitritt fest. Manager mit `drain` sind nicht gleichzeitg Worker.

Nach der Initialisierung gibt `docker info` Informationen zum Knoten aus. Hiermit kann die Konfiguration überprüft werden.

Ein initialisierter Knoten kann einem anderen beitreten mit `docker swarm join --token <Token> <IP-Adresse[:Port]>`. Je nach Token tritt der Knoten als Manager oder Worker bei. Alternativ kann man einen Token-Manager mit `swarm join-token` verwenden: `docker swarm join-token worker` oder `docker swarm join-token worker` gibt den notwendigen Kommandozeilenbefehl zum Beitreten mit dem entsprechenden Token aus. `join-token` gibt mit `-q` (`--quiet`) nur das entsprechende Token aus oder generiert ein neues mit `--rotate`.

Referenz: https://docs.docker.com/reference/cli/docker/swarm/init/

Mehr zu Raft snapshots: https://docs.docker.com/engine/swarm/raft/

Manche Optionen von `docker init` können ebenfalls für `docker join` verwendet werden. Mit `docker node ls` werden tabellarisch Mitglieder mit ID, HOSTNAME, STATUS, AVAILABILITY und MANAGER STATUS ausgegeben. Ein `*` hebt die eigene ID hervor und der MANAGER STATUS unterscheidet zwischen `Leader`, `Reachable` und leer für Worker. Der HOSTNAME eines Knoten lautet `worker#` oder `manager#`, wobei `#` eine steigende Ganzzahl ist.
Referenz: https://docs.docker.com/reference/cli/docker/swarm/join/

## Docker services
Services sind praktisch eine Gruppe von Containern, die in einem Schwarm gestartet und von ihm verwaltet werden. Um mit Services arbeiten zu können, muss ein Knoten im Swarm Modus sein. Ein Service kann in seiner einfachsten Form mit `docker service create <Image>` erstellt werden, wonach seine ID ausgegeben wird. Mit `docker service ls` können aktive Services angezeigt, mit `docker service remove <Service>` entfernt und mit `docker service update <Optionen> <Service>` seine Parameter aktualisiert werden. Die offizielle Dokumentation zu `service create` führt relevante Optionen auf. Einige wie `--network` gleichen oder ähneln den von `docker run`. Hier ist eine Auswahl von Optionen, die für uns zum Testen relevant sein könnten:
- `--name <Name>` gibt einem Service einen Namen, der in `service ls` sichtbar ist.
- `--replicas <Anzahl>` ist standardmäßig 1. Bei größeren Zahlen versucht ein Schwarm stets, die angegebene Anzahl an Repikas zu erhalten. `service ls` zeigt an, wie viele davon laufen, bspw. `1/1`, `5/5` oder `4/5`, falls eins ausgefallen ist.
- `--replicas-max-per-node <Anzahl>` ist standardmäßig 0 (unbegrenzt). Gibt die maximale `Anzahl` an Replikas des Services pro Knoten an.
- `--mode <replicated/global>` ist standardmäßig `replicated`. `global` legt fest, dass ein Service auf jedem aktiven Knoten im Schwarm laufen soll.
- `--mode <replicated-job/global-job>` startet Service als `job`. Diese Art von Service läuft nur bis zum Ende einer Operation. Replizierte Jobs laufen gleichzeitig, können aber mit `--max-concurrent <Anzahl>` beschränkt werden.
- `--restart-max-attempts <Anzahl>` gibt an, wie oft versucht wird, einen Service neuzustarten.
- `--secret [source=]<Quelle>[,target=<Ziel>]` legt ein Geheimnis im Container unter `/run/secrets` oder `/run/secrets/<Ziel>` ab. Auch Besitzer, Gruppe und Zugriffsrechte können definiert werden. Für mehrere Geheimnisse kann die Option mehrmals verwendet werden. Eignet sich für Schlüssel.
- `--config [source=]<Quelle>[,target=<Ziel>]` mountet config-Datei in den Container. Auch Besitzer, Gruppe und Zugriffsrechte können definiert werden. Eignet sich für Datenbanken.
- `--env <Variable>=<Wert>` legt Umgebungsvariablen an. Für mehrere Variablen kann die Option mehrmals verwendet werden.
- `--constraint <Bedingung>` beschränkt, welche Knoten einen Task ausführen können. Bedingungen werden mit `==` und `!=` formuliert, bspw. `--constraint node.role==manager`. Für mehrere Bedingungen kann die Option mehrmals verwendet werden, wodurch sie mit einem logischen `AND` verknüpft werden.
- `--network` verhält sich wie bei `docker run`. Hier ist es nur sinnvoll, `overlay` als Treiber zu verwenden.
- `--mount` verhält sich wie bei `docker run`. Aufgrund des Umfangs lohnt es sich eher, die offizielle Dokumentation heranzuziehen.

Ein Beispiel mit `--mount`:
```bash
    docker service create \
    --name my-service \
    --replicas 3 \
    --mount type=volume,source=my-volume,destination=/path/in/container,volume-label="color=red",volume-label="shape=round" \
    nginx:alpine
```
Referenz: https://docs.docker.com/reference/cli/docker/service/create/

Mit `docker service ps <Service>` werden alle Tasks und die ausführenden Knoten tabellarisch dargestellt.
https://docs.docker.com/reference/cli/docker/service/ps/

Bei `docker service update` wird an die Optionen `--mount`, `--secret`, `--network` entweder `-add` oder `-rm` angehängt. Beispiel: `--mount-add`
https://docs.docker.com/reference/cli/docker/service/update/

Services lassen sich mit `docker service inspect <ID/NAME>` inspizieren. Die Ausgabe ist im JSON-Format, kann aber mit `--pretty` oder `--format pretty` lesbarer gemacht werden.
Referenz: https://docs.docker.com/reference/cli/docker/service/inspect/

Nachdem ein Service im Modus `replicated` erstellt wurde, kann die Anzahl der zu erzielenden Replikas mit `docker service scale <Service>=<Anzahl> [weitere Paare]` oder `docker service update --replicas=<Anzahl> <Service>` geändert werden.
Referenz: https://docs.docker.com/reference/cli/docker/service/scale/

Die einzelnen Tasks eines Services lassen sich mit `docker service ps <Service>` tabellarisch anzeigen. Die Ausgabe lässt sich zusätzlich mit `--filter` und `--format` bearbeiten.
Zum Formatieren und weitere Optionen: https://docs.docker.com/reference/cli/docker/service/scale/

# Einfacher Test (nicht durchgeführt)
Allgemeiner Ablauf:
- Drei Knoten werden mit `docker swarm init` initialisiert
- Auf einem Knoten wird mit `docker swarm join-token worker` der Befehl ausgegeben, mit dem andere Knoten dem Schwarm beitreten können
- Spezifischer Test
- Nach und nach Knoten entfernen mit `docker swarm leave`; mit `--force` beim letzten

Verwendetes Image:
- ros:jazzy
- `docker service create --name jazzy ros:jazzy`

Erster Test:
- Manager initialisiert mit `--availability drain`
- Ein Service gestartet mit `--replicas 1`
- Mit `docker service <Service>` prüfen, welcher Knoten den Service ausführt
- Ausführenden Knoten mit `docker node update --availability pause <Knoten>` stoppen
- Mit `docker service ps <Service>` erneut prüfen
- Mit `docker service scale <Service>=3` Replikationen erhöhen, dann `docker service ps <Service>`
- Falls nicht verteilt, Last mit `docker service update --replicas-max-per-node 1` verteilen

Zweiter Test:
- Nachdem alle Knoten als Worker beigetreten sind, verlässt der Manager mit `docker swarm leave --force` den Schwarm
- Mit `docker node ls` die Knoten inspizieren

Dritter Test:
- Ein Knoten tritt als Manager bei
- Der erste Manager verlässt den Schwarm
- Der neue Manager startet einen Service mit `--replicas 3 --replicas-max-per-node 1`
- In `docker service ls` sollten 2/3 aktiv sein
- Erste Manager versucht, mit seinem eigenen Token wieder beizutreten


# Docker Swarm Strategie
Für den Gebrauch von Docker Swarm brauchen wir eine Strategie, um allen Robotern den Beitritt zu ermöglichen und den Schwarm erweiterbar zu machen. Der Schwarm wird von einem Roboter gestartet, welcher dann als Leader bezeichnet wird. Durch `docker swarm init` werden Beitritts-Tokens generiert, mit denen weitere Roboter beitreten können. Eine Herausforderung besteht darin, diese Tokens allen Robotern verfügbar zu machen und vor dem Beitritt zu entscheiden, ob ein Roboter Worker oder Manager werden soll. Obwohl es möglich ist, alle Roboter als Manager zu integrieren, warnt der offizielle Guide zum Administrieren und Warten von Docker Swarms vor möglichen Leistungsengpässen. In einem kleinen, nicht-kritischen Schwarm sei das Risiko gering, wenn Ressourcen zum Administrieren begrenzt werden.

Referenz: https://docs.docker.com/engine/swarm/admin_guide/#run-manager-only-nodes

Sind keine Manager vorhanden, arbeiten laut Guide Worker weiter, können jedoch nicht mehr administriert werden. Folglich können Services nicht hinzugefügt, bearbeitet oder entfernt werden. Es sollten daher genug Manager möglichst verteilt existieren, sodass auch beim Spalten des Schwarms Manager erreicht werden können.

Referenz: https://docs.docker.com/engine/swarm/admin_guide/#recover-from-losing-the-quorum

## Verteilung der Join-Tokens
Initiiert und verlässt ein Roboter mehrmals einen Schwarm mit `sudo docker swarm init` und `sudo docker swarm leave -f`, wird jedes Mal ein anderes Join-Token generiert. Daher muss beim Start des Schwarms ein Roboter die Verantwortung dafür übernehmen, den Schwarm zu initiieren und die Token mit anderen Robotern auszutauschen. Da sich voraussichtlich in der Praxis alle Roboter beim Systemstart räumlich nahe sind, betrachten wir folgende Möglichkeiten, um die Tokens auszutauschen.

### SSH/SCP
Alle Roboter bekommen einen einfachen Namen wie "Anton", "Ida", oder "Otto" und eine statische IPv4-Adresse vergeben. Nach dem Setup von Batman-adv wird durch avahi-autoipd dynamisch eine IPv4-Adresse vergeben. Die statische Adresse ist notwendig, um zuverlässige Hosts für SSH anzugeben.
Auf allen Robotern wird SSH eingerichtet: Lokal auf jedem Roboter wird ein Schlüsselpaar bestehend aus öffentlichem und privatem Schlüssel in `~/.ssh/` generiert. Diese Paare sind gebunden an den lokalen Nutzer und müssen manuell auf jedem Roboter generiert werden, da nicht gewährleistet ist, dass alle Nutzer und Gerätenamen identisch oder bekannt sind. Die öffentlichen Schlüssel werden mit Namen und Adressen gesammelt, um zentral ein eine Konfigurations-Datei mit allen Hosts anzulegen. Diese wird anschlließend inklusive öffentlicher Schlüssel aller bekannten Hosts auf alle Roboter in `~/.ssh/` kopiert.
Der Leader des Schwarms hinterlegt generierte Tokens im Verzeichnis `~/swarm_tokens/` und kopiert sie über SCP ins selbe Verzeichnis auf allen erreichbaren Hosts. Der Service, der diese Aufgabe übernimmt, kann optional Logik erhalten und bestimmen, ob das Worker- oder Manager-Token kopiert wird.

Vorteilhaft an dieser Methode ist, dass SSH eingerichtet wird und später weiterverwendet werden kann. Dazu sind SSH und SCP sicher und können für spätere Projekte mit Sicherheitsanforderungen verwendet werden. Optional können Roboter, die nicht Leader sind, die Tokens weiter teilen. Zur Verwendung von statischen IP-Adressen rät auch Docker. Diese seien als `--advertise-addr` zuverlässiger und sicherer:
> "Because manager nodes are meant to be a stable component of the infrastructure, you should use a fixed IP address for the advertise address to prevent the swarm from becoming unstable on machine reboot.
> 
> If the whole swarm restarts and every manager node subsequently gets a new IP address, there is no way for any node to contact an existing manager. Therefore the swarm is hung while nodes try to contact one another at their old IP addresses.
> 
> Dynamic IP addresses are OK for worker nodes."

Nachteilhaft ist die Komplexität der Methode. Es müssen beim Systemstart alle Roboter erreichbar sein und über statische IPv4-Adressen verfügen. Es ist anfangs nicht möglich, über eine GUI die Erreichbarkeit der Roboter zu prüfen, da es ohne Schwarm noch keine Docker Swarm Services gibt. Für einen autonomen Start ist nicht klar, woher ein Roboter wissen soll, ob er auf Tokens warten oder Leader werden soll. Auch kann ein Roboter momentan nicht unterscheiden, ob er vor einem Einsatz gestartet wird oder während des EInsatzes neugestartet wurde. Es sind diesbezüglich weitere Überlegungen nötig. Zuletzt wird es mit zunehmender Anzahl an Robotern immer aufwendiger, neue Roboter zu integrieren, da die Hosts aller Roboter aktualisiert werden müssen. SCP kann Dateien überschreiben, jedoch erfordert es Root-Rechte, um `~/.ssh/config` zu bearbeiten (mit SCP nicht getestet). Weiterhin müssen alle Roboter erreichbar sein, um von der Änderung zu erfahren. Ansonsten müssen über Services regelmäßig Updates geteilt und auf der Zielmaschine mit dem eigenen Stand verglichen werden. Möglicherweise gelingt es, mit SCP eine temporäre Datei anzulegen, welche durch den Service auf der Zielmaschine die veraltete `~/.ssh/config` überschreibt.

### HTTP-Server
Ein Roboter oder PC im Netzwerk mit einer statischen IPv4-Adresse betreibt einen einfachen HTTP-Server, welcher Token-Dateien ausliefert. Der Host ist entweder immer der Leader und liefert seine eigenen Tokens aus oder kann Tokens von einem Leader erhalten. Über einen Service rufen die Roboter mit `curl` die Tokens ab.

Diesse Methode ist einfacher umzusetzen als SSH, da nur der Host des Servers konfiguriert werden muss. Es wird jedoch wieder eine statische IP-Adresse benötigt und neue Roboter müssen im selben Netzwerk sein wie der Server.

### USB-Stick
Roboter werden nacheinander mit einem beliebigen USB-Stick gestartet. Nach Systemstart prüft ein Service, ob auf dem USB-Stick ein Verzeichnis `swarm_tokens/` mit Dateien für die Tokens vorhanden sind. Falls nicht, wird der Roboter zum Leader und legt die von ihm generierten Tokens in dem Verzeichnis ab. Eine zusätzliche Datei könnte mit zwei Zahlen festhalten, wie viele Manager maximal im Netz sein sollen und wie oft das Manager-Token verwendet haben. Falls diese Dateien vorhanden sind, tritt der Roboter dem Schwarm mit dem Worker- oder Manager-Token bei und inkrementiert die eine Zahl in der zusätzlichen Datei.

Diese Methode ist erheblich leichter umzusetzen, da sie nur einen einfachen Service benötigt. Der USB-Stick kann beliebig oft verwendet werden, da keine Roboter-abhängigen Informationen wie Nutzername, IP-Adresse oder MAC-Adresse benötigt werden. Auch wird keine Erreichbarkeit zu einem besonderen Roboter vorausgesetzt. Wenn der Schwarm heruntergefahren wird, können die Dateien auf dem USB-Stick gelöscht werden, um beim nächsten Start einen neuen Leader zu ernennen und neue Tokens zu generieren.

## Replikation von Docker Swarm Services
In einem Docker Swarm werden Services explizit von einem Manager mit `docker service create` erstellt. Wie zuvor beschrieben, wird ein zu erreichender Zustand für den Schwarm definiert. Durch `create` versucht der Schwarm, die angegebene Anzahl an Replikas unter berücksichtigung von `--replicas` und `--replicas-max-per-node` einzuhalten. Docker Swarm berücksichtigt nicht, wie leistungsfähig einzelne Hosts sind. Wir verwenden daher Labels, welche beim Erstellen eines Services Constraints angegeben werden können. Eine Liste von verwendeten Labels:

CAN_BECOME_MANAGER=true
CAN_BECOME_LEADER=false
CPU_ARCHITECTURE=x86_64 # oder ARM64, System z, PowerPC64, RISC-V, x86
    uname --help
    lscpu --help
    ^ mit service_helper.bash anlegen
CPU_COUNT=8 # oder 4, 2, ...
    nproc
    lscpu
RAM_GE16=false # grob greater equal 16 GB, true oder false
RAM_GE8=true
RAM_GE4=true
HAS_2D_MAP_DATA=false # oder true
HAS_3D_MAP_DATA=false
HAS_DB_DATA=false

https://docs.docker.com/reference/cli/docker/service/create/#label
