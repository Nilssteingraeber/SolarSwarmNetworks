# Einarbeitung in Docker Swarm
## Konzept
Um später zu zeigen, dass unser Netzwerk funktioniert, wollen wir mehrere Knoten zusammenarbeiten lassen und Aufgaben verteilen. Bei Ausfällen einzelner Knoten soll gewährleistet sein, dass wichtige Aufgaben von anderen Knoten übernommen werden. Docker bietet dazu den Swarm Modus an, um Cluster Management und Features zum Orchestrieren von Services zu realisieren. Ein Schwarm besteht aus mehreren Docker hosts im Swarm Modus, welche manager, worker oder beide sein können. Manager verwalten Knoten im Schwarm und erteilen Aufgaben. Worker führen diese in Form von `swarm services` aus. `Services` oder Dienste bestehen aus einzelnen `Tasks` oder Aufgaben. In einem Schwarm wird ein optimaler Zustand definiert, den Docker versucht einzuhalten. Für uns relevant ist zuerst nur, dass der Schwarm beim Ausfall eines Knoten seine Tasks auf andere Knoten verteilt, damit die davon abhängigen Services weiter funktionieren.  
Referenz: https://docs.docker.com/engine/swarm/key-concepts/

## Docker init und join
Ein Schwarm wird mit `docker swarm init` gestartet. `init` gibt eine Bezeichnung des erstellten Knoten und zwei Join-Tokens aus. Diese Tokens werden anschließend benötigt, um weitere Worker oder Manager hinzuzufügen. Mit `docker swarm leave` kann der Schwarm verlassen werden, wobei der letzte Knoten `--force` verwenden muss, da der Docker Daemon es sonst verhindert. Die offizielle Docker Dokumentation bietet eine Übersicht der Optionen, sowie ihrer Standardwerte. Hier ist eine Auswahl von Optionen, die für uns zum Testen relevant sein könnten:
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
Services sind praktisch eine Gruppe von Containern, die in einem Schwarm gestartet und von ihm verwaltet werden. Um mit Services arbeiten zu können, muss ein Knoten im Swarm Modus sein. Ein Service kann in seiner einfachsten Form mit `docker service create <Image>` erstellt werden, wonach seine ID ausgegeben wird. Mit `docker service ls` können aktive Services angezeigt, mit `docker service remove <Service>` entfernt und mit `docker service update <Optionen> <Service>` seine Parameter aktualisiert werden. Die offizielle Dokumentation zu `service create` führt eine Vielzahl an Optionen auf. Einige wie `--network` gleichen oder ähneln den von `docker run`. Hier ist eine Auswahl von Optionen, die für uns zum Testen relevant sein könnten:
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

# Einfacher Test
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

# Test mit ROS2 Service

Zwischenablage
    "Because manager nodes are meant to be a stable component of the infrastructure, you should use a fixed IP address for the advertise address to prevent the swarm from becoming unstable on machine reboot.

    If the whole swarm restarts and every manager node subsequently gets a new IP address, there is no way for any node to contact an existing manager. Therefore the swarm is hung while nodes try to contact one another at their old IP addresses.

    Dynamic IP addresses are OK for worker nodes."
Offizieller Guide zum Administrieren und Warten eines Docker Swarms: https://docs.docker.com/engine/swarm/admin_guide/