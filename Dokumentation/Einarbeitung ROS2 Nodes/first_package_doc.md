# Einleitung
In diesem Abschnitt soll anhand eines einfachen Beispiels gezeigt werden, wie ROS2 Packages erzeugt werden. Anschließend soll in Tests die Kommunikation zwischen Nodes getestet werden.

Voraussetzungen:
    - Linux (hier Ubuntu 24.04)
    - ros2 jazzy Installation
    - colcon
    - rosdep

Die Tests wurden auf einem privatem Laptop in einer VM und einem NUC durchgeführt. Für den NUC wurde das Docker Image `ros:jazzy` verwendet, welches alle Voraussetzungen erfüllt. Bei Verwendungs dieses Images müssen `colcon` und `rosdep` eventuell aktualisiert werden.

# ROS2 sourcen
Bei jedem Öffnen der Konsole muss `source /opt/ros/jazzy/setup.bash` ausgeführt werden.
Alternativ dies mit `echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc` automatisiert werden.
Da hier die globale ROS-Installation gesourct wird, nennt man sie das Underlay. Um Abhängigkeiten von Packages besser zu verwalten und Konflikte zu vermeiden, verwendet man üblicherweise ein Overlay. Dies ist eine paketspezifische Installation mit Ergänzungen um vorausgesetzte Bibliotheken.

Referenz: https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html

# Workspace einrichten
Dieses Package ist ein einfaches Beispiel zu ersten Testzwecken. Ziel ist es, ein einfaches Publisher-Subscriber-Modell umzusetzen und unter verschiedenen Bedienungen zu testen:
- Auf der selben Maschine
- In zwei Containern auf der selben Maschine
- In zwei Containern auf jeweils verschiedenen Maschinen

Das Verzeichnis `~/solarswarm` soll als Workspace dienen und Packages in einem Verzeichnis `src` liegen. In `src` erzeugt `ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name simple_pub first_package` ein Package namens `first_package`, welches Python Code für seine Nodes verwenden soll. Stattdessen können Nodes auch in C implementiert und mit CMake gebaut werden. Die Tutorials der offiziellen Dokumentation bietet ein side-by-side für beide Varianten. Der Einfachheit wegen verwenden wir Python. `--node-name simple_pub` erzeugt bereits eine Node, die als Referenz hilft, das Package um weitere Nodes zu erweitern. `pkg create` stellt alle fürs Bauen des Packages notwendigen Dateien. Diese müssen jedoch angepasst und um funktionierende Python Skripte ergänzt werden. In der `package.xml` können bereits `<description>` und `<maintainer>` und in `setup.py` `maintainer`, `maintainer_email` und `description` gefüllt werden.

`colcon` und `rosdep` sind zwei grundlegende Tools, um ein Package zu bauen und Abhängigkeiten zu installieren.
Von `~/solarswarm` aus kann mit `colcon build --packages-select first_package` das Package gebaut werden, doch macht es noch nichts. `colcon build` würde genügen, allerdings alle vorhandenen Packages bauen und eventuell länger dauern. Gebaut werden sollte nach jeder Änderung des Codes vor Tests.
`rosdep install -i --from-path src --rosdistro jazzy -y` installiert Abhängigkeiten aller Packages im Workspace. Diese müssen im Package mit ihrer eindeutigen Bezeichnung selbst angegeben werden. Abhängigkeiten können eingetragen werden, nachdem sie bei der Programmierung der Nodes festgestellt wurden.

Zur Ausführung sollte das Overlay mit `source install/local_setup.bash` gesourct werden.

Referenz: https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html

# Publisher Node erstellen
`~/solarswarm/src/first_package/first_package` beinhaltet Python-Skripte für Nodes. `simple_pub.py` ist bereits vorhanden und beinhaltet nur eine einfache `print()`-Anweisung, die nun mit dem Publisher ersetzt wird. Anschließend wird in einer neuen Datei `simple_sub.py` der Subscriber umgesetzt.

Für einen Publisher wird eine Klasse benötigt, die von der Klasse Node aus dem Modul `rcl.node` erbt. Des Weiteren wird `rclpy` zur Ausführung und ein Nachrichtentyp benötigt. In `simple_pub.py` wurde eine möglichst einfache Implementierung umgesetzt. `simple_sub.py` beinhaltet eine einfache Implementierung eines Subscribers.

Anschließend müssen Abhängigkeiten der Python Module und Entrypoints angegeben werden.  
In `package.xml`:  
    `<exec_depend>rclpy</exec_depend>`
    und `<exec_depend>std_msgs</exec_depend>`
    bei den anderen `_depend` Tags

In `setup.py`:  
    Das Directory `entry_points` hat den Schlüssel `console_scripts`, zu dem eine Liste an Strings gehört. `'simple_pub = first_package.simple_pub:main'` ist bereits in der Liste, aber (mit Komma getrennt) muss `'simple_sub = first_package.simple_sub:main'` noch hinzugefügt werden.

Von `~/solarswarn` aus installiert `rosdep install -i --from-path src --rosdistro jazzy -y` die notwendigen Python Module. `colcon build --packages-select first_package` baut das Package.

Referenz: https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

# Erster Test: Auf der selben Maschine
Zum Ausführen beider Nodes werden zwei Terminals benötigt, die jeweils das Overlay sourcen. Dazu dient `source install/local_setup.bash` bzw. `source ~/solarswarm/install/local_setup.bash`, wenn man sich nicht zuvor ins Workspace begibt.

Die Nodes werden mit `ros2 run <Name des Packages> <Name der Node>` gestarten. Hier:
- `ros2 run first_package simple_pub` in dem ersten Terminal
- `ros2 run first_package simple_sub` in dem zweiten Terminal

Der Test ist erfolgreich, wenn jede Sekunde das erste Terminal `Status gesendet.` und das zweite `Publisher still running...` ausgeben.

# Zweiter Test: In zwei Containern auf der selben Maschine
Als Image für die Docker-Container verwenden wir `ros:jazzy`. Mit `sudo docker pull ros:jazzy` kann es installiert werden. `sudo docker run -it ros:jazzy` startet einen Container mit interaktiver TTY. Das Image hat bereits eine ROS2 Distribution und `which colcon` bzw. `which rosdep` geben an, dass `colcon` und `rosdep` vorinstalliert sind. Der Nutzer heißt hier `ubuntu`. Bei Verwendung anderer Images sollte geprüft werden, ob diese Tools vorhanden sind.

## Einfache Dockerfiles ohne Netzwerk
Für den Test sollen zwei Dockerfiles `simple_pub_dockerfile` und `simple_sub_dockerfile` verwendet werden. Diese Dateien und die zugehörigen Skripte müssen im gleichen Verzeichnis wie `first_package` liegen. `container_test.zip` beinhaltet alle notwendigen Dateien, inklusive einer Kopie von `first_package`. Die Container sollen auf `ros:jazzy` basieren und eine Kopie von `first_package` haben, welches mit `rosdep` und `colcon` gebaut wird. Hier ist der Aufbau von `simple_pub_dockerfile` (`simple_sub_dockerfile` sieht fast gleich aus):
```
    FROM ros:jazzy
    ADD first_package /home/ubuntu/solarswarm/src/first_package
    ADD pub_init.bash /home/ubuntu/solarswarm
    WORKDIR /home/ubuntu/solarswarm
    SHELL ["/bin/bash", "-c"]
    RUN ["rosdep", "update"]
    ENTRYPOINT ["bash", "pub_init.bash"]
```

`pub_init.bash` sieht so aus:
```
    source /opt/ros/jazzy/setup.bash
    #rosdep update; #beim Bauen sinnvoller wegen Ausführdauer und Internetzugang
    rosdep install -i --from-path src --rosdistro jazzy -y
    colcon build --packages-select first_package
    source install/setup.bash
    ros2 run first_package simple_pub
```

Die Notation mit eckigen Klammern wird von Docker empfohlen. Statt `ENTRYPOINT ["bash", "pub_init.bash"]` wäre auch `ENTRYPOINT bash pub_init.bash` erlaubt, würde aber eine Warnung beim Bauen ausgeben.

`SHELL ["/bin/bash", "-c"]` ist notwendig, damit der `source` Befehl funktioniert und `ros2` funktionieren kann.
Lösung von Bruno Bronosky als Verbesserung zu Anubhav Sinhas Antwort. Referenz: https://stackoverflow.com/a/25423366

`first_package` soll im Container in `/home/ubuntu/solarswarm/src` liegen, damit es aus dem Verzeichnis `solarswarm` aus gebaut werden kann. Die Container werden in zwei unterschiedlichen Terminals gebaut mit
    `sudo docker build --tag pub -f simple_pub_dockerfile .` bzw.
    `sudo docker build --tag sub -f simple_sub_dockerfile .`
und gestartet mit
    `sudo docker run pub` bzw.
    `sudo docker run sub`

Ohne das Setzen von `SHELL` in der Dockerfile gelang es zuvor nicht, über `ENTRYPOINT` oder `CMD` das Skript vollständig auszuführen, da `source` nicht ausgeführt wurde und `ros2` der Shell unbekannt blieb. Die Funktionalität des Bash-Skripts befand sich zuvor in der Dockerfile, wurde aber verlagert, da es nur gelang, mit `sudo docker run -it pub` und `sudo docker run -it sub` innerhalb der laufenden Container das Skript auszuführen. Eine mögliche Erklärung dafür ist, dass unter Ubuntu `-i` standardmäßig `bash` verwendet wird, `CMD` und `ENTRYPOINT` von Docker hingegen `sh`. Mit `sudo docker run -it ros:jazzy` und anschließend `cat /etc/os-release` ließ sich prüfen, dass das Image `ros` zur Zeit des Tests (Mai 2025) auf Ubuntu 24.04.2 LTS basiert.

`pub` muss zuerst gestartet werden, da die Node `simple_sub` vom Publisher abhängt. Mit Docker Compose ließe sich die Abhängigkeit notieren, dass beispielsweise hier `sub` immer danach gestartet wird. Im Verlauf unseres Projekts werden wir uns darauf stützen, doch bei diesem kleinen Test reicht es, die Container manuell zu starten.
Sobald der Container `pub` im ersten Terminal läuft, sollte nach kurzer Zeit kontinuierlich die Ausgabe `Status Publiziert.` erscheinen. Anschließend kann im zweiten Container im anderen Terminal der Subscriber gestartet werden. Die wiederholte Ausgabe `Publisher still running...` deutet an, dass sich die Nodes gefunden haben, was bei uns auch ohne durch Docker definiertes Netzwerk der Fall war.

## Docker-Compose mit Netzwerk
Docker bietet verschiedene Netzwerktreiber. Am leichtesten lassen sich Container als Services in einer Docker-Compose.yaml einem Netzwerk zuordnen. Für den Test auf einem Host verwenden wir den Standard Netzwerktreiber `bridge`. Für den Test über verschiedene Hosts werden wir den `overlay`-Treiber verwenden, da dieser Docker daemons miteinander verbindet. 
Quelle: https://docs.docker.com/engine/network/

Für die Docker-Compose.yaml lassen sich einfach die Dockerfiles vom vorherigen Versuch verwenden. Sie sieht wie folgt aus:
```
    services:
            pub:
                build: ./simple_pub_dockerfile
                networks:
                    - my_network
                # tty: true
                # stdin_open: true
            sub:
                build: ./simple_sub_dockerfile
                networks:
                    - my_network
                # tty: true
                # stdin_open: true
        networks:
            my_network:
                driver: bridge
```
Beide Container terminieren, sobald sie laufen. Mit mit `tty` und `stdin_open` können sie zwar offen gehalten werden, geben aber keine sichtbare Ausgabe.
Um zu prüfen, ob die Kommunikation zwischen den Knoten gelingt, wurde manuell ein Netzwerk mit `sudo docker network create -d bridge my_network` erstellt und beide Container wie zuvor gestartet.
`sudo docker run --network=my_network pub` und `sudo docker run --network=my_network sub` in verschiedenen Terminals startet die Container in dem Netzwerk. Nach kurzer Zeit sollten wieder beide ROS2 Nodes miteinander komunizieren - erkennbar an den Ausgaben. Das Ergebnis gleicht dem vorherigen. Docker-Netzwerke scheinen für diesen Test auf einem Gerät überflüssig. 

# Dritter Test: In zwei Containern auf verschiedenen Maschinen (nicht durchgeführt)
> Overlay networks are often used to create a connection between Swarm services, but you can also use it to connect standalone containers running on different hosts. When using standalone containers, it's still required that you use Swarm mode to establish a connection between the hosts.
Referenz: https://docs.docker.com/engine/network/drivers/overlay/

Ähnlich wie beim letzten Versuch soll als nächstes ein Netzwerk über zwei Maschinen manuell mit `sudo docker network create -d overlay my_network` erstellt werden. Eine Maschine führt den Publisher-Knoten mit `sudo docker run --network=my_network -it pub` und die andere den Subscriber-Knoten mit `sudo docker run --network=my_network -it sub` aus.