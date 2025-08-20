# Einleitung
Dieses Verzeichnis beinhaltet alle notwendigen Dateien für den Start eines Mock-Roboters. Ein Mock-Roboter ist ein WLAN-fähiger Rechner mit Linux (Ubuntu) als Betriebssystem. Wegen der Anforderungen an dieses Projekt werden andere Distributionen oder ältere Versionen als Ubuntu 24.04 nicht getestet. Der Start setzt neben dem Betriebssystem eine Internetverbindung für die Vorbereitung voraus. 

# Vorbereitung
## Docker
Eine ordentliche Installation von Docker erfordert mehrere Schritte, welche sich nicht in einem Skript automatisieren lassen. Sollte Docker nicht installiert sein, orientiere Dich bitte am offiziellen Guide: https://docs.docker.com/engine/install/

Zur Ausführung von ROS2 Nodes wird das Image `ros:jazzy` verwendet: 
```bash
    sudo docker pull ros:jazzy # optional, da es sonst beim Bauen installiert wird
```

## Kernel-Module
Stelle sicher, dass die Kernel-Module `batman_adv` und `iwlwifi` verfügbar sind:
```bash
    find /lib/modules/$(uname -r) -type f -name '*.ko*'
```
Referenz: https://unix.stackexchange.com/a/184880

## Command Line Tools
Zur Ausführung von `batman_adv_setup.bash`:
```bash
    sudp apt update
    sudo apt install iw
    sudo apt install batctl
    sudo apt install avahi-autoipd
```

# Start
## bat0 einrichten
Wenn der Mock-Roboter WLAN-fähig ist, hat er ein dazugehöriges Netzwerk-Interface. Häufig wird es `wlan0` genannt, kann je nach Gerät aber auch anders heißen. Im Falle des NUCs ist es `wlp0s20f3`. Mit `ip link show` lassen sich alle verfügbaren Netzwerk-Interfaces.

Führe mit der richtigen bezeichnung das `baatman_adv_setup.bash` Skript aus:
```bash
    sudo batman_adv_setup.bash wlp0s20f3 # wlp0s20f3 ersetzen mit eigenem Interface
```

## Docker Compose
Von diesem Verzeichnis aus startet `sudo docker compose up --build` den Container mit dem Mock-Roboter. Die ROS2 Node `mock_data` beginnt nun, Nachrichten zu veröffentlichen.

Nach dem Start lässt sich der Container manuell inspizieren:
```bash
    # auf dem Host
    sudo docker ps # ID des zu inspizierenden Containers kopieren
    sudo docker exec -it <Container ID> bash # ID einfügen

    # im Container, falls mit `ros2` manuell Befehle ausgeführt werden sollen
    source /opt/ros/jazzy/setup.bash
    source install/local_setup.bash
```