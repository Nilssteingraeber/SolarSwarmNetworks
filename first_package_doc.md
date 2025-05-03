# Voraussetzungen
Linux (hier Ubuntu 24.04)
ros2 jazzy Installation
colcon
rosdep


# ROS2 sourcen
Bei jedem Öffnen der Konsole muss `source /opt/ros/jazzy/setup.bash` ausgeführt werden.
Alternativ dies mit `echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc` automatisiert werden.
Da hier die globale ROS-Installation gesourct wird, nennt man sie das Underlay. Um Abhängigkeiten von Packages besser zu verwalten und Konflikte zu vermeiden, verwendet man üblicherweise ein Overlay. Dies ist eine paketspezifische Installation mit Ergänzungen um vorausgesetzte Bibliotheken.

Quelle: https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html

# Workspace einrichten
Dieses Package ist ein einfaches Beispiel zu ersten Testzwecken. Ziel ist es, ein einfaches Publisher-Subscriber-Modell umzusetzen und unter verschiedenen Bedienungen zu testen:
- Auf der selben Maschine
- In zwei Containern auf der selben Maschine
- In zwei Containern auf jeweils verschiedenen Maschinen

Das Verzeichnis `~/solarswarm` soll als workspace dienen und Packages in einem Verzeichnis `src` liegen. In `src` erzeugt `ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name simple_pub first_package` ein Package namens `first_package`, welches Python Code für seine Nodes verwenden soll. `--node-name simple_pub` erzeugt bereits eine Node, die als Referenz hilft, das Package um weitere Nodes zu erweitern. `pkg create` stellt alle fürs Bauen des Packages notwendigen Dateien. Diese müssen jedoch angepasst und um funktionierende Python Skripte ergänzt werden. In der `package.xml` können bereits `<description>` und `<maintainer>` und in `setup.py` `maintainer`, `maintainer_email` und `description` gefüllt werden.

`colcon` und `rosdep` sind zwei grundlegende Tools, um ein Package zu bauen und Abhängigkeiten zu installieren.
Von `~/solarswarm` aus kann mit `colcon build --packages-select first_package` das Package gebaut werden, doch macht es noch nichts. `colcon build` würde genügen, allerdings alle vorhandenen Packages bauen und eventuell länger dauern. Gebaut werden sollte nach jeder Änderung des Codes vor Tests.
`rosdep install -i --from-path src --rosdistro jazzy -y` installiert Abhängigkeiten aller Packages im workspace. Diese müssen im Package mit ihrer eindeutigen Bezeichnung selbst angegeben werden. Abhängigkeiten können eingetragen werden, nachdem sie bei der Programmierung der Nodes festgestellt wurden. 

Zur Ausführung sollte das Overlay mit `source install/local_setup.bash` gesourct werden.

Quelle: https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html

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

Quelle: https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

# Erster Test: Auf der selben Maschine
Zum Ausführen beider Nodes werden zwei Terminals benötigt, die jeweils das Overlay sourcen. Dazu dient `source install/local_setup.bash` bzw. `source ~/solarswarm/install/local_setup.bash`, wenn man sich nicht zuvor ins Workspace begibt.

Die Nodes werden mit `ros2 run [Name des Packages] [Name der Node]` gestarten. Hier:
- `ros2 run first_package simple_pub` in dem ersten Terminal
- `ros2 run first_package simple_sub` in dem zweiten Terminal

Der Test ist erfolgreich, wenn jede Sekunde das erste Terminal `Status gesendet.` und das zweite `Publisher still running...` ausgeben.

# Zweiter Test: In zwei Containern auf der selben Maschine
# Dritter Test: In zwei Containern auf jeweils verschiedenen Maschinen