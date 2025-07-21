# Mock-Roboter
## Übersicht
Das ROS2 Package `mock_robot` soll Ziel /Z70/ im Pflichtenheft erfüllen. Mock-Roboter sind ROS2 Nodes, die auf Nicht-Robotern gestartet werden können, um Roboterprozesse zu simulieren. Das Package soll eine einzige Node implementieren, die Mockdaten für Tests liefert, darunter: Batteriestand, CPU-Last (über Intervalle gemittelt), Position, Orientierung und Sonstiges (JSON-String).

Zum Erstellung des Packages wird sich an den Schritten aus dem Kapitel **first_package** orientiert. Für die verschiedenen Mockdaten werden Message Interfaces benötigt. Da die Daten auf verschiedenen Topics gepublisht werden sollen, genügen die vordefinierten Interfaces aus `example_interfaces` und `geometry_msgs`. Batteriestand und CPU-Last lassen sich als Float und Sonstiges als String darstellen. `example_interfaces` bietet für diese Datentypen die Interfaces String und Float64. Die offizielle Dokumentation des Packages empfiehlt, diese nur für Tests zu verwenden und eigene Interfaces fürs Deployment zu definieren. Für die Position und die Orientierung bietet das Package `geometry_msgs` das Message Interface Pose, welches aus einem Point und einem Quaternion (Vektor mit Skalar) besteht. Auf Wunsch der betreuenden Lehrperson wird Pose nicht verwendet. Für die Mock-Roboter werden zufällige Werte für die Position und Standardwerte für die Orientierung verwendet.

## Rekonstruktion
Das Package `mock_robot` wurde mit dem Befehl
```bash
    ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name mock_data mock_robot --dependencies rclpy rcl_interfaces example_interfaces geometry_msgs python3-psutil
```
erstellt. Es hat nach Erzeugung bereits `mock_data` und allen notwendigen Dateien. Hier angegebene Abhängigkeiten werden der `package.xml` automatisch hinzugefügt, jedoch müssen Angaben über Maintainer, Description und Version in `package.xml` und `setup.py` anktualisiert werden. Es sei zu beachten, dass `--dependencies` `<depend>`-Tags generiert, welche eine Abhängigkeit fürs Bauen und Ausführen beschreibt. Für Python-Module ist ersteres überflüssig, weswegen die Verwendung von `<exec_depend>`-Tags empfehlen wird. In `mock_robot/mock_robot/mock_data.py` wurde die Funktionalität der Node implementiert.

Zur Bestimmung der CPU-Last und Batterie wird das Python-Modul `psutil` verwendet, welches die Messung abstrahiert und betriebssystemunabhängig macht, solange das ausführende Betriebssystem unterstützt wird. Die Dokumentation des Moduls beinhaltet eine akutelle Liste aller unterstützten Betriebssysteme und Python Versionen. Unter "System related functions" sind die für die Node verwendeten Funktionen `getloadavg` und `sensors_battery` dokumentiert. Der Eintrag zu `getloadavg` beinhaltet ein Beispiel, wie die Funktion für nützliche Werte verwendet wird.

Referenz: https://psutil.readthedocs.io/en/latest/

Zur Bestimmung der IP-Adresse wird das Python-Modul `socket` verwendet, basierend auf einem geeksforgeeks-Beitrag
```python
    data["ip"] = gethostbyname(gethostname())
```
Referenz: https://www.geeksforgeeks.org/python/python-program-find-ip-address/

Zur Bestimmung der MAC-Adresse wird eine Regular Expression aus einem geeksforgeeks-Beitrag verwendet:
```python
    data["mac"] = ':'.join(re.findall('..', '%012x' % uuid.getnode()))
```
Referenz: https://www.geeksforgeeks.org/python/extracting-mac-address-using-python/

## Verwendung
`mock_robot.zip` sollte im ROS2 Workspace in `src` entpackt werden. Vom Workspace aus können die Abhängigkeiten mit `rosdep install -i --from-path src --rosdistro jazzy -y` installiert werden. `colcon build --packages-select mock-robot` baut das Package, wonach mit `source install/local_setup.bash` das Overlay gesourct werden sollte. Ist durch Modifikationen an diesem Package die Installation weiterer Packages oder Python-Module notwendig, müssen ihre entsprechenden rosdep Schlüssel der `package.xml` hinzugefügt werden.

Zur rosdep Namensauflösung: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html#how-do-i-know-what-keys-to-put-in-my-package-xml

Anschließend kann der Mock-Roboter mit `ros2 run mock_robot mock_data` manuell gestartet werden. Zur Laufzeit wird mit `ros2 node list` die Node als `mock_robot_status_pub` angezeigt. Sie verfügt über die drei Parameter `system_intervall` (default 3.0), `geo_intervall` (default 3.0) und `misc_intervall` (default 60.0), welche bestimmen, wie häufig Timer-Callback-Funktionen publishen. Da die Timer bei start der Node initialisiert werden, müssen die Parameter zur Launch-TIme festgelegt werden.

| Topic | Interface | Gruppe | Bedeutung |
|-------|-----------|--------|-----------|
| robot_battery | Float64 | system | Batteriestand data in % zwischen 0.0 und 100.0 oder gleich -1.0 (falls keine Informationen vorhanden sind) als 64-bit float |
| robot_cpu | Float 64 | system | Durchschnittliche CPU-Last data aller Kerne in der letzten Minute in % zwischen 0.0 und 100.0 als 64-bit float|
| robot_point | Point | geo | Drei (zufällige) Koordinaten x, y und z als 64-bit floats |
| robot_orientation | Quaterion | geo | Vier Werte x, y, z, w als 64-bit floats |
| robot_misc | String | misc | JSON-String data als string, welcher hier IP, MAC und datetime hat |

Die Nachrichten auf diesen Topics können mit `ros2 topic echo <Bezeichnung>` abgehört oder mit Subscribern, die mit entsprechenden Message Interfaces lauschen, verarbeitet werden.

### Message Interface
Definition geometry_msgs/msg/Point
```
    float64 x
    float64 y
    float64 z
```

Definition geometry_msgs/msg/Quaternion
```    
    float64 x 0
    float64 y 0
    float64 z 0
    float64 w 1
```

String und Float64 haben ein Feld `data` vom Typ `string` und `float64`. In der Node wird das Interface als Klasse importiert und eine Instanz davon erzeugt. Diese hat data oder x, y, z (und w) als Attribute. Mit `<Publisher>.publish(<Instanz>)` wird eine Nachricht veräffentlicht.

Gemetry_msgs Definitionen: https://docs.ros.org/en/jazzy/p/geometry_msgs/__message_definitions.html  
Example_interfaces Definitionen: https://docs.ros.org/en/jazzy/p/example_interfaces/__message_definitions.html