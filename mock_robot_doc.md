# Mock-Roboter
## Übersicht
Das ROS2 Package `mock_robot` soll Ziel /Z70/ im Pflichtenheft erfüllen. Mock-Roboter sind ROS2 Nodes, die auf Nicht-Robotern gestartet werden können, um Roboterprozesse zu simulieren. Das Package soll eine einzige Node implementieren, die Mockdaten für Tests liefert, darunter: Batteriestand, CPU-Last (über Intervalle gemittelt), Position, Orientierung und Sonstiges (JSON-String).

Zum Erstellung des Packages wird sich an den Schritten aus dem Kapitel **first_package** orientiert. Für die verschiedenen Mockdaten werden Message Interfaces benötigt. Da die Daten auf verschiedenen Topics gepublisht werden sollen, genügen die vordefinierten Interfaces aus `example_interfaces` und `geometry_msgs`. Diese werden ab Version 0.7.0 durch `custom_interfaces` Interfaces ersetzt, welche zusätzlich das Feld `nid` als String zur einfachen Identifizierung des Publishers haben. Es verwendet zuerst die MAC, kann aber noch ersetzt werden. Batteriestand und CPU-Last lassen sich als Float und Sonstiges als String darstellen. `example_interfaces` bietet für diese Datentypen die Interfaces *String* und *Float64*. Die offizielle Dokumentation des Packages empfiehlt, diese nur für Tests zu verwenden und eigene Interfaces fürs Deployment zu definieren. Daher implementiert `custom_interfaces` *RobotBattery*, *RobotCpu* und *RobotMisc*. Für die Position und die Orientierung bietet das Package `geometry_msgs` das Message Interface *Pose*, welches aus einem *Point* und einem *Quaternion* (Vektor mit Skalar) besteht. Auf Wunsch der betreuenden Lehrperson wird Pose nicht verwendet. *RobotPoint* und *RobotQuaternion* erweitern die Interfaces *Point* und *Quaternion* ebenfalls um das Feld `nid`.

## Rekonstruktion
Das Package `mock_robot` wurde mit dem Befehl
```bash
    ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name mock_data mock_robot --dependencies rclpy rcl_interfaces example_interfaces geometry_msgs python3-psutil
```
erstellt. Es hat nach Erzeugung bereits `mock_data` und alle notwendigen Dateien. Hier angegebene Abhängigkeiten werden der `package.xml` automatisch hinzugefügt, jedoch müssen Angaben über Maintainer, Description und Version in `package.xml` und `setup.py` anktualisiert werden. Es sei zu beachten, dass `--dependencies` `<depend>`-Tags generiert, welche eine Abhängigkeit fürs Bauen und Ausführen beschreibt. Für Python-Module ist ersteres überflüssig, weswegen die Verwendung von `<exec_depend>`-Tags empfehlen wird. In `mock_robot/mock_robot/mock_data.py` wurde die Funktionalität der Node implementiert.

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
Die Vorbereitung des Mock-Roboters erfordert eine Internetverbindung. Das Package `mock_robot` muss mit `custom_interfaces` in einem ROS2 Workspace in `src` abgelegt werden. Vom Workspace aus können die Abhängigkeiten mit `rosdep install -i --from-path src --rosdistro jazzy -y` installiert werden. `colcon build --packages-select <Package>` baut ein einzelnes Package, wonach mit `source install/local_setup.bash` das Overlay gesourct werden muss. Ist durch Modifikationen an diesem Package die Installation weiterer Packages oder Python-Module notwendig, müssen ihre entsprechenden rosdep Schlüssel der `package.xml` hinzugefügt werden.

Zur rosdep Namensauflösung: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html#how-do-i-know-what-keys-to-put-in-my-package-xml

Anschließend kann der Mock-Roboter mit `ros2 run mock_robot mock_data` manuell gestartet werden. Zur Laufzeit wird mit `ros2 node list` die Node als `mock_robot_status_pub` angezeigt. Sie verfügt über die drei Parameter `system_intervall` (default 3.0), `geo_intervall` (default 1.0) und `misc_intervall` (default 20.0), welche bestimmen, wie häufig Timer-Callback-Funktionen publishen. Da die Timer bei start der Node initialisiert werden, müssen die Parameter zur Launch-TIme festgelegt werden.

| Topic | Interface | Gruppe | Bedeutung |
|-------|-----------|--------|-----------|
| robot_battery | RobotBattery | system | Batteriestand data in % zwischen 0.0 und 100.0 oder gleich -1.0 (falls keine Informationen vorhanden sind) als 64-bit float |
| robot_cpu | BatteryCpu | system | Durchschnittliche CPU-Last data aller Kerne in der letzten Minute in % zwischen 0.0 und 100.0 als 64-bit float|
| robot_point | RobotPoint | geo | Drei (zufällige) Koordinaten x, y und z als 64-bit floats |
| robot_orientation | RobotQuaternion | geo | Vier Werte x, y, z, w als 64-bit floats |
| robot_misc | RobotMisc | misc | JSON-String data als string, welcher hier IP, MAC und datetime hat |

Die Nachrichten auf diesen Topics können mit `ros2 topic echo <Topic>` abgehört oder mit Subscriptions, die mit entsprechenden Message Interfaces lauschen, verarbeitet werden. Das Package `sw_robot` implementiert eine Node `data_sink`, welche Scubscriptions für die oben genannten Topics implementiert. 
```python
    class RobotStatusSub(Node):
        def __init__(self):
            super().__init__("robot_status_sub")
            self.__battery_sub = self.create_subscription(RobotBattery, 'robot_battery', self.battery_callback, 3)
            
        def battery_callback(self, msg):
            print(f"Battery of {msg.nid}: {msg.data}")
```
In diesem Beispiel ist zu sehen, wie eine Subscription für ein bestimmtes Topic erstellt und auf die Felder einer Message zugegriffen wird. Das Interface *RobotBattery* muss nach dem Bauen von `custom_interfaces` als Klasse importert werden. Die Klasse ist benannt nach der Datei `custom_interfaces/msg/RobotBattery.msg`.
```python
    from custom_interfaces.msg import RobotBattery
    from custom_interfaces.srv import SetRobotActivity
```

## Die Klasse MockRobotStatusPub
Der Mock-Roboter ist eine ROS2 Node `mock_data`. Implementiert wird diese als Klasse *MockRobotStatusPub* in `mock_robot/mock_robot/mock_data.py`, welche von der Klasse *rclpy.node.Node* erbt.

ROS2 Parameter:
- system_intervall - Timer Intervall für Gruppe "system" als float, default 3.0
- geo_intervall - Timer Intervall für Gruppe "geo" als float, default 1.0
- misc_intervall - Timer Intervall für Gruppe "misc" als float, default 20.0
> Hinweis: Die Timer werden zur Initialisierung erstellt. Ihre Intervalle können nur vor dem Start der Node geändert werden.

Attribute:
- __nid - Node ID als String, zu 0.7.0 noch die MAC-Adresse
- __activity - Aktivität des Mock-Roboters als String, default 'auto'.
- __activity_srv - ROS2 Service *SetRobotActivity*, löst `set_activity_callback` aus.
- __battery_pub - ROS2 Publisher als *rclpy.publisher.Publisher*. Siehe [Tabelle](#verwendung) für Parameter.
- __cpu_pub
- __point_pub
- __orientation_pub
- __misc_pub
- __system_timer - *rclpy.timer.Timer* als Auslöser für Publisher Callback-Funktionen  
- __geo_timer
- __misc_timer
- points - Liste an zweidimensionalen Vektoren als *numpy.array*. Sie spiegeln Koordinaten (Breitengrad und Längengrad als floats) auf vordefinierten Routen wieder, von denen zur Initialisierung eine zufällig ausgewählt wird.
- goal - Index des nächsten Punkts in points, default 1. Wenn __activity == 'auto', wird goal stets nach Erreichen eines Punkts inkrementiert und nach dem letzten Punkt auf 0 zurückgesetzt.
- max_vec_len - Schrittweite in Richtung eines Punkts als float, default 5.288731323495855e-05. Der Wert entspricht dem Zwölftel der Strecke zwischen zwei willkürlich gewählten Punkten.
- current - Vektor zur virtuellen Position des Mock-Roboters als *numpy.array*, default points[0]

Methoden:
- @property get_nid()
- @property get_activity()
- @property get_battery_pub()
- @property get_cpu_pub()
- @property get_point_pub()
- @property get_orientation_pub()
- @property get_misc_pub()
- set_activity_callback(request, response) - Ausgelöst durch den Service *SetRobotActivity*. Erhält eine gültige, vordefinierte Aktivität aus `('auto', 'idle', 'manual')` und antwortet, ob `__activity` erfolgreich gesetzt wurde. Für 'manual' wird versucht, Koordinaten aus einem JSON-String zu lesen und `points` mit diesen zu überschreiben.
- system_timer_callback() - Ermittelt sequentiell Batteriestand und durchschnittliche CPU-Last der letzten Minute. Beide Daten werden auf ihren jeweilen Topics veröffentlicht. Ein negativer Wert für den Batteriestand deutet an, dass dieser nicht ermittelt werden konnte.
- geo_timer_callback() - Berechnet anhand `current`, `points`, `goal` und `max_vec_len` einen neuen Vektor für `current` und veröffentlicht diesen. Für die Orientierung werden die Default-Werte von *geometry_msgs.msg.Point* veröffentlicht.
- misc_timer_callback() - Veröffentlicht MAC-
> Hinweis: Aufruf der Getter durch Decorator ohne Klammern

| RobotBattery | RobotCpu | RobotPoint | RobotQuaternion | RobotMisc | SetRobotActivity |
|--------------|----------|------------|-----------------|-----------|-----------------|
| string nid | string nid | string nid | string nid | string nid | string activity |
| float64 data | float64 data | float64 x | float64 x | string data | string details |
| | | float64 y | float64 y| | --- |
| | | float64 z | float64 z| | string msg |
| | | | float64 w | | |

## Die Klasse RobotStatusSub
Der Data Sink ist eine ROS2 Node `data_sink`. Implementiert wird diese als Klasse *RobotStatusSub* in `sw_robot/sw_robot/data_sink.py`, welche von der Klasse *rclpy.node.Node* erbt.

Attribute:
- __battery_sub - ROS2 Subscription als *rclpy.subscription.Subscription*. Siehe [Tabelle](#verwendung) für Parameter. Löst `battery_callback` aus.
- __cpu_sub - Löst `cpu_callback` aus.
- __point_sub - Löst `point_callback` aus.
- __orientation_sub - Löst `orientation_callback` aus.
- __misc_sub - Löst `misc_callback` aus.

Methoden:
- @property get_battery_sub()
- @property get_cpu_sub()
- @property get_point_sub()
- @property get_orientation_sub()
- @property get_misc_sub()
> Hinweis: Aufruf der Getter durch Decorator ohne Klammern