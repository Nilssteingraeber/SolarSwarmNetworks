# Mock-Roboter
## Übersicht
Das ROS2 Package `mock_robot` soll Ziel /Z70/ im Pflichtenheft erfüllen. Mock-Roboter sind ROS2 Nodes, die auf Nicht-Robotern gestartet werden können, um Roboterprozesse zu simulieren. Das Package soll eine einzige Node implementieren, die Mockdaten für Tests liefert, darunter: Batteriestand, Aktivität, CPU-Last (über Intervalle gemittelt), Position, Orientierung und Sonstiges (JSON-String). Aus dem Mock-Roboter wurden Anforderungen an Roboter-Statusmeldungen und eine Datensenke abgeleitet.

## Rekonstruktion
Zum Erstellung der Packages `mock_robot` wurde wird sich an den Schritten aus dem Kapitel **first_package** orientiert. Für die verschiedenen Mockdaten werden Message Interfaces benötigt. Da die Daten auf verschiedenen Topics gepublisht werden sollen, genügen die vordefinierten Interfaces aus `example_interfaces` und `geometry_msgs`. Diese werden ab Version 0.7.0 durch `custom_interfaces` Interfaces ersetzt, welche zusätzlich das Feld `nid` als String zur einfachen Identifizierung des Publishers haben. Die NID wird momentan aus der MAC einer Hostmaschine ohne `:` bestimmt. Sollte dies sich ändern, ist eine Änderung an `get_nid()` in `robot_util.py` erforderlich. Bei dem Gebrauch einer Hashfunktionen ist darauf zu achten, dass der resultierende Hash keine Sonderzeichen auaßer `_` enthält und Abhängigkeiten berücksichtigt werden. Diese Einschränkung ergibt sich dadurch, dass mit der NID Namen für Nodes und Services konstruiert werden, welche nicht alle Zeichen zulassen. Bei der Suche nach einem geeignetem Algorithmus und Modul ist weiter zu beachten, ob der rosdep-Schlüssel im Index vorhanden ist, da das Modul sonst manuell installiert werden muss.

Der Batteriestand und CPU-Last lassen sich als Floats und Aktivität und Sonstiges als String darstellen. `example_interfaces` bietet für diese Datentypen die Interfaces *String* und *Float64*. Die offizielle Dokumentation des Packages empfiehlt, diese nur für Tests zu verwenden und eigene Interfaces fürs Deployment zu definieren. Daher implementiert `custom_interfaces` *RobotBattery*, *RobotCpu*, *RobotActivity* und *RobotMisc*. Für die Position und die Orientierung bietet das Package `geometry_msgs` das Message Interface *Pose*, welches aus einem *Point* und einem *Quaternion* (Vektor mit Skalar) besteht. Auf Wunsch der betreuenden Lehrperson wird Pose nicht verwendet. *RobotPoint* und *RobotQuaternion* erweitern die Interfaces *Point* und *Quaternion* ebenfalls um das Feld `nid`.

Das Package `mock_robot` wurde mit dem Befehl
```bash
    ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name mock_data mock_robot --dependencies rclpy rcl_interfaces example_interfaces geometry_msgs python3-psutil
```
erstellt. Es hat nach Erzeugung bereits `mock_data` und alle notwendigen Dateien. Hier angegebene Abhängigkeiten werden der `package.xml` automatisch hinzugefügt, jedoch müssen Angaben über Maintainer, Description und Version in `package.xml` und `setup.py` anktualisiert werden. Es sei zu beachten, dass `--dependencies` `<depend>`-Tags generiert, welche eine Abhängigkeit fürs Bauen und Ausführen beschreibt. Für Python-Module ist ersteres überflüssig, weswegen die Verwendung von `<exec_depend>`-Tags empfohlen wird. In `mock_robot/mock_robot/mock_data.py` wurde die Funktionalität der Node implementiert. Ab `mock_robot` v0.9.0 wurden Basisfunktionalitäten, die auch für Roboter relevant sind, entkoppelt.

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
### Vorbereitung
Das Projekt beinhaltet die bereits gebauten Packages `mock_robot`, `data_sink` und `custom_interfaces`. Im entsprechenden Arbeitsverzeichnis muss zuerst das Overlay mit `source install/local_setup.bash` geladen werden.

Sollten Änderungen vorgenommen oder die Packages aus anderen Gründen neu gebaut werden müssen, ist eine Internetverbindung notwending. `mock_robot` und `data_sink` hängen beide von `custom_interfaces` ab. Die Installation der Äbhängigkeiten das Bauen erfolgen mit
```bash
    rosdep install -i --from-path src --rosdistro jazzy -y
    colcon build [--packages-select <Package>]
```
Ist durch Modifikationen an diesem Package die Installation weiterer Packages oder Python-Module notwendig, müssen ihre entsprechenden rosdep Schlüssel der `package.xml` des relevanten Packages hinzugefügt werden. Generell finden Änderungen in `src` statt und erfordern, das betroffene Packages neu gebaut werden.

Zur rosdep Namensauflösung: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html#how-do-i-know-what-keys-to-put-in-my-package-xml

### Start des Mock-Roboters
Wurde der Mock-Roboter gebaut, kann er mit `ros2 run mock_robot mock_data` manuell gestartet werden. Zur Laufzeit wird mit `ros2 node list` die Node als `mock_robot_status_pub_<nid>` angezeigt. Sie verfügt über die drei Parameter `system_intervall` (default 3.0), `geo_intervall` (default 1.0) und `misc_intervall` (default 20.0), welche bestimmen, wie häufig Timer-Callback-Funktionen Nachrichten veröffentlichen. Da die Timer bei start der Node initialisiert werden, müssen die Parameter zur Launch-TIme festgelegt werden.

Diese Tabelle stellt die Zugehörigkeit der geforderten Topics zu ihren Interfaces und den drei Timern dar:

| Topic | Interface | Gruppe | Bedeutung |
|-------|-----------|--------|-----------|
| robot_battery | RobotBattery | system | Batteriestand data in % zwischen 0.0 und 100.0 oder gleich -1.0 (falls keine Informationen vorhanden sind) als 64-bit float |
| robot_cpu | RobotCpu | system | Durchschnittliche CPU-Last data aller Kerne in der letzten Minute in % zwischen 0.0 und 100.0 als 64-bit float |
| robot_activity | RobotActivity | system | Aktuelle Aktivität als String: 'auto' (Mock-Roboter fährt Route ab) 'manual' (Punkt wird angefahren), 'idle', 'recharge' |
| robot_point | RobotPoint | geo | Drei Koordinaten x, y und z als 64-bit floats |
| robot_orientation | RobotQuaternion | geo | Vier Werte x, y, z, w als 64-bit floats |
| robot_misc | RobotMisc | misc | JSON-String data als string, welcher hier IP, MAC und datetime hat |

Die Nachrichten auf diesen Topics können mit `ros2 topic echo <Topic>` abgehört oder mit Subscriptions, die mit entsprechenden Message Interfaces lauschen, verarbeitet werden. Das Package `sw_robot` implementiert eine Node `data_sink`, welche Scubscriptions für die oben genannten Topics implementiert und kann dazu als Vorlage diesnen. Ein Beispiel wird später zur Erläuterung der Node gezeigt.

Die folgenden Services des Mock-Roboters können mit `ros2 service call` aufgerufen werden. Felder werden dabei als JSON-String gefüllt, insofern eine Eingabe erforderlich ist:

| Service | Interface | Beschreibung |
|---------|-----------|--------------|
| set_robot_activity_`nid` | SetRobotActivity | `activity` erhält einen Bezeichner für eine zu setzende Aktivität und je nach Aktivität zusätzliche Details. Momentan erlaubt: 'auto' fährt eine Route ab (Routen-Index in `details`), 'manual' fährt einen Punkt an und wird dann 'idle' (Koordinaten als JSON-String in `details`), 'recharge', 'idle'. `msg` teilt mit, ob die Änderung gelungen ist.|
| robot_service_info_`nid` | RobotServiceInfo | `service` erhält einen leeren String oder String bestehend aus alphanumerischen Zeichen, '/' und '_'. `services` enthält eine Liste aus Strings, welche verfügbare Services mit Typ getrennt durch ein Leerzeichen beinhalten (Beispiel: `/set_robot_activity_A81835345312 [custom_interfaces/srv/RobotActivity]`). Ist `service` nicht leer, wird die Liste mit dem Eingabestring gefiltert. Das erste Wort von `msg` ist 'Success' oder 'Failure'. 'Success' schließt jedoch nicht aus, dass keine Services gefunden wurden (betrachte stattdessen Länge der Liste). | 
| robot_interface_info_`nid` | RobotInterfaceInfo | `interface` erhält einen leeren String oder Bezeichnung für ein Interface. Wenn es ein solches Interface gibt, wird die Definition in `definition` zurückgegeben (`interfaces` bleibt leer). Ansonsten wird eine Liste aller Interfaces in `interfaces` zurückgeblieben (`definition` bleibt leer). |

### Übersicht der Interfaces
| RobotBattery | RobotCpu | RobotActivity | RobotPoint | RobotQuaternion | RobotMisc |
|--------------|----------|---------------|------------|-----------------|-----------|
| string nid | string nid | string nid | string nid | string nid | string nid |
| float64 data | float64 data | string activity | float64 x | float64 x | string data |
| | | | float64 y | float64 y| |
| | | | float64 z | float64 z| |
| | | | | float64 w | |

| SetRobotActivity | RobotServiceInfo | RobotInterfaceInfo |
|------------------|------------------|--------------------|
| string activity | string service "" | string interface "" |
| string details | --- | --- |
| --- | string msg | string definition |
string msg | string[] services | string[] interfaces |

In Python-Nodes werden Interfaces wie Klassen behandelt. Ihre Namen entsprechen dem Dateinamen ohne Endung. Beispielsweise wird das Interface aus `msg/RobotBattery.msg` wie folgt importiert:
```python
    from custom_interfaces.msg import RobotBattery
    # Eine Nachricht (etwa in einer Timer-Callback-Funktion) wird über den Konstruktor der Klasse erstellt
    msg = RobotBattery()
    # Felder können Attribute einer gefüllt werden 
    msg.nid = 'dummy'
    msg.data = 100.0
    # Über ein Publisher-Objekt kann mit publish(msg) eine Nachricht veröffentlicht werden
    self.publishers['battery'].publish(msg)
```

Services werden auf ähnliche Weise importiert, jedoch werden Antworten auf Service Calls mit einem `return` der Service-Callback-Funktion veröffentlicht:
```python
    from custom_interfaces.srv import RobotServiceInfo
    # Ein Methoden-Kopf erhält (neben self) zwei Parameter, häufig request und response
    # Folgender Methoden-Kopf ist Teil der Klasse BaseStatusPub:
    def service_info_callback(self, request, response):
        # Auf die Felder kann wie auf Attribute zweier Objekte zugegriffen werden
        print(request.service)
        response.msg = 'dummy'
        return response # Antwort
```

Aus Tests mit der Python built-in-Funktion `type()` geht hervor, dass `request` und `response` zwei verschiedenen Klassen angehören. In diesem Fall haben sie die Klassennamen (mit `type(request).__name__`) *RobotServiceInfo_Request* und *RobotServiceInfo_Response*. Wegen der Umsetzung der der Interfaces als Klassen, lässt sich zu diesen ein Klassendiagramm zeichnen:

![custom_interfaces Klassen](<custom_interfaces_class diagram.svg>)

## Klasse MockRobotStatusPub
Der Mock-Roboter ist eine ROS2 Node `mock_data`. Implementiert wird diese als Klasse *MockRobotStatusPub* in `mock_robot/mock_robot/mock_data.py`, welche von *BaseStatusPub* und *MockPosition* erbt.

![mock_robot Klassen](<mock_robot_class diagram.svg>)

*MockRobotStatusPub* implementiert drei Timer (Intervalle durch Parameter übergeben) mit entsprechenden Timer-Callback-Funktionen. Diese veröffentlichen Nachrichten auf den oben genannten Topics. Die genannten Publishers und Services werden im Konstruktor der Oberklasse *BaseStatusPub* erstellt. Die Service-Callback-Funktionen müssen jedoch in *MockRobotStatusPub* überschrieben werden. Für Publishers, Services, Actions und Timers werden mehrere Dictionaries angelegt. Die Schlüssel der erstellten Publishers und Services entsprechen ihren Bezeichnungen ohne `'robot'`: battery, cpu, activity, point, orientation, misc, set_activity, service_info und interface_info (als Strings).

*MockPosition* simuliert die Bewegung auf einer von drei Formen: Entlang einer Linie, eines Dreiecks oder eines Vierecks. Die Formen sind eine Liste aus zwei bis vier Koordinaten, gespeichert als Vektoren aus dem Modul `numpy` in `points`. `points` ist eine zufällig aus `mock_routes` gewählte Liste. Das Modul `numpy` erlaubt das Rechnen mit Vektoren. Eine Position `current` läuft schrittweise auf einen Punkt in `points` zu und hält dabei eine maximale Schrittweite `max_vec_len` ein. `goal` ist der Index des anzustrebenden Punktes in `points`. 
Der Geo-Timer-Callback in *MockRobotStatusPub* entscheidet, dass bei `activity == 'auto'` nach erreichen eines Zielpunktes der nächste (oder erste, falls am Ende von `points`) angestrebt werden soll und bei `activity == 'manual'` in `'idle'` gewechselt wird. Bei `activity in ('idle', 'recharge')`.

*Util* hat mehrere statische Methoden, die Systeminformationen wie Batteriestand oder MAC-Adresse bestimmen.

Durch die Verwendung der Dekoratoren `@property` (Getter) und `@<Property>.setter` (Setter) werden Attribute gekapselt. Listen und Dictionaries ohne Setter können nicht ersetzt, ihre Elemente aber manipuliert werden. Setter prüfen Parameter, melden aber nicht, wenn ein Wert nicht geändert wurde. Nur der Setter von Points erzeugt ein `ValueError`.

Methoden:
- @activity.setter activity(a) - `a` muss in allowed_activities sein.
- @points.setter points(p) - `p` muss eine nicht leere Liste mit Objekten der Klasse `numpy.ndarray` sein. Die Länge des Arrays und der Typ seiner Elemente  müssen mit denen in `points` übereinstimmen.
- @goal.setter goal(g) - `g` muss ein valider Index von points sein.
- @max_vec_len.setter max_vec_len(m) - `m` muss ein positiver Float sein.
- @current.setter current(a) - `a` muss ein Objekten der Klasse `numpy.ndarray` sein. Die Länge des Arrays und der Typ seiner Elemente müssen mit `current` übereinstimmen.
- set_activity_callback(request, response) - Ausgelöst durch den Service *SetRobotActivity*. Erhält eine gültige, vordefinierte Aktivität aus `('auto', 'idle', 'manual', 'recharge')` und antwortet, ob `activity` erfolgreich gesetzt wurde. Für `'manual'` wird versucht, Koordinaten aus einem JSON-String in `details` zu lesen und `points` mit diesen zu überschreiben. Für `'auto'` wird versucht, einen Index aus `details` für `mock_routes` zu lesen.
- system_timer_callback() - Ermittelt sequentiell Batteriestand und durchschnittliche CPU-Last der letzten Minute. Beide Daten und die Aktivität werden auf ihren jeweilen Topics veröffentlicht. Ein negativer Wert für den Batteriestand deutet an, dass dieser nicht ermittelt werden konnte.
- geo_timer_callback() - Berechnet anhand `current`, `points`, `goal` und `max_vec_len` einen neuen Vektor für `current` und veröffentlicht diesen. Für die Orientierung werden die Default-Werte von *geometry_msgs.msg.Point* veröffentlicht.
- misc_timer_callback() - Veröffentlicht IP-Adresse, MAC-Adresse, und Datetime.

> Hinweis: Getter und Setter werden implizit aufgerufen. Innerhalb der Klasse: `temp = self.activity` und `self.activity = 'idle'`.  
> Warnung: Aktuell werfen Setter keine Fehler, falls das Setzen eines Werts fehlgeschlagen ist. `points` kann jedoch `ValueError` erzeugen.


## Die Klasse RobotStatusSub
Der Data Sink ist eine ROS2 Node `data_sink`. Implementiert wird diese als Klasse *RobotStatusSub* in `sw_robot/sw_robot/data_sink.py`, welche von der Klasse *BaseStatusSub* erbt.

![data_sink Klassen](<data_sink_class diagram.svg>)

*RobotStatusPub* implementiert einen Timer (Intervall durch Parameter übergeben) mit entsprechender Timer-Callback-Funktion. Diese schreibt batchweise alle Zustände der bekannten Roboter in eine Datenbank (`forward_batch`) oder gibt sie aus (`forward_batch_test`). `__batch_timer` (`batch_intervall` default 10.0) ruft `batch_timer_callback` auf, welcher `forward_batch` aufruft und einen Zeitstempel ausgibt. `subscription_callback` wird von allen Subscriptions in `subscriptions` aufgerufen und nimmt eine Fallunterscheidung für die Klasse der Nachricht vor. `nodes` beinhaltet die `nid` einer Node als Schlüssel und ein `dict` mit Daten als Wert. Die Schlüssel dieses Dictionaries sind `'battery'`, `'cpu'`, etc.. `check_nid` prüft, ob die `nid` einer Nachricht in `nodes` vorhanden ist. Falls nicht, wird ein Eintrag mit leerem Daten-`dict` erstellt.
*BaseStatusPub* definiert `forward_batch` und die Callback-Funktionen, jedoch müssen jedoch überschrieben werden. `connect_db` versucht bei fehlender oder geschlossener Verbindung zu einer Datenbank, eine neue Verbindung aufzubauen. Dazu werden die Umgebungsvariablen `DB_NAME`, `DB_USER`, `DB_PASSWORD`, `DB_HOST` und `DB_PORT`. Die Methode gibt `True` zurück, falls zuletzt eine gültige Verbindung besteht. *RobotStatusSub* versucht in `forward_batch` bis zu fünf-mal, eine Verbindung aufzubauen mit je einer Sekunde Pause zwischen Versuchen. Die Umgebungsvariablen `DB_TABLE_NAME` und `DB_COLUMN_NAMES_*` können an die zugehörige Datenbank angepasst werden.


## Datenfluss
Im folgenden Abschnitt soll auf den Zusammenhang der Netzwerkkomponenten eingegangen werden. Zunächst sei diese Abbildung zu betrachten:

![Publisher-Subscriber bei zwei Robotern](obsolete/sw_message_flow.svg)

Grundlegend für den Verkehr von ROS2 Nachrichten ist die ROS Middleware (RMW). Sie basiere laut ROS2 Dokumentation auf dem Industriestandard DDS/RTPS (Data Distribution Service/Real-Time Publish-Subscribe Protocol), welcher für die Entdeckung von Knoten und die Serialisierung, sowie den Transport von Daten zuständig sei. Wir verwenden per Standardkonfiguration von ROS2 Jazzy eProsimas Fast DDS aufgrund der Apache 2 Lizenz.

Referenz: https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Different-Middleware-Vendors.html

Entdeckung (eng. discovery) ist dafür verantwortlich, dass sich Publisher, Subscriber, Service Clients, Service Server und sonstige Teilnehmer im ROS Graphen erkennen. Sie geht über das eigene Betriebssystem oder in unserem Fall den eigenen Container hinaus und erkennt alle erreichbaren Teilnehmer im Netzwerk mit der selben `ROS_DOMAIN_ID`.

**--- to-do: Roboter-zu-Roboter über batman-adv testen ---
was wenn in mehreren Docker Netzwerken gleichzeitig?**

In der Abbildung dargestellt sind zwei Roboter, die sich im selben Netzwerk befinden. Die gestrichelten Pfeile deuten auf den Fluss einer Nachricht der Publisher Node in Container 1 von Roboter 1. Jeder Fluss soll als eigener Fall betrachtet werden. In jedem Fall wird davon ausgegangen, dass die Discovery der RMW gelingt. Alle Container basieren auf dem Image ros:jazzy und haben somit ROS2 Jazzy installiert.

Im ersten Fall befindet sich der Subscriber in Container 1 von Roboter 1. Da sich beide ROS2 Nodes im selben Container befinden, wird kein Netzwerk benötigt. Die Nachricht kommt unmittelbar über die RMW an.  
Im zweiten Fall befindet sich der Subscriber in Container 2 von Roboter 1. Die 1 und 2 von Roboter 1 müssen über ein Netzwerk verbunden werden. Docker verbindet Container standardmäßig über ein Bridge-Netzwerk, welches üblich für Container auf dem selben Host sind.  
Im dritten Fall befinden sich Subscriber und Publisher auf verschiedenen Robotern. Um die jeweiligen Container zu verbinden, wird ein Overlay-Netzwerk benötigt, was die Teilnahme beider Docker daemons in einem Docker-Schwarm voraussetzt. Die Übertragung der Nachricht findet dann über WLAN statt.

**--- to-do: Testen, ob ob `--network host` benötigt wird**

Referenz: https://docs.docker.com/engine/network/


| Schicht | Zuordnung am OSI-Referenzmodell|
|---|--------------------------------------|
| **7** Anwendungsschicht      | ROS2 Node im Container publiziert oder empfängt Daten |
| **6** Darstellungsschicht    | Umwandlung der ROS2 Nachricht durch RMW |
| **5** Sitzungsschicht        | Sitzung durch RMW |
| **4** Transportschicht       | UDP/TCP zwischen Containern |
| **3** Vermittlungsschicht    | Routing zwischen Containern durch Docker Swarm |
| **2** Sicherungsschicht      | MAC-basiertes Mesh-Routing durch BATMAN-adv |
| **1** Bitübertragungsschicht | WLAN-Hardware |