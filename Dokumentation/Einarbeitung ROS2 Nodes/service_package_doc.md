# Einleitung
Dieser Abschnitt baut auf `first_package_doc.md` auf. An Stelle eines Publisher-Subscriber-Modells soll hier ein einfaches Service-Client-Modell mit einem eigenen Dateninterface und Variablen für den implementiert werden. Services und Clients werden jeweils als eigene ROS2 Node. Da der Erzeugungsprozess dem der vorherigen Nodes ähnelt, werden die notwendigen Schritte kurzgefasst.

1. ROS2 sourcen:
    - `source /opt/ros/jazzy/setup.bash` in jeder neuen Konsole oder `echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc` zum Automatisieren
2. Workspace (hier `solarswarm`) und darin ein Verzeichnis `src` anlegen
3. In `solarswarm/src` ein Package erzeugen:
    - `ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name simple_service service_demo`
4. In `package.xml` und `setup.py` Informationen zum Author und Package füllen:
    - `<description>` und `<maintainer>` in `package.xml`
    - `maintainer`, `maintainer_email` und `description` in `setup.py`
5. In `solarswarm/src/service_demo/service_demo` werden die Nodes angelegt:
    - `simple_service.py` liegt bereits vor und muss bearbeitet werden
    - `simple_client.py` muss noch angelegt und bearbeitet werden
6. In `setup.py` Entrypoints hinzufügen:
    ```python
        entry_points={
            'console_scripts': [
                'echo_service = service_demo.simple_service:main',
                'echo_client = service_demo.simple_client:main',
            ],
        },
    ```

Als Vorlage dienen der Service und Client der offiziellen Dokumentation: https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html

Nach jeder Änderung muss von `solarswarm` aus das Package mit `colcon build --packages-select service_demo` neu gebaut werden. Mit `rosdep install -i --from-path src --rosdistro jazzy -y` werden Abhängigkeiten aller Packages in `src` installiert. Zur Ausführung sollte man das Overlay mit `source install/local_setup.bash` sourcen.

Gestartet wird der Service mit `ros2 run service_demo echo_service`. Der Client wird mit `ros2 run service_demo echo_client` gestartet.

# ROS2 Parameter
In ROS2 ermöglichen Parameter es, Werte während der Ausführung einer Node oder beim Start mit einer `launch file` oder `run`-Argumenten ändern zu können. Sie sind eine eigene Klasse aus `rclpy` und müssen vor ihrer Verwendung deklariert werden:
- Im Konstruktor der Node werden Parameter mit `self.declare_parameter('<Bezeichnung>', <Default-Wert>)` deklariert.
- Parameter-Objekte verwenden mit dem Konstruktor `rclpy.parameter.Parameter(name, type_=None, value=None)` initialisiert.
- `type_` ist ein Wert aus einer Enumeration (die Klasse `Parameter.Type`). Auf die Typen kann direkt zugegriffen werden, beispielsweise mit `rclpy.Parameter.Type.STRING`, oder einer aus einem Wert abgeleitet werden mit der Klassenmethode `from_parameter_value(parameter_value)`.
- `self.get_parameter('<Bezeichnung>')` gibt das Parameter-Objekt zu einem deklariertem Parameter zurück.
- `<Parameter-Objekt>.get_parameter_value()` gibt den aktuellen Wert eines Parameters zurück.
- Mit `self.set_parameters(<Liste>)` überschreiben alle Parameter-Objekte in der übergebenen Liste die entsprechenden Parameter.

Bei der Deklaration kann einem Parameter ebenfalls eine Beschreibung übergeben werden:
- Die Beschreibung wird der Klasse `ParameterDescriptor` übergeben, welche mit `from rcl_interfaces.msg import ParameterDescriptor` importiert werden kann.
- Die Beschreibung wird mit `self.declare_parameter('<Bezeichnung>', <Default-Wert>, ParameterDescriptor(description='<Beschreibung>'))` hinzugefügt und mit `ros2 param describe <Node> <Parameter>` ausgegeben.

Referenz: https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html
rclpy Dokumentation: https://docs.ros.org/en/jazzy/p/rclpy/api/parameters.html

Nodes können auf Änderungen eigener Parameter oder Parameter anderer Nodes reagieren: 
```python
    # Unter imports
    from rclpy.parameter_event_handler import ParameterEventHandler

    # In __init__ nach Deklaration eines Parameters
    self.handler = ParameterEventHandler(self)
    self.callback_handle = self.handler.add_parameter_callback(
        parameter_name='<Parameter>',
        node_name='<Node>',
        callback=self.callback,
    )
```

Referenz: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Monitoring-For-Parameter-Changes-Python.html

Zum Arbeiten mit Parametern gibt es folgende Befehle:
- `ros2 param list` gibt die Parameter-Bezeichnungen aller Nodes aus.
- `ros2 param get <Node> <Parameter>` gibt den Wert eines Parameters aus als `<Typ> value is: <Wert>`.
- `ros2 param set <Node> <Parameter> <Wert>` ändert den Wert eines Parameters.
- `ros2 param dump <Node>` gibt alle Parameter mit ihren Werten im yaml-Format aus.
- `ros2 param load <Node> <yaml-Datei>` lädt alle Werte aus einer Datei.
- Bei einem Start mit `run` können die Parameter der Datei mit den Argumenten `--ros-args --params-file <Datei>` übergeben werden.

Referenz: https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html

Die Node `echo_client` hat zwei Parameter:
```python
    # In __init__
    self.declare_parameter('nid', 0, ParameterDescriptor(description='Eine nutzerdefinierte ID der Node'))
    self.declare_parameter('msg', 'Default message', ParameterDescriptor(description='Zu schickende Nachricht'))

    # In main
    nid = self.req.nid = self.get_parameter('nid').get_parameter_value()
    msg = self.req.msg = self.get_parameter('msg').get_parameter_value().string_value
```

# ROS2 Interfaces
## Allgemeines
ROS2 Interfaces definieren den Inhalt von Anfragen und Antworten von Nodes. In `solarswarm/src/service_demo` werden die Verzeichnisse `msg`, `srv` und `action` benötigt. In einer Datei mit der Endung `.msg`, `.srv` und `.action` können Attributnamen und Datentypen für eine Message (Publisher-Subscriber), einen Service (Request-Response) oder eine Action (Request-Response-Feedback) definiert werden:
- Grundsätzliches Schema (Kleinbuchstaben): `<datentyp> <attributname>`
- Default-Werte: `<datentyp> <attributname> <wert>`
- Konstanten (Großbuchstaben): `<DATENTYP> <ATTRIBUTNAME>=<wert>`
- Kommentare: `#`
- In `.srv` wird der Inhalt zwischen Request und Response geteilt mit einer Zeile `---`
- In `.action` wird der Inhalt zwischen Request, Response und Feedback (regelmäßige Updates) geteilt mit einer Zeile `---`

Beispiel für `.srv`:
```
    # Request
    int32 a
    int32 b
    ---
    # Response
    int32 sum
```

Zu jedem Datentyp kann ein Array angelegt werden. Die offizielle Dokumentation bietet einige Beispiele für die verwendete Syntax:
```
    int32[] unbounded_integer_array
    int32[5] five_integers_array
    int32[<=5] up_to_five_integers_array

    string string_of_unbounded_size
    string<=10 up_to_ten_characters_string

    string[<=5] up_to_five_unbounded_strings
    string<=10[] unbounded_array_of_strings_up_to_ten_characters_each
    string<=10[<=5] up_to_five_strings_up_to_ten_characters_each
```

Übersicht der Datentypen:

![Tabelle der Datentypen](ros2_interface_datatypes.png)

Referenz: https://docs.ros.org/en/jazzy/Concepts/Basic/About-Interfaces.html

## Interface anlegen
Da Interfaces nur mit dem Build Tool ament_cmake gebaut werden können und `service_demo` ament_python benutzt, erstellen wir in `solarswarm/src` ein Package mit `ros2 pkg create --build-type ament_cmake --license Apache-2.0 custom_interfaces`.

In `custom_interfaces/src/EchoServiceIf.srv` werden die Daten des Services definiert:
```
    int32 nid
    string msg
    ---
    string msg
```

Erweiterung in `package.xml` bei den anderen depend-Tags:
```xml
    <buildtool_depend>rosidl_default_generators</buildtool_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
```

Erweiterung in `CMake.txt`: 
```
    # Unter find_package(ament_cmake REQUIRED)
    find_package(rosidl_default_generators REQUIRED)

    set(srv_files
    "srv/EchoService.srv"
    )
    rosidl_generate_interfaces(${PROJECT_NAME}
    ${srv_files}
    )

    # In der vorletzten Zeile
    ament_export_dependencies(rosidl_default_runtime)
```

Import in `simple_service.py` und `simple_client.py`:
```python
    from custom_interfaces.srv import EchoServiceIf
```

Abhängigkeiten in `package.xml` der `service_demo`:
```xml
    <build_depend>custom_interfaces</build_depend>
    <exec_depend>custom_interfaces</exec_depend>
```

Anschließend müssen beide Packages mit `colcon build` gebaut und mit `source ~/solarswarm/install/local_setup.bash` gesourct werden. Dann können Service und Client in zwei verschiedenen Terminals gestartet werden mit `ros2 run service_demo simple_service` und `ros2 run service_demo simple_client`.

Ist der Service noch nicht verfügbar, sieht die Ausgabe des Clients so aus:
```
    [INFO] [1750189690.223504935] [echo_client]: service not available, waiting again...
```

Ist er aktiv, erscheint regelmäßig etwa: 
```
    [INFO] [1750189888.309450007] [echo_client]: Echo received: Default message
    [INFO] [1750189890.318656151] [echo_client]: Echo received: Default message
```

Der Service zeigt etwa:
```
    [INFO] [1750189890.314953478] [echo_service]: Received from 0:
    Default message
```

Referenz: https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html

## Parameter Tests
Der `simple_client` schickt mehrmals eine Anfrage an den Service. Für diesen Test soll dies in einer Endlosschleife geschehen, um zur Laufzeit Parameter verändern und die Auswirkungen sehen zu können. Zum Starten der Nodes: `ros2 run service_demo simple_service` und `ros2 run service_demo simple_client`

Erster Test:
- Ein Service und Client werden gestartet. Mit `ros2 param list` werden ihre Parameter ausgegeben.
- Mit `ros2 param get <Node> <Parameter>` wird der Wert der `nid` des Clients ausgegeben.
- Mit `ros2 param set <Node> <Parameter>` wird der Wert der `nid` des Clients auf `'1'` gesetzt. Die Ausgabe sollte sich verändern.
- Mit `ros2 param set <Node> <Parameter>` wird der Wert der `msg` des Clients auf `Hello World` gesetzt. Erneut sollte sich die Ausgabe verändern.
- Mit `ros2 param dump <Node> > param_test_dump.yaml` werden die Parameter-Werte gepseichert und der Client beendet.
- Anschließend wird der Client mit `ros2 run service_demo simple_client --ros-args --params-file param_test_dump.yaml` neugestartet.
- Die Ausgabe sollte wieder gleich sein. Mit `ros2 param get <Node> <Parameter>` können die Parameter-Werte nochmal geprüft werden.

Zweiter Test:
- Ein Service und ein Clients werden gestartet. Mit `ros2 param list` werden ihre Parameter ausgegeben.
- Mit `ros2 param set <Node> <Parameter>` wird der Wert der `nid` des Clients auf `'1'` gesetzt.
- Anschließend wird ein zweiter Client gestartet. Eventuell kommt es zu einem Konflikt, da beide `echo_test` heißen sollten.

# to-do: Tests durchführen