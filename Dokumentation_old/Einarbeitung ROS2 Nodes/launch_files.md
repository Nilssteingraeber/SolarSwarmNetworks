# Launch Files
ROS2 verfügt über einen Befehl `ros2 launch [Package] <launch_file>`, welcher nach Angaben einer `launch file` Nodes starten oder stoppen, sowie Trigger auslösen oder auf sie reagieren kann. Launch files erleichtern es, komplexere ROS2 Systeme zu starten ersparen die manuelle Übergabe von `run`-Argumenten zur Konfiguration.

Launch files können in drei Formaten geschrieben werden: XML, YAML und Python
```xml
    <?xml?>
    <launch>
        <node pkg="<Package>" exec="<Node>" name="<Node Name>" namespace="<Namespace>"/>
        <node pkg="<Package>" exec="<Node>" name="<Node Name>" namespace="<Namespace>"/>
    </launch>
```
```yaml
    %YAML
    ---
    launch:
        - node:
            pkg: "<Package>"
            exec: "<Node>"
            name: "<Node Name>"
            namespace: "<Namespace>"
        - node:
            pkg: "<Package>"
            exec: "<Node>"
            name: "<Node Name>"
            namespace: "<Namespace>"
```
```python
    #!/usr/bin/python
    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                package="<Package>",
                namespace="<Namespace>",
                executable="<Node>",
                name="<Node Name>"
            ),
            Node(
                package="<Package>",
                namespace="<Namespace>",
                executable="<Node>",
                name="<Node Name>"
            )
        ])
```

Um eine launch file von einem Package benutzen zu können, sollte `<exec_depend>ros2launch</exec_depend>` der entsprechenden `package.xml` hinzugefügt werden, um sicherzustellen, dass der `launch`-Befehl und alle Formate erkannt werden.

Referenz: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html

Launch files ermöglichen es auch, Argumente mit Default-Werten zu deklarieren, andere launch files zu inkludieren, launch files durch Gruppen in bestimmten Namespaces zu inkludieren und Mappings anzugeben. Mit `ros2 launch <Arg>:=<Wert>` können Argumente übergeben werden. 

Teil eines Beispiels zu `arg` in YAML:
```yaml
  %YAML
  ---
  launch:
    - arg:
      name: "background_r"
      default: "0"
    - node:
      pkg: "turtlesim"
      exec: "turtlesim_node"
      name: "sim"
      namespace: "turtlesim2"
      param:
      - name: "background_r"
        value: "$(var background_r)"

```


Vollständiges Beispiel zu allen Schlüsselwörtern und Formaten: https://docs.ros.org/en/jazzy/How-To-Guides/Launch-file-different-formats.html

Hinweis: Die Schlüsselwörter unterscheiden sich nicht zwischen XML und YAML. Das Python-Format hingegen sieht sehr anders aus und importiert Klassen aus dem Modul `launch`. Das Python-Format ist das aufwendigste der drei, bietet aber die Möglichkeit, dynamische Logik, Event Handler oder Timer/Verzögerung zu implementieren. In XML und YAML gibt es beispielsweise keine Schlüsselwörter, um Abhängigkeiten zwischen Nodes zu definieren. Kann eine Node erst nach einer anderen gestartet werden, muss in Python ein Event Handler oder Verzögerung implementiert werden.

Der offizielle roslaunch-Artikel zu ROS2 erwähnt, dass eventuell ein deterministisches Startup implementiert wird. Aussage im ROS1 Wiki:
> roslaunch does not guarantee any particular order to the startup of nodes – although this is a frequently requested feature, it is not one that has any particular meaning in the ROS architecture as there is no way to tell when a node is initialized.

Erwähnung im roslaunch-Artikel zur ROS2:
> Hopefully this is another case on which the launch system for ROS 2 can improve, at least for nodes with a lifecycle, a.k.a. Managed Nodes.


Referenz: https://design.ros2.org/articles/roslaunch.html

# Beispiel zu service_demo (nicht getestet)
```yaml
    %YAML 1.2
    ---
    launch:
        - node:
            pkg: "service_demo"
            exec: "echo_service"
            name: "echo_service"
            namespace: "echo"

        - node:
            pkg: "service_demo"
            exec: "echo_client"
            name: "echo_client"
            namespace: "echo"
            param:
            -
                name: "nid"
                value: "11"
            -
                name: "msg"
                value: "Hello World!"
```
![Ausgabe ros2 launch](service_demo_launch_output.png)