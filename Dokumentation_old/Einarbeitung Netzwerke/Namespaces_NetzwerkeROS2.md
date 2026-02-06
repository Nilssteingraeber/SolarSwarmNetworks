# Namespaces und ROS2-Netzwerk

## Namespaces in ROS2

Ein Namespace in ROS2 ist eine Möglichkeit, die Namen von Knoten, Themen, Diensten und anderen ROS-Entitäten zu organisieren und zu isolieren. Dies ist besonders nützlich, wenn mehrere Roboter oder Systeme ähnliche Namen verwenden.

### Beispiel für Namespaces

Angenommen, du hast zwei Roboter, die beide das Thema `/cmd_vel` verwenden, um Geschwindigkeitsbefehle zu empfangen. Ohne Namespaces könnten diese Themen in Konflikt geraten. Mit Namespaces kannst du die Themen wie folgt unterscheiden:

- **Roboter 1**: `/robot1/cmd_vel`
- **Roboter 2**: `/robot2/cmd_vel`

Hier sind `robot1` und `robot2` die Namespaces, die zu den Themen hinzugefügt werden.

### Verwendung von Namespaces

Ein Namespace kann für einen Knoten festgelegt werden, indem der Parameter `__ns` verwendet wird, wenn der Knoten gestartet wird. Zum Beispiel:

```bash
ros2 run my_package my_node --ros-args --remap __ns:=robot1
```
Dieser Befehl startet den Knoten my_node im Namespace robot1.





# ROS2-Netzwerk

## Einführung

Ein ROS2-Netzwerk bezieht sich auf die Kommunikation zwischen verschiedenen Knoten in einem ROS2-System. ROS2 verwendet **DDS (Data Distribution Service)** als Middleware, um eine flexible und effiziente Datenverteilung zu ermöglichen.

## Beispiel für ein ROS2-Netzwerk

Angenommen, du hast zwei Roboter, die miteinander kommunizieren müssen. Jeder Roboter hat einen Knoten, der Geschwindigkeitsbefehle veröffentlicht, und einen anderen Knoten, der diese Befehle abonniert.

- **Roboter 1**: Veröffentlicht Geschwindigkeitsbefehle auf `/robot1/cmd_vel`
- **Roboter 2**: Abonniert Geschwindigkeitsbefehle von `/robot1/cmd_vel`

In diesem Fall ermöglicht das ROS2-Netzwerk die Kommunikation zwischen den Knoten der beiden Roboter.

## Einrichtung eines ROS2-Netzwerks

Um ein ROS2-Netzwerk einzurichten, müssen alle Knoten korrekt konfiguriert sein und die richtigen Themen und Dienste verwenden. Hilfreiche Tools zum Überprüfen der Themen und Knoten im Netzwerk:

```bash
ros2 topic list
ros2 node list
```


## Zusammenfassung

- **Namespaces** helfen, Namenskonflikte zu vermeiden und ROS-Entitäten besser zu organisieren.
- **ROS2-Netzwerk** ermöglicht die Kommunikation zwischen verschiedenen Knoten in einem ROS2-System.
- Beide Konzepte können zusammen verwendet werden, um ein robustes und organisiertes ROS2-System zu erstellen.

