# Datenfluss
Bisher wurden die wichtigsten Komponenten des Roboter-Schwarms, das Ad-hoc Netzwerk über BATMAN-adv, die Containerisierung und Orchestrierung von Services durch Docker und die Umsetzung von Robotern durch ROS2 isoliert betrachtet. Im folgenden Abschnitt soll dargestellt werden, welchen Zweck die integrierten Komponenten in der Übertragung von Daten haben werden.

## Einordnung in das OSI-Referenzmodell
Docker - arbeitet primär auf Schicht 2-4, ROS2 arbeitet auf  Schicht 4-7
| Schicht | Zuordnung          |
|---|--------------------------|
| **7** Anwendungsschicht      |
| **6** Darstellungsschicht    |
| **5** Sitzungsschicht        |
| **4** Transportschicht       |
| **3** Vermittlungsschicht    | 
| **2** Sicherungsschicht      | MAC-basiertes Mesh-Routing durch BATMAN-adv |
| **1** Bitübertragungsschicht | WLAN-Hardware |

ROS Node
ROS Middleware (DDS/RTPS)
Docker Container
Docker (docker0 -> virtuelles bridge Interface (default))
BATMAN-adv
WLAN

## Betrachtung Akteure
- 2 Nodes in einem Container
- 2 Nodes in verschiedenen Containern
- 2 Nodes auf verschiedenen Robotern
![Publisher-Subscriber bei zwei Robotern](sw_message_flow.svg)


