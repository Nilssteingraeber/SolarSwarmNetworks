#  ROS 2: Ausführliche Grundkonzepte

##  1. Packages
- **Was ist ein Package?**  
  Ein Package in ROS 2 ist eine modulare Einheit, die Quellcode, Konfigurationen und Ressourcen für eine bestimmte Funktionalität zusammenfasst. Sie ist der grundlegende Baustein für die Organisation und Wiederverwendung von Code in einem ROS-System.

- **Inhalte typischer Packages**:
  - Python- oder C++-Nodes
  - Launch-Files zum Starten von Komponenten
  - Parameter-Dateien (.yaml)
  - Schnittstellen wie Messages (`.msg`) oder Services (`.srv`)

- **Erstellung eines neuen Packages**:
  ```bash
  ros2 pkg create --build-type ament_cmake mein_paket
  ```

---

##  2. Nodes
- **Was ist ein Node?**  
  Ein Node ist ein ausführbarer Prozess, der Teil eines Robotersystems ist. Jeder Node sollte eine spezifische, klar abgegrenzte Aufgabe übernehmen – zum Beispiel das Auslesen eines Sensors oder die Ansteuerung eines Motors.

- **Vorteile**:
  - Modularität: einzelne Komponenten lassen sich separat entwickeln und debuggen.
  - Fehlertoleranz: ein abgestürzter Node betrifft nicht das gesamte System.
  - Wiederverwendbarkeit: gut designte Nodes können in verschiedenen Projekten eingesetzt werden.

- Aktive Nodes anzeigen:
  ```bash
  ros2 node list
  ```
- Informationen zu einem Node abrufen:
  ```bash
  ros2 node info /node_name
  ```
---

##  3. Topics & Messages
- **Topics** sind benannte Kanäle, über die Nodes Daten senden (publish) oder empfangen (subscribe) können. Die Kommunikation ist asynchron und basiert auf dem pub/sub-Prinzip.

- **Messages** definieren das Datenformat, das über ein Topic übertragen wird. Sie basieren auf `.msg` Dateien, z. B.:
  ```text
  string data
  ```

- **Beispiel:**
  - Ein Publisher kann Nachrichten wie folgt senden:
    ```python
    self.publisher_.publish(String(data="Hallo ROS2"))
    ```
  - Ein Subscriber registriert sich für ein Topic:
    ```python
    self.subscription = self.create_subscription(
        String,
        'chatter',
        callback,
        10  # Queue size
    )
    ```
- Alle verfügbaren Topics auflisten:
  ```bash
  ros2 topic list
  ```
- Nachrichten eines Topics anzeigen:
  ```bash
  ros2 topic echo /chatter
  ```
- Nachricht manuell auf ein Topic publizieren:
  ```bash
  ros2 topic pub /chatter std_msgs/String "data: 'Hallo Terminal!'"
  ```
- Typ eines Topics anzeigen:
  ```bash
  ros2 topic type /chatter
  ```
- Nachrichtentyp beschreiben:
  ```bash
  ros2 interface show std_msgs/msg/String
---

##  4. Services
- **Was ist ein Service?**  
  Services ermöglichen eine synchrone Kommunikation zwischen zwei Nodes. Der Client sendet eine Anfrage (Request), der Server antwortet darauf (Response).

- **Struktur einer `.srv` Datei**:
  ```text
  int64 a
  int64 b
  ---
  int64 sum
  ```

- Liste aller verfügbaren Services:
  ```bash
  ros2 service list
  ```
- Typ eines Service anzeigen:
  ```bash
  ros2 service type /add_two_ints
  ```
- Service aufrufen:
  ```bash
  ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 5}"
  ```

---

##  5. Actions
- **Wozu Actions?**  
  Actions erweitern Services um Feedback-Meldungen und ermöglichen damit das Management von lang laufenden Prozessen, wie z. B. Navigation oder Greifvorgänge.

- **Komponenten**:
  - Goal (Ziel)
  - Feedback (Zwischenergebnisse)
  - Result (Endergebnis)

- **Typische Anwendungsfälle**:
  - Roboter-Navigation (Zielposition erreichen)
  - Manipulator-Tasks (Greifen, Platzieren)
  - Prozesse mit Abbruchmöglichkeiten

- Alle registrierten Actions auflisten:
  ```bash
  ros2 action list
  ```
- Typ einer Action anzeigen:
  ```bash
  ros2 action info /some_action
  ```
- Beispiel-Aufruf:
  ```bash
  ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
  ```

---

##  6. Parameters
- **Was sind Parameter?**  
  Parameter sind konfigurierbare Werte, die das Verhalten eines Nodes zur Laufzeit beeinflussen. Sie ermöglichen flexible, wiederverwendbare Nodes.

- **Arten von Parametern**:
  - Standard-Datentypen: `bool`, `int`, `double`, `string`, `list`
  - Konfigurationsquellen: Kommandozeile, YAML-Dateien, Setzen zur Laufzeit

- **Beispiel-Setzen eines Parameters**:
  ```bash
  ros2 param set /mein_node max_speed 1.5
  ```
- Wert eines Parameters abrufen:
  ```bash
  ros2 param get /mein_node max_speed
  ```

##  7. Middleware (DDS)
- **Was ist DDS?**  
  DDS (Data Distribution Service) ist ein Publish/Subscribe-Middleware-Standard für verteilte Systeme. Es ermöglicht ROS 2, Nodes effizient und zuverlässig über Netzwerke hinweg zu verbinden.

- **RMW (ROS Middleware Wrapper)**  
  ROS 2 nutzt eine Abstraktionsschicht, um verschiedene DDS-Implementierungen auszutauschen (z. B. Fast DDS, Cyclone DDS, RTI).

- **Vorteile von DDS**:
  - Echtzeitfähigkeit
  - QoS-Steuerung (Quality of Service)
  - Skalierbarkeit und Robustheit

- DDS-Implementierung anzeigen:
  ```bash
  echo $RMW_IMPLEMENTATION
  ```
- Umgebungsvariable setzen (z. B. Cyclone DDS aktivieren):
  ```bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ```

---

##  8. Praktische Tools & Befehle

| Zweck                | Beispielbefehl                                                  |
|---------------------|------------------------------------------------------------------|
| Liste aller Topics  | `ros2 topic list`                                                |
| Nachrichten publizieren | `ros2 topic pub /status std_msgs/String "data: 'Online'"`     |
| Topic-Inhalte anzeigen | `ros2 topic echo /status`                                     |
| Services anzeigen    | `ros2 service list`                                             |
| Parameter setzen     | `ros2 param set /mein_node name Wert`                           |
| Actions auflisten    | `ros2 action list`                                              |
| Nodes überwachen     | `ros2 node info /node_name`                                     |

---

