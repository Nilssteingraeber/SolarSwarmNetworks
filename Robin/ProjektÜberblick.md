# Projektübersicht & Status

## Überblick – Projekt
| Thema            | Inhalt                                                                 |
|------------------|------------------------------------------------------------------------|
| Ziel             | Verteiltes Robotersystem ohne zentrale Instanz, modulare Microservices, automatische Synchronisation, UI für Monitoring & Steuerung |
| Kerntechnologien | **Netzwerk**: batman-adv, Docker, ROS2 Jazzy<br>**Frontend**: VueJS (Node), Cesium, Tilemaker, Tileserver<br>**Backend**: FastAPI, Uvicorn<br>**Datenbank**: PostgreSQL (noch nicht vollständig) |
| Besonderheit     | Automatisierte Selbstorganisation der Roboter im Mesh-Netzwerk, flexible Service-Platzierung und Synchronisation bei Netzzusammenführungen |

---

## Projektstatus & Planung
| Bereich              | Erledigt ✅                           | Offen / To-Do ❌                                      | Zeitplanung (grober Rahmen) | Priorität |
|-----------------------|---------------------------------------|-------------------------------------------------------|-----------------------------|-----------|
| **Frontend**          | Grundstruktur erstellt<br>3D-Visualisierung (Cesium)<br>OpenStreetMap Integration (NRW)<br>Timeline-Feature<br>Anzeige mehrerer Roboter<br>UI mit dynamischen Elementen<br>Generierung von Custom-Geo-Daten (Boxen, Zylinder, Pfade) | Einbindung an die Datenbank<br>Recording der Daten (für Timeline)<br>Export der Custom-Geo-Daten | Platzhalter | Mittel |
| **Backend**           | FastAPI + Uvicorn Webserver<br>Docker-Containerisierung<br>API-Endpunkte (GET/POST) für: Status, Roboter, Start, Neighbors, State, Statechange | ROS-Befehle über FastAPI in Docker-Container aufrufbar machen<br>Test auf DB mit Datasink<br>RCLPY-Integration in FastAPI | Platzhalter | Hoch |
| **Datenbank**         | PostgreSQL-Anbindung eingerichtet    | Vollständige Implementierung, Anbindung an Backend, Tests<br>Test: schreibt Data Sink korrekt in DB? | Nächste Woche geplanter Test | Hoch |
| **Netzwerk (batman-adv, Docker, ROS2)** | Setup & Tests erfolgreich | Integration aller Technologien, Synchronisation, Failover-Handling<br>Unklar: Wie soll Docker Swarm später aussehen/funktionieren? (bisher nur Roboter + Datasink vorgesehen) | Platzhalter | Sehr hoch |
| **Microservices (ROS2)** | Drei Services vorhanden:<br>- `set_robot_activity_<nid>` (SetRobotActivity)<br>- `robot_service_info_<nid>` (RobotServiceInfo)<br>- `robot_interface_info_<nid>` (RobotInterfaceInfo)<br>Details siehe Datei `Dokumentation/Austausch/austausch_v.md` | Entwicklung & Integration weiterer eigener Services<br>Prüfen, wie die ROS2 Services sinnvoll in Gesamtsystem eingebunden werden | Platzhalter | Sehr hoch |
| **Integration (Merge)**| –                                   | Frontend ↔ Backend, DB ↔ Backend, UI ↔ Services       | Platzhalter | Sehr hoch |
| **Dokumentation**     | Grobes Pflichtenheft & GitHub        | Überarbeitung & Ergänzung während/ nach Integration    | Platzhalter | Hoch |
| **Geräte**            | PI & NUC & Linux-Maschine<br>Tests erfolgreich, NUC und PI mit sehr guter Verbindung | Weitere Tests im Zusammenspiel mit Netzwerk und Backend | Platzhalter | Hoch |
| **Testing**           | Mock-Roboter Tests teilweise vorhanden | Erweiterte Unit-Tests, Simulation mit Mock-Robotern    | Platzhalter | Hoch |

---

## Bekannte Herausforderungen
- **Frontend**: Sehr große Datengrößen der 3D-Meshes → aufwändiges Parsen/Detailgrad; fehlende DB-Einbindung; Abstraktion noch uneinheitlich.  
- **Backend**: Test auf Datenbank mit Datasink fehlt noch; RCLPY kann nicht direkt in FastAPI importiert werden, wenn das Skript nicht im ROS-Workspace liegt.  
- **Netzwerk/Docker**: Noch keine klare Vorstellung, wie der Swarm später aussehen und funktionieren soll.  
- **Datenmanagement**: Noch unklar, mit wie vielen Robotern der Data Sink synchron arbeiten kann (Producer-Consumer-Problem). Dieses Thema soll nach den Klausuren vertieft werden.  

---

## Nächste Schritte
- **Frontend**
  - Datenbank-Anbindung vorbereiten und implementieren  
  - Recording der Roboter-Daten für Timeline einbauen  
  - Export-Funktion für Custom-Geo-Daten (Boxen, Zylinder, Pfade) entwickeln  

- **Backend**
  - ROS-Befehle über FastAPI-Endpunkte in Docker lauffähig machen  
  - Test: Datasink schreibt korrekt in PostgreSQL-Datenbank  
  - RCLPY-Integration im ROS-Workspace prüfen und lauffähig machen  

- **Datenbank**
  - Vollständige Implementierung der Tabellen/Modelle  
  - Schnittstellen-Tests mit Backend und Datasink durchführen  

- **Netzwerk**
  - Konzept für Docker Swarm-Architektur entwickeln (Roboterrollen, Datasink, Manager/Worker)  
  - Weitere Integrationstests mit batman-adv und ROS2  

- **Microservices**
  - Mock-Robot Services (`SetRobotActivity`, `RobotServiceInfo`, `RobotInterfaceInfo`) in die Architektur einbinden  
  - Entwicklung weiterer Services für Robotermanagement und Synchronisation  

- **Testing**
  - Erweiterte Unit-Tests für Schnittstellen und Datenfluss  
  - Simulationen mit Mock-Robotern (Nachbarn, Signalstärke, Synchronisation)  

- **Dokumentation**
  - Dokumentation parallel zur Entwicklung aktualisieren  
