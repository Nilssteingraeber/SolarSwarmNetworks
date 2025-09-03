# Projektübersicht & Status

## Überblick – Projekt
| Thema            | Inhalt                                                                 |
|------------------|------------------------------------------------------------------------|
| Ziel             | Verteiltes Robotersystem ohne zentrale Instanz, modulare Microservices, automatische Synchronisation, UI für Monitoring & Steuerung |
| Kerntechnologien | **Netzwerk**: batman-adv, Docker Swarm, ROS2 Jazzy<br>**Frontend**: (noch zu verifizieren)<br>**Backend**: FastAPI<br>**Datenbank**: (noch nicht vollständig) |
| Besonderheit     | Automatisierte Selbstorganisation der Roboter im Mesh-Netzwerk, flexible Service-Platzierung und Synchronisation bei Netzzusammenführungen |

---

## Projektstatus & Planung
| Bereich              | Erledigt ✅                           | Offen / To-Do ❌                                      | Zeitplanung (grober Rahmen) | Priorität |
|-----------------------|---------------------------------------|-------------------------------------------------------|-----------------------------|-----------|
| **Frontend**          | Grundstruktur erstellt                | Technologien verifizieren, Feinschliff, Testing        | Platzhalter                  | Mittel    |
| **Backend**           | Erste Umsetzung begonnen              | Technologien verifizieren, API-Erweiterung             | Platzhalter                 | Hoch      |
| **Datenbank**         | Basis vorhanden                       | Vollständige Implementierung, Anbindung an Backend     | Platzhalter                   | Hoch      |
| **Netzwerk (batman-adv, Docker, ROS2)** | Setup & Tests erfolgreich | Integration aller Technologien, Synchronisation, Failover-Handling | Platzhalter | Sehr hoch |
| **Microservices (ROS2)** | –                                   | Entwicklung & Integration eigener Services             | Platzhalter                   | Sehr hoch |
| **Integration (Merge)**| –                                   | Frontend ↔ Backend, DB ↔ Backend, UI ↔ Services       | Platzhalter                   | Sehr hoch |
| **Dokumentation**     | Grobes Pflichtenheft & Github        | Überarbeitung & Ergänzung während/ nach Integration    | Platzhalter                   | Mittel    |
| **Geräte**     | PI & NUC & Linux-Maschine                 | Tests erfolgreich -> NUC und PI hervorragend Verbindung    | Platzhalter                   | Hoch    |
| **Testing**           | Mock-Roboter Tests teilweise vorhanden        | Erweiterte Unit-Tests, Simulation mit Mock-Robotern    | Platzhalter                    | Hoch      |
