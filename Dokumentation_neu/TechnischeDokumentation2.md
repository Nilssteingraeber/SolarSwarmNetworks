# Technische Dokumentation - SolarSwarm
Deckblatt
    Titel
    Art der Arbeit
    Hochschule, Studiengang
    Autoren
    Betreuer
    Abgabedatum

## Abstract
Im Rahmen einer Projektarbeit für das Modul Softwarepraktikum im Studiengang Informatik Bachelor wurde über die Fachsemester 4 und 5, basierend auf den Anforderungen des Betreuers, ein Softwareprojekt abgewickelt. Dieses beschäftigte sich mit dem Errichten eines selbstheilenden Netzwerks und seiner Visualisierung, wodurch eine Grundlage für zukünftige Projekte geboten werden soll. 
Diese Arbeit beschreibt die zwei wesentlichen Teile *SolarSwarm Visualizer* und *SolarSwarm Network* des Softwarepraktikums.
Kurzfassung
    Diese Dokumentation beschreibt...
    Ziel
    Vorgehensweise
    Ergebnisse

---

## Inhaltsverzeichnis
Inhaltsverzeichnis

---

## 1. Einleitung
Einleitung
    Projektkontext
    Ziel der Dokumentation
    Abgrenzung

---

## 2. Fachlicher und technischer Überblick
Theoretische Grundlagen/Stand der Technik
    Fachliche Grundlagen
    Relevante Technologien/Methoden
    Vergleich bestehender Lösungen (Netzwerkmodi; vom Betreuer ROS2 und Docker/Kybernetes vorgeschrieben - Vorkenntnisse über Docker)
    Begriffsdefinitionen

---

## 3. Anforderungen an das Projekt
Anforderungen und Analyse
    Projektziele
    Nutzer
    Rahmenbedingungen
    Lasten-/Pflichtenheft

---

## 4. Konzept und Design
Konzept und Design
    Systemübersicht
    Komponenten
    Datenmodell
    Schnittstellen
    Technologieentscheidungen mit Begründung
    Diagramme

---

## 5. Implementierung
Implementierung
    Entwicklungsumgebung
    Projektstruktur
    Zentrale Algorithmen/Module
    Besondere Herausforderungen
    Wichtige, kommentierte Codebeispiele

---

## 6. Test und Evaluation
Test und Evaluation
    Teststrategie
    Testfälle
    Testergebnisse
    Bewertung, ob Ziel erreicht
    Eventuell Performance/Usability?

---

## 7. Ergebnisse und Diskussion
Ergebnisse und Diskussion
    Was wurde erreicht?
    Abweichungen vom ursprünglichen Plan
    Reflexion
    Technische und fachliche Bewertung

---

## 8. Fazit
Fazit und Ausblick
    Zusammenfassung
    Erkenntnisse
    Verbessungsvorschläge
    Weiterentwicklungsmöglichkeiten

---

## Literaturverzeichnis
Literaturverzeichnis

---

## Anhang
Anhang
    Diagramme
    Testprotokolle
    Benutzerhandbuch
    Installationsanleitung
    Glossar





# 1 Einleitung
SolarSwarm ist ein Projekt des Instituts für Elektromobilität an der Hochschule Bochum, an dem Teams aus Studierenden aus verschiedenen Fachbereichen über Projektarbeiten gemeinsam daran arbeiten, einen autonomen, mobilen Roboterschwarm für umweltfördernde Aufgaben wie das Sammeln von Müll, Pflanzen von Bäumen oder Pflegen von Grünflächen zu entwickeln. Die technische Umsetzung und Analyse hinsichtlich der ökonomischen Auswirkungen dieses Projekts sollen durch die interdisziplinäre Zusammenarbeit gelingen. Im Rahmen einer Projektarbeit für das Modul Softwarepraktikum im Studiengang Informatik Bachelor wurde von unserem Team über das vierte und fünfte Fachsemester ein Softwareprojekt für SolarSwarm abgewickelt. Um eine Grundlage für den Roboterschwarm zu schaffen, sollte ein vom Internet getrenntes, lokales Netzwerk für die Kommunikation, Kooperation, Überwachung und Steuerung der Roboter errichtet und eine benutzerfreundliche Webapplikation entwickelt werden. In einer anfänglichen Einarbeitungsphase wurde sich mit Docker und dem Robot Operating System (ROS2) vertraut gemacht und eine Lösung für das Bilden eines eigenen Netzwerks gesucht. Mit vier zukünftig als Roboter vorgesehenen Linux-Computern als Mock-Roboter wurden regelmäßig Netzwerk- und Softwaretests durchgeführt, neue Bedürfnisse festgestellt und inkrementell auf eine Lösung hingearbeitet. Obwohl eine Grundlage besteht, wird zusätzlicher Aufwand und Verständnis über unser System erforderlich, um Mock-Daten durch echte Daten zu ersetzen oder das Datenmodell zu erweitern. Wegen der zeitlichen Einschränkung konnten keine Feldtests mit den Computern gemacht werden. Somit ist nicht klar, wie leistungsstark ihre Hardware ist oder wie sich Störung oder hohe Auslastung des Netzwerks auf das System auswirken. Eine besondere Herausforderung stellt der Verlust des Quorums im Schwarm dar. Die momentane Lösung dafür kann zur kurzzeitigen Unterbrechung mancher Dienste führen. Diese Arbeit dokumentiert die zwei wesentlichen Erzeugnisse dieser Projektarbeit, SolarSwarm Visualizer und SolarSwarm Networks, mit dem Ziel, weitere Teams aus möglicherweise unterschiedlichen Fachbereichen auf die Arbeit mit der von uns gezeugten Grundlage vorzubereiten. Dazu sollen die Architektur und Komponenten beschrieben, Entscheidungen begründet und Verwendung und Erweiterung erläutert werden, sowie Probleme und offene Aufgaben diskutiert werden.

# 2 Architektur
# 2.1 Übersicht
# 2.2 Ebenen
# 3 Komponenten
## 3.1 Anwendungs-Ebene
### 3.1.1 Roboter
### 3.1.2 Datensenke
### 3.1.3 Backend
### 3.1.4 Frontend
### 3.1.5 Tile-Server
## 3.2 Docker-Ebene
### 3.2.1 Was ist Docker?
### 3.2.2 Docker Services
compose und stack
### 3.2.3 Docker Swarm
### 3.2.4 Quorum an Managern
## 3.3 System-Ebene
### 3.3.1 Systemkonfiguration
### 3.3.2 Service-Helfer-Skript
### 3.3.3 Systemd Services
# 4 Verwendung
## 4.1 Installation
## 4.2 Start
## 4.3 Logging
## 4.4 Problembehandlung
# 5 Erweiterung