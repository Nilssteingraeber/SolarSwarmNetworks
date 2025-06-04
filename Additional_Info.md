# ROS Signing Key Expiration & Neue Pakete für die Repository-Verwaltung

Die **ROS Signing Key Expiration** steht wieder bevor (1. Juni), und ein neues Paket wurde eingeführt, um die **Schlüssel- und Repository-Verwaltung** für ROS-Benutzer zu vereinfachen.

Die folgenden Pakete sind jetzt verfügbar:
- **`ros-apt-source`** und **`ros2-apt-source`** für Ubuntu
- **`ros2-release`** für RHEL  

Diese Pakete enthalten sowohl den **ROS Signing Key** als auch die **Repository-Konfiguration** für **ROS** und **ROS 2**.

## Warum ist das wichtig?  

Dieses Update ist ein großer Schritt in Richtung **vereinfachter Schlüsselaktualisierungen**, sodass ROS-Benutzer **synchron bleiben** – solange ihr System aktuell ist!  

Aus **Sicherheitsgründen** war die Möglichkeit, **Schlüssel zu rotieren**, ohne Benutzer zu beeinträchtigen, ein langfristiges Ziel des **Infrastructure PMC**. Dies stellt den **ersten Schritt** zur Erreichung dieses Ziels dar.

## Paketvarianten  

Für **Ubuntu/Debian**-Distributionen gibt es zwei Paketvarianten:
- **`ros-apt-source`** & **`ros2-apt-source`** – Enthalten Schlüssel und Repository-Konfigurationen für die Hauptrepositories von **ROS** und **ROS 2**:
  - [ROS Main Repository](http://packages.ros.org/ros)
  - [ROS 2 Main Repository](http://packages.ros.org/ros2)
- **`ros-testing-apt-source`** & **`ros2-testing-apt-source`** – Enthalten Schlüssel und Repository-Konfigurationen für die **ROS-Testrepositories**:
  - [ROS Testing Repository](http://packages.ros.org/ros-testing)
  - [ROS 2 Testing Repository](http://packages.ros.org/ros2-testing)

Für **RHEL** gibt es ein einzelnes Paket, **`ros2-release`**, das **beide Repository-Typen** verwaltet und das **Hauptrepository** standardmäßig aktiviert.

---

## Migrationsanleitung: Nach dem 1. Juni  

Es gibt **zwei Möglichkeiten**, auf das neue Paket zu migrieren:

### Option 1: Manuelle Aktualisierung & Migration  

Aktualisieren Sie den Signing Key manuell mit den folgenden Befehlen:

```bash
sudo rm /usr/share/keyrings/ros-archive-keyring.gpg
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```



### Option 2: Entfernen der vorherigen Konfiguration für ROS 2

Vor der Migration sollte die bestehende Konfiguration entfernt werden:

```bash
sudo rm /etc/apt/sources.list.d/ros2.list 
sudo rm /usr/share/keyrings/ros-archive-keyring.gpg
```

#### Reference [ROS Signing Key Migration Guide](https://discourse.ros.org/t/ros-signing-key-migration-guide/43937#p-93537-how-do-i-migrate-after-june-1st-4)
