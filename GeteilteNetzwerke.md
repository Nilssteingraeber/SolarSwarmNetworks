# Überlegungen
Für unser Projekt wollen wir ein physisches Netzwerk über WLAN selbst aufbauen. Dabei gehen wir davon aus, dass mobile Roboter dieses nutzen werden und sich unabhängig von einem Router bewegen können. Wir bezeichnen diese Roboter als Knoten. Das Netzwerk soll skalierbar und stabil sein, was fordert, dass sich problemlos dem Netzwerk anschließen und davon lösen können.

Es gibt zwei Herangehensweisen an diese Aufgabe:
- Hotspot: Unter mehreren nahegelegenen Knoten wird einer zum Hotspot ernannt und mittelt die Kommunikation der anderen. Denkbar problematisch hieran ist, dass viel Last auf einen Knoten fällt und sich alle in dessen Nähe befinden müssen. Dazu muss je Anhäufung entschieden werden, wer zum Hotspot wird, und es gibt einen Single Point of Failure. Wegen der begrenzten Reichweite unserer Roboter, würden sie sich entweder ballen oder stetig neue Netzwerke errichten, die sich einander nicht sehen. Zur Abhilfe Logik für die Hotspots zu implementieren, wann sie mergen oder Knoten beauftragen, neue Netzwerke zu bilden, wäre aber sehr aufwendig.
- Peer-to-Peer: Alle Knoten tragen der Netzwerkkommunikation bei. Dadurch sollte weniger Last auf einzelne Knoten fallen und Ausfälle weniger Knoten kein großes Problem darstellen. Weiter könnten Knoten räumlich Ketten bilden und das Netzwerk sich so weiter erstrecken. Generell wären Knoten "freier", wenn sie sich unabhängig von einem Hotspot bewegen könnten.

Da Mobilität und Skalierbarkeit besonders wichtig für Solarswarm sind, beschäftigen wird uns weiter mit Peer-to-Peer-Netzwerken. Eine weit verbreitete Lösung sind Ad-hoc-Netzwerke.

# Ad-hoc-Netzwerk
7SINGAL beschreibt Ad-hoc-Netzwerke als dezentrale Netzwerke, bei denen alle Knoten gleichberechtigt sind, Daten aneinander weiterleiten und sich am Routing beteiligen. Sie seien flexibel, skalierbar, und kosteneffizient, weshalb sie in vielen Bereichen benutzt werden. Wegen ihrer Unabhöngigkeit von existierender Infrastruktur seien diese Netzwerke schnell aufbaubar und ausfallsicher.

Ad-hoc eignet sich aus vielen Gründen sehr gut für unsere Zwecke. Da unsere Roboter voraussichtlich auf Feldern, Straßen oder in Parks agieren werden, werden wir keine Infrastruktur haben und wegen der großen Distanzen häufig Knoten aus dem Netzwerk wegfallen oder sich ihm anschließen. Wegen der Gleichstellung ist das Netzwerk ausfallsicher und werden keine speziellen Rollen oder Rechte benötigt. Auch fallen kaum Kosten für das Projekt an, da lediglich WLAN-fähige Geräte benötigt werden, deren Netzwerkkarten Ad-hoc unterstützen, und Ad-hoc schnell konfiguriert werden kann.

Quelle: https://www.7signal.com/ad-hoc-network

## MANETs und WSNs
7SIGNAL beschreibt verschiedene Arten von Ad-hoc-Netzwerken und ihre Anwendungsbereiche, von denen zwei für uns relevant sind:
- Mobile Ad-hoc-Netzwerke (MANETs) seien Netzwerke, bei denen die Knoten mobil sind und sich frei bewegen können. Sie werden häufig in Militär- und Notfalleinsatzszenarien verwendet.
- Drahtlose Sensornetzwerke (WSNs) seien Netzwerke aus räumlich verteilten Sensoren, die Umweltbedingungen überwachen und aufzeichnen und in Anwendungen wie Umweltüberwachung und industrieller Automatisierung eingesetzt werden.
Quelle: https://www.7signal.com/ad-hoc-network

Eine Studie aus dem Jahr 2016 vergleicht unter anderem Performanz, Anwendungen und Netzwerktopologien beider Arten. Die Forscher stellten fest, dass WSNs üblicherweise aus einer großen Anzahl an starren Sensoren oder Knoten bestehen. Sie haben eine zentrierte Topologie mit einem "Sink" als Sammelknoten der Daten. MANETs hingegen haben üblicherweise eine geringe Anzahl an mobilen Knoten. Sie haben eine dynamische, verteilte Topologie, die sich mit Bewegung der Knoten ändert. WSNs seien aufgrund ihrer starren Topologie bei längeren Anwendungen mit wenigen Veränderungen des Netzes in Anwendung. MANETs hingegen werden oft temporär errichtet. Notfälle und Militäroperationen werden als Beispiele benannt.

Quelle: https://ijircce.com/admin/main/storage/app/pdf/WKhmzqHcQIdYXUfKs3fWTZdX7jZhXlVXP4b52aV0.pdf

Zur Überwachung oder zum Sammeln von Daten in einer Datenbank könnte es für uns später interessant sein, einen "Sink" im Netzwerk zu haben, um Sensordaten zu erhalten. Das lässt sich aber auch in einem MANET mit abgesonderten Knoten umsetzen. Aufgrund unserer Ansprüche eignet sich für unsere Zwecke ein MANET eher.

## Unterstützung überprüfen
Ad-hoc wird mit dem IBSS Modus umgesetzt, welcher von der Netzwerkkarte der Roboter unterstützt werden muss. Für MANETs wird jedoch üblicherweise der Mesh Modus bzw. 802.11s verwendet.
Mit `lspci -v` oder `iw list` kann überprüft werden, ob die Modi unterstützt werden.
Quelle: https://superuser.com/questions/1141862/setting-up-wireless-mesh-network-open-80211s-in-ubuntu
Alternativ bietet https://wireless.docs.kernel.org/en/latest/en/users/drivers.html eine übersicht aller Wireless Treiber ihre Modi.

# B.A.T.M.A.N. advanced (batman-adv)
Für eine konkrete Umsetzung benötigen wir ein Routing-Protokoll. Da es einfach umzusetzen ist und häufig empfohlen wird, verwenden wir batman-adv und das davon bereitgestellte Netzwerk-Interface bat0. Dafür benötigen wird das Kernel-Modul batman-adv und das Tool batctl zum Konfigurieren und Debuggen.

Zum Laden des Kernel-Moduls: `sudo modprobe batman-adv`
Zum automatischen Laden nach Systemstart: https://www.open-mesh.org/projects/batman-adv/wiki/Debian_batman-adv_AutoStartup

Zu Beginn richten wir uns nach dem öffiziellen Guide. Falls nötig, nehmen wir weitere Konfigurationen vor und dokumentieren diese später.
Zum Guide: https://www.open-mesh.org/projects/batman-adv/wiki/Quick-start-guide

Im FAQ zu batman-adv wird zu Beschränkungen folgendes angemerkt:
    "Batman has no security implemented. Also assigning IP addresses to the node(s) is not Batman's task.
    You may want to use underlying security mechanisms, like: IBSS RSN."
    Quelle: https://www.open-mesh.org/projects/batman-adv/wiki/Faq
Wir werden uns nicht mit der Sicherheit dieses Protokolls beschäftigen. Bei auf batman-adv basierenden Anwendungen sollten Schwächen und Lösungen jedoch untersucht werden.

## Kurzfassung zum Einrichten
Installation von batctl
    `sudp apt update && sudo apt install batctl`
Laden des Kernel-Moduls
    `sudo modprobe batman-adv`
Erstellen eines bat0-Interfaces
    ```iw dev wlan0 del
    iw phy phy0 interface add wlan0 type ibss
    ip link set up mtu 1532 dev wlan0
    iw dev wlan0 ibss join my-mesh-network 2412 HT20 fixed-freq 02:12:34:56:78:9A
    batctl if add wlan0
    ip link set up dev bat0
    sudo avahi-autoipd bat0```