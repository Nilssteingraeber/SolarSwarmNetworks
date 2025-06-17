# Überlegungen
Für unser Projekt wollen wir ein physisches Netzwerk über WLAN selbst aufbauen. Dabei gehen wir davon aus, dass mobile Roboter dieses nutzen werden und sich unabhängig von einem Router bewegen können. Wir bezeichnen diese Roboter als Knoten. Das Netzwerk soll skalierbar und stabil sein, was fordert, dass sich problemlos dem Netzwerk anschließen und davon lösen können. Zum Testen verwenden wir NUC Mini-PCs.

Es gibt zwei Herangehensweisen an diese Aufgabe:
- Hotspot: Unter mehreren nahegelegenen Knoten wird einer zum Hotspot ernannt und mittelt die Kommunikation der anderen. Denkbar problematisch hieran ist, dass viel Last auf einen Knoten fällt und sich alle in dessen Nähe befinden müssen. Dazu muss je Anhäufung entschieden werden, wer zum Hotspot wird, und es gibt einen Single Point of Failure. Wegen der begrenzten Reichweite unserer Roboter, würden sie sich entweder ballen oder stetig neue Netzwerke errichten, die sich einander nicht sehen. Zur Abhilfe Logik für die Hotspots zu implementieren, wann sie mergen oder Knoten beauftragen, neue Netzwerke zu bilden, wäre aber sehr aufwendig.
- Peer-to-Peer: Alle Knoten tragen der Netzwerkkommunikation bei. Dadurch sollte weniger Last auf einzelne Knoten fallen und Ausfälle weniger Knoten kein großes Problem darstellen. Weiter könnten Knoten räumlich Ketten bilden und das Netzwerk sich so weiter erstrecken. Generell wären Knoten "freier", wenn sie sich unabhängig von einem Hotspot bewegen könnten.

Da Mobilität und Skalierbarkeit besonders wichtig für Solarswarm sind, beschäftigen wird uns weiter mit Peer-to-Peer-Netzwerken. Eine weit verbreitete Lösung sind Ad-hoc-Netzwerke.

# Ad-hoc-Netzwerk
7SINGAL beschreibt Ad-hoc-Netzwerke als dezentrale Netzwerke, bei denen alle Knoten gleichberechtigt sind, Daten aneinander weiterleiten und sich am Routing beteiligen. Sie seien flexibel, skalierbar, und kosteneffizient, weshalb sie in vielen Bereichen benutzt werden. Wegen ihrer Unabhöngigkeit von existierender Infrastruktur seien diese Netzwerke schnell aufbaubar und ausfallsicher.

Ad-hoc eignet sich aus vielen Gründen sehr gut für unsere Zwecke. Da unsere Roboter voraussichtlich auf Feldern, Straßen oder in Parks agieren werden, werden wir keine Infrastruktur haben und wegen der großen Distanzen häufig Knoten aus dem Netzwerk wegfallen oder sich ihm anschließen. Wegen der Gleichstellung ist das Netzwerk ausfallsicher und werden keine speziellen Rollen oder Rechte benötigt. Auch fallen kaum Kosten für das Projekt an, da lediglich WLAN-fähige Geräte benötigt werden, deren Netzwerkkarten Ad-hoc unterstützen, und Ad-hoc schnell konfiguriert werden kann.

Referenz: https://www.7signal.com/ad-hoc-network

## MANETs und WSNs
7SIGNAL beschreibt verschiedene Arten von Ad-hoc-Netzwerken und ihre Anwendungsbereiche, von denen zwei für uns relevant sind:
- Mobile Ad-hoc-Netzwerke (MANETs) seien Netzwerke, bei denen die Knoten mobil sind und sich frei bewegen können. Sie werden häufig in Militär- und Notfalleinsatzszenarien verwendet.
- Drahtlose Sensornetzwerke (WSNs) seien Netzwerke aus räumlich verteilten Sensoren, die Umweltbedingungen überwachen und aufzeichnen und in Anwendungen wie Umweltüberwachung und industrieller Automatisierung eingesetzt werden.
Referenz: https://www.7signal.com/ad-hoc-network

Eine Studie aus dem Jahr 2016 vergleicht unter anderem Performanz, Anwendungen und Netzwerktopologien beider Arten. Die Forscher stellten fest, dass WSNs üblicherweise aus einer großen Anzahl an starren Sensoren oder Knoten bestehen. Sie haben eine zentrierte Topologie mit einem "Sink" als Sammelknoten der Daten. MANETs hingegen haben üblicherweise eine geringe Anzahl an mobilen Knoten. Sie haben eine dynamische, verteilte Topologie, die sich mit Bewegung der Knoten ändert. WSNs seien aufgrund ihrer starren Topologie bei längeren Anwendungen wie Umweltmessungen mit wenigen Veränderungen des Netzes in Anwendung. MANETs hingegen werden oft temporär errichtet. Notfälle und Militäroperationen werden als Beispiele benannt.

Referenz: https://ijircce.com/admin/main/storage/app/pdf/WKhmzqHcQIdYXUfKs3fWTZdX7jZhXlVXP4b52aV0.pdf

Zur Überwachung oder zum Sammeln von Daten in einer Datenbank könnte es für uns später interessant sein, einen "Sink" im Netzwerk zu haben, um Sensordaten zu erhalten. Das lässt sich aber auch in einem MANET mit abgesonderten Knoten umsetzen. Aufgrund unserer Ansprüche eignet sich für unsere Zwecke ein MANET eher.

## Unterstützung überprüfen
Ad-hoc lässt sich mit dem IBSS Modus umsetzen, welcher von der Netzwerkkarte der Roboter unterstützt werden muss. Speziell für ein MANET benötigen wir ein Mesh. Mit `lspci -v` oder `iw list` kann überprüft werden, ob diese Modi unterstützt werden, wobei letzterer entweder `mesh` oder `mesh point` heißen kann.
Befehle aus: https://superuser.com/questions/1141862/setting-up-wireless-mesh-network-open-80211s-in-ubuntu

`iw list` gibt unter "Supported interface modes" eine Liste aller unterstützten Modi aus. Die Netzwerkkarte des NUC unterstützt IBSS, managed, AP, AP/VLAN, monitor, P2P-client, P2P-GO und P2P-device, jedoch nicht den Mesh Modus. Dieser muss nicht unbedingt von der Netzwerkkarte selbst unterstützt werden. Anstatt sie zu ersetztn, genügt die Verwendung von USB-WLAN-Sticks, welche auch aufgrund der begrenzten Reichweite der NUCs zum Einsatz kommen könnten. Weiter stellte sich bei der Suche nach einem geeigneten Routing-Protokoll raus, dass verbreitete Protokolle auch mit dem IBSS Modus Mesh-Netzwerke errichten können.

# Wahl eines Routing-Protokolls
Für eine konkrete Umsetzung benötigen wir ein Routing-Protokoll, sodass sich Nodes gegenseitig finden und Pakete ihr Ziel erreichen. B.A.T.M.A.N. (advanced), Babel und OLSR gehören zu weit verbreiteten Protokollen. Sie sind typisch für MANETs, doch keins davon ist eindeutig optimal. Laut einem älteren Vergleich der Universidad Central de Venezuela habe die Forschung noch kein eindeutiges Optimum gefunden. Tests würden häufig unter optimalen Bedingungen indoor durchgeführt werden, weswegen auch eine eigene Testreihe unter Berücksichtigung verschiedener Bandbreiten von ihr gemacht wurde. Aus den Tests geht zwar hervor, dass es hinsichtlich Throughput und Verlust bei hoher Bandbreite bemerkbare Unterschiede zwischen den Protokollen gibt, jedoch können wir nicht allein augrunddessen eine Entscheidung treffen. Zum einen haben die verglichenen Protokolle seit 2013 Erweiterungen bekommen oder sich Anforderungen angepasst, zum anderen steht die Umsetzbarkeit unseres Projektes im Vordergrund.
Referenz: https://ve.scielo.org/pdf/rfiucv/v28n1/art02.pdf

Wir orientieren uns daher an Trends, Empfehlungen und Zugänglichkeit der Protokolle. Ein Trend lässt sich in der Freifunk Gemeinschaft erkennen, welche sich am Betreiben von unabhöngigen Mesh-Netzwerken beteiligen. In diesen sei OLSR lange Zeit das "Standardprotokoll", doch gebe es mittlerweile konkurrierende Alternativen. B.A.T.M.A.N. und Babel werden als Beispiele aufgeführt, wobei B.A.T.M.A.N. immer beliebter werde. freifunk.net hält eine besondere Stärke dessen fest:
    "Dieses kann flexibler mit den spontanen Veränderungen in Meshnetzwerken umgehen, denn die Information über die besten Verbindungen zwischen allen B.A.T.M.A.N.-Knoten ist auf das gesamte Netz verteilt." 
Referenz: https://freifunk.net/worum-geht-es/technik-der-community-netzwerke/

B.A.T.M.A.N. oder sein Nachfahre B.A.T.M.A.N. advanced werden ebenfalls in Foren als auch von ChatGPT empfohlen. Es sei leicht einzurichten und benötige keine manuelle IP-Konfiguration. Im Weiteren basieren wir daher unser Netzwerk auf diesem Routing-Protokoll.

# B.A.T.M.A.N. advanced (batman-adv)
B.A.T.M.A.N. steht für "Better Approach to Mobile Ad-hoc Networking". In der Dokumentation zum Prinzip von BATMAN (zur Lesbarkeit ohne Punkte) wird vergleichend OLSR als "momentan meist eingesetzte Protokoll" für Drahtlose Ad-hoc-Netzwerke bezeichnet, welches trotz seiner Erweiterungen durch wachsende Netzwerke herausgefordert werde, da jeder sich Knoten bei Änderungen über einen neu zu berechnenden Graphen stets aller anderen Knoten bewusst sein muss. Hingegen seien sich BATMAN-Knoten (auch Originators genannt) nur ihrer Nachbarn bewusst.
BATMAN zeichnet aus, dass das Netzwerk mit Originator Messages (OGMs) geflutet wird, wodurch Knoten einander ihre Existenz preisgeben. Durch diese Broadcasts erhalte ein Knoten mehrere OGMs desselben Ausgangsgnoten über seine Nachbarn und könne anhand ihrer Ankunftszeit und Verlässlichkeit den besten Hop-Nachbarn bestimmen. Das ist das Hauptprinzip von BATMAN und relevant, um sich der Möglichkeit der Aggregation von OGMs zur Reduzierung von Protokoll-Overhead und Vermeidung von Broadcast Loops.
Referenz: https://www.open-mesh.org/projects/open-mesh/wiki/BATMANConcept

BATMAN advanced verwendet MAC-Adressen auf der 2. Ebene des OSI-Modells, während seine Konkurrenten meist auf der 3. Ebene routen und Knoten durch IP-Adressen repräsentiert werden.
Referenz: https://www.open-mesh.org/projects/batman-adv/wiki/Tweaking
Es ist zu dem als Kernel-Modul implementiert worden, um im Kernel Modus effizienter Pakete zu verarbeiten, und verwendet Netzwerk Interfaces - konventionell mit batX (z.B. bat0) durchnummeriert. Inzwischen können Knoten mehrere Interfaces haben und an mehreren Meshes gleichzeitig teilnehmen, weswegen sie einander zugeordnet werden müssen. Wir benötigen nur ein Mesh.
Genaueres zum Hinzufügen mehrerer: https://www.open-mesh.org/projects/batman-adv/wiki/Tweaking

Aus dem konzeptuellen Design von BATMAN-adv ergeben sich einige Eigenschaften, von denen das BATMAN Team berichet:
- network-layer agnostic - you can run whatever you wish on top of batman-adv: IPv4, IPv6, DHCP, IPX ..
- nodes can participate in a mesh without having an IP
- easy integration of non-mesh (mobile) clients (no manual HNA fiddling required)
- roaming of non-mesh clients
- optimizing the data flow through the mesh (e.g. interface alternating, multicast, forward error correction, etc)
- running protocols relying on broadcast/multicast over the mesh and non-mesh clients (Windows neighborhood, mDNS, streaming, etc)
Referenz: https://www.open-mesh.org/projects/batman-adv/wiki/Wiki

Ein Mesh kann mit Gateways und nicht-Knoten Clients (sogenannten Nachbarn) interagieren und unterstützt Roaming. Gateways bieten die möglichkeit, dass das Netzwerk oder Clients mit der Außenwelt interagieren können. Da unsere Roboter aus Sicherheitsgründen keinen Internetzugriff haben werden, benötigen wir keine. Clients in unser Netzwerk einzubinden könnte zum Aufnehmen von Sensorddaten oder Überwachen des Netzwerks interessant sein. Die offizielle Dokumentation bietet Hilfen, um diese Features zu implementieren, zu verstehen und zu optimieren.
Nicht-BATMAN-Knoten einbauen: https://www.open-mesh.org/projects/batman-adv/wiki/Quick-start-guide
Funktionsweise: https://www.open-mesh.org/projects/batman-adv/wiki/Client-announcement
Roaming optimieren: https://www.open-mesh.org/projects/batman-adv/wiki/Client-roaming

Eine Übersicht der erweiterten Features von BATMAN advanced bietet eine Präsentation aus dem Jahr 2014: https://downloads.open-mesh.org/batman/papers/batman-adv_v_intro.pdf 

## Voraussetzungen
Für den Start mit BATMAN-adv auf Linux benötigen wird das Kernel-Modul `batman-adv` und das Kommandozeilentool `batctl` zum Konfigurieren und Debuggen. Das Kernel-Modul ist im Kernel der neueren Linux Distributionen bereits enthalten. Es kann mit `sudo modprobe batman-adv` geladen werden. Zum automatischen Laden nach Systemstart bietet open-mesh eine Anleitung an: https://www.open-mesh.org/projects/batman-adv/wiki/Debian_batman-adv_AutoStartup

Zur Sicherheit sei angemerkt: Im FAQ zu batman-adv wird zu Beschränkungen folgendes angemerkt:
> "Batman has no security implemented. Also assigning IP addresses to the node(s) is not Batman's task.
> You may want to use underlying security mechanisms, like: IBSS RSN."
Referenz: https://www.open-mesh.org/projects/batman-adv/wiki/Faq
Wir werden uns nicht mit der Sicherheit dieses Protokolls beschäftigen. Auf batman-adv basierende Projekte sollten Schwächen und Lösungen weiter untersucht werden.

## Einrichtung
Zu Beginn richten wir uns nach dem offiziellen Guide von Open Mesh. Falls nötig, nehmen wir weitere Konfigurationen vor und dokumentieren diese später.
Zum Guide: https://www.open-mesh.org/projects/batman-adv/wiki/Quick-start-guide

Sollte das im Guide verwendete WLAN-Interface `wlan0` nicht verfügbar sein, kann man sich mit `ip link` vorhandene Interfaces anzeigen lassen. Beim NIC heißt das WLAN-Interface `wlp0s20f3`. Das verwendete Tool `iw` muss zusätzlich installiert werden.

### Angepasst an NUC Mini-PCs
Für den NUC sehen die Schritte des offiziellen Guide wie folgt aus:

Installation von batctl: `sudp apt update && sudo apt install batctl`
Laden des Kernel-Moduls: `sudo modprobe batman-adv`
Erstellen eines bat0-Interfaces:
```bash
    sudo iw dev wlp0s20f3 del
    sudo iw phy phy0 interface add wlp0s20f3 type ibss
    sudo ip link set up mtu 1532 dev wlp0s20f3
    sudo iw dev wlp0s20f3 ibss join my-mesh-network 2412 HT20 fixed-freq 02:12:34:56:78:9A #Fehler bei diesem Schritt
    sudo batctl if add wlp0s20f3
    ip link set up dev bat0
```
Installation und Aktivierung von `avahi-autoipd` für automatische IP Vergabe:
```bash
    sudo apt install avahi-autoipd`
    sudo avahi-autoipd bat0`
```

`sudo iw dev wlp0s20f3 del` löscht temporär das Netzwerkinterface, wodurch der NUC keinen Internetzugang mehr hat. Den Mini-PC neuzustarten, ist der einfachste Weg, um alle Änderungen umzukehren. Demnach muss nach jedem Systemstart das Kernel-Modul (falls nicht automatisch) geladen und das Interface neu konfiguriert werden.

Der vierte Befehl zum Erstellen eines bat0-Interfaces schlägt auch beim Weglassen optionaler Parameter fehl: `command failed: Operation not supported (-95)`.
Dennoch gibt `batctl if` nach Auführung der restlichen zwei Befehle `wlp0s20f3: active` aus. Auch die automatische Zuweisung einer IP-Adresse funktioniert. Es ist somit ein aktives BATMAN-Interface, jedoch muss noch getestet werden, ob es tatsächlich Teil des Netzwerks `my-mesh-network` ist.

### Alternativ iwconfig statt iw
Installation von batctl: `sudp apt update && sudo apt install batctl`
Laden des Kernel-Moduls: `sudo modprobe batman-adv`
Erstellen eines bat0-Interfaces:
```bash
    sudo iwconfig wlp0s20f3 mode ad-hoc # WLAN muss ausgeschaltet sein, sonst Fehler "Driver or resource busy"
    sudo iwconfig wlp0s20f3 essid my-mesh-network # Optional freq 2412M anhängen
    sudo ip link set up mtu 1532 dev wlp0s20f3 # hiervor WLAN wieder einschalten
    sudo batctl if add wlp0s20f3
    ip link set up dev bat0
```
`iwconfig` Befehle aus: https://wireless.docs.kernel.org/en/latest/en/users/documentation/iw/replace-iwconfig.html#join-an-ibss-ad-hoc-network

WLAN im Terminal ein-/ausschalten mit `nmcli radio wifi on` bzw. `off`.
Referenz: https://askubuntu.com/a/834194

Installation und Aktivierung von `avahi-autoipd` für automatische IP Vergabe:
```bash
    sudo apt install avahi-autoipd`
    sudo avahi-autoipd bat0`
```

Hier werden keine Fehlermeldungen ausgegeben und auch die IP-Zuweisung gelingt. Dieser Ansatz sollte ebenfalls getestet werden. Anders zum davorigen Ansatz ist, dass der NUC weiterhin einen Internetzugang hat.

## Konfigurieren und debuggen mit batctl
Mit dem Kommandozeilentool `batctl` können Interfaces zu Mesh-netzwerken hinzugefügt oder entfernt werden, Parameter von batman-adv geändert und Features von batman-adv ein- oder ausgeschaltet werden. Seine gesamte Dokumentation kann nach der Installation in seiner `man page` (einsehbar mit `man batctl`) aufgerufen werden.
Sie wird auch online angeboten: https://downloads.open-mesh.org/batman/manpages/batctl.8.html

Zum Debuggen bietet `batctl` die Möglichkeit, Knoten anzupingen, Routing Loops aus Logs zu lesen und dem Kernel-Modul live Informationen zu entnehmen. Ist der Kernel mit `debugfs` kompiliert (was standardmäßig der Fall sei), könne `batctl` mehr wiedergeben. Das BATMAN Team listet weitere mit `debugfs` entnehmbare Informationen auf:
- A list of other mesh nodes in the network (originators).
- Lists of none-mesh nodes connected to the network (clients or neighbors).
- A list of available gateways in the network.
- Log messages from the batman-adv module (if debug is compiled into the module).
Referenz: https://www.open-mesh.org/projects/batman-adv/wiki/Using-batctl

Mit `batctl` können wir sicherstellen, dass unser Netzwerk funktioniert und Knoten darin sichtbar sind. Das BATMAN Team bietet auch eine Troubleshooting Sektion mit häufig aufkommenden Problemen und Lösungen: https://www.open-mesh.org/projects/batman-adv/wiki/Troubleshooting

## Loops
In der Netzwerktechnik betrachet man zwei Arten von Loops: Routing Loops (Schicht 3) und Bridge/Switch (Schicht 2) Loops. Beide beschreiben eine Situation, in der ein Paket immer wieder zwischen den selben Knoten verschickt wird und nie sein vorhergesehenes Ziel erreicht. Dieses Paket verbraucht Bandbreite solange es lebt und kann zu einer erheblicher Verlangsamung bis hin zu einem Stillstand einiger Knoten oder im schlimmsten Fall des ganzen Netzwerks führen. Die beiden Arten unterscheiden sich darin, wo und wie ein Loop entstehen kann.

**Routing Loops** können durch Fehler im Routing-Table mehrerer Knoten entstehen. In der einfachsten Form kann ein Fehler entstehen, wenn beispielsweise zwei Knoten A und B mit einander und einem dritten Knoten C verbunden sind. A möchte C erreichen und weiß, dass die Route zu C über B günstiger ist und schickt ein Paket an B. Wenn C ausfällt und B nicht erfährt, dass A ebenfalls keine Verbindung zu C mehr hat, schickt B das Paket zu A zurück. Teilt B nun A nicht mit, dass er C nicht erreichen konnte, versucht A das Ziel C über B zu erreichen. Ein Loop entsteht. Weitere Pakete an C häufen sich an, sodass die gesamte Bandbreite zwischen A und B aufgebraucht werden kann. Bei größeren Netzwerken würden dazwischenliegende Knoten ebenfalls beeinträchtigt werden.
Referenz: https://www.geeksforgeeks.org/what-is-routing-loop-and-how-to-avoid-routing-loop/

**Bridge Loops** können durch redundante Wege zu einem einem Ziel entstehen. Zum einen können falsche Verkabelung oder Konfiguration dafür verantwortlich sein. Werden zwei Ports eines Switches mit einem Kabel oder zwei Switches (welche auch Bridges sind) redundant miteinander verbunden, könne ein solcher Loop zu "Broadcast Storms" führen. In einem Netzwerk mit mehreren Switches können Broadcast Storms auch durch Broadcasts über einfache Verbindungen angestoßen werden. Erhalte ein Switch ein an eine Broadcast oder Multicast MAC-Adresse gerichtetes Paket oder eine ihm unbekannte MAC-Adresse, flute er alle Ports außer den des Senders mit einem Broadcast. Wird über mehrere Switches ein Loop gebildet, broadcasten sie im Kreis. Während bei Routing Loops nur beteiligte Knoten belastet werden, erreichen die Broadcasts auch andere Knoten.   
Referenz: https://www.catchpoint.com/network-admin-guide/switching-loops

BATMAN advanced hat Loop Protection im Header jedes Payloads, welche beim Zwischenschalten einer Bridge jedoch verloren gehe. Vom Fall falscher Verkabelung abgesehen, ist die Entstehung von Loops ein logisches Problem. Beiträge im Internet empfehlen verschiedene Tools, um Netzwerktopologien zu analysieren und Loops zu entdecken, sowie Schutzmechanismen. catchpoint nennt das Spanning Tree Protocol (STP) den Standard, um Switch Loops zu verhindern. Üblicherweiser werde jeder Switch so konfiguriert, dieses Protokoll zu verwenden.
BATMAN advanced implementiert aber seine eigene Lösung, da STP nicht über link qualities Bescheid wisse und sich nicht eigne. STP identifiziert redundante Wege über die Netzwerktopologie, welche sich jedoch bei einem Mesh-Netzwerk stetig ändern kann. Bridge Loop Avoidance lässt sich für eine bestehende `bridge` mit `batctl bl 1` aktivieren.
Referenzen: https://www.open-mesh.org/projects/batman-adv/wiki/Bridge-loop-avoidance

BATMAN's Bridge Loop Avoidance funktioniert ebenfalls über einen Header und sieht vor, dass Clients am Mesh von sogenannte Backbone Gateways für sich beansprucht werden. Diese meinen Mesh-Knoten, die ebenfalls mit einem LAN verbunden sind. Jeder Client kann nur von einem Backbone Gateway beansprucht werden, welcher sich für ihre Broadcasts vorantwortlicht.
Genauere Informationen zum Konzept und eine Versuchsreihe: https://www.open-mesh.org/projects/batman-adv/wiki/Bridge-loop-avoidance-II

### Erste Labortests
Im Labor des Instituts für Elektromobilität wurden zwei NUCs mit den Skripten und in verschiedenen WLAN-Netzen getestet, um erfolgreich ein erstes Mesh-Netzwerk mit batman-adv einzurichten:

Zuerst wurde auf beiden `batman-adv_setup1.bash` ausgeführt, während sie mit dem Gastnetzwerk der Hochschule verbunden waren. Wie zuvor warf der Befehl `sudo iw dev wlp0s20f3 ibss join my-mesh-network` die Meldung `command failed: Operation not supported (-95)` `RTNETLINK answers: Operation not permitted` (behoben mit `sudo` vor `ip link set up dev bat0`), führte aber die darauffolgenden Befehle aus, sodass das Netzwerkinterface `bat0` eingerichtet und IP-Adressen vergeben wurden. Ihre IP-Adressen lauteten `169.254.15.146` und `169.254.15.142`, ihre MAC-Adressen `30:0025:52:f7:10` und `04:ea:56:89:84:92`. Das Skript istterminierte nach der IP-Ausgabe von avahi nicht, weswegen jeweils in einem weiteren Terminal mit `batctl ping <MAC-Adresse>` die Erreichbarkeit der NUCs getestet wurde. Diese konnten sich nicht finden. Das selbe Ergebnis wurde mit `batman-adv_setup2.bash` erreicht, jedoch ohne die Fehlerausgabe des `join`-Befehls.

Der Betreuer vermutete, dass die Firewall der Hochschule die Pings blockieren könnte. Daher wurden die Tests im ungeschützten Netzwerk des Instituts wiederholt. Diesmal terminierten beide Skripte und gaben eine PID für den BATMAN Daemon aus.

Da unser Mesh-netzwerk ohne Infrastruktur bestehen soll, wurden die Tests ohne Verbindung zu einem WLAN-Netz wiederholt. Das Ergebnis war das selbe wie beim ersten Versuch.


Zwischenablage
neighbor/originator tables: https://www.open-mesh.org/projects/batman-adv/wiki/Understand-your-batman-adv-network
https://www.kernel.org/doc/Documentation/networking/batman-adv.txt