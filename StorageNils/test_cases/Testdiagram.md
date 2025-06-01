# Testszenarien
<br></br>

<table><tbody><tr><th colspan="2"><h2><a id="Testaufbau_1"></a>Testaufbau 1</h2></th></tr><tr><td><p>NUC-Publisher</p></td><td><p><strong>NUC-Subscriber</strong></p></td></tr><tr><td colspan="2"><p>Komponenten</p></td></tr><tr><td><ul><li>Publisher Node</li><li>Stromversorgung</li><li><em>Antenne</em></li><li><em>Sensor(en)</em></li></ul></td><td><ul><li>Publisher Node</li><li>Stromversorgung</li><li><em>Antenne</em></li></ul></td></tr><tr><td colspan="2"><p>Aufbau</p></td></tr><tr><td><p>Einzelner NUC, steht irgendwo stationär und misst eventuell Daten</p><p>Publisher für diese Messdaten (oder Testdaten)</p><p>Sollte theoretisch keinen Monitor brauchen, weil nur Daten gesendet werden sollen. Kann über den Subscriber getestet werden.</p></td><td><p><strong>Einzelner </strong>NUC, soll neben dem <em>Publisher</em> aufgebaut sein</p><p><strong>Subscriber </strong>für diese (Test-)Daten</p><p>Sollte mit Monitor verbunden sein, um Daten korrekt anzeigen zu können</p></td></tr><tr><td colspan="2"><p>So kann die Verbindung zwischen einem Publisher und Subscriber getestet werden.</p><p>Sind nebeneinander aufgebaut, um keine Verbindungsabbrüche oder schwache Signale beachten zu müssen.</p><p>Es soll noch kein Abbrechen der Verbindung getestet werden.</p><p>Stabile Stromversorgung (Abbruch, s. o.)</p></td></tr></tbody></table>
<br></br>

## **Testaufbau 2**

<table><tbody><tr><th><p>NUC-Publisher 1</p></th><th><p>NUC-Publisher 2</p></th><th><p>NUC-Subscriber</p></th></tr><tr><td colspan="3"><p>Komponenten</p></td></tr><tr><td colspan="2"><ul><li>Publisher Node</li><li>Stromversorgung</li><li><em>Antenne</em></li><li><em>Sensor(en)</em></li></ul></td><td><ul><li>Publisher Node</li><li>Stromversorgung</li><li><em>Antenne</em></li></ul></td></tr><tr><td colspan="3"><p>Aufbau</p></td></tr><tr><td colspan="2"><ul><li>Publisher von Daten</li><li>Kein Anschluss an Monitor</li><li>Stabile Stromversorgung</li><li>Bleibt in der Nähe des Subscribers</li></ul></td><td><ul><li><strong>Einzelner </strong>NUC, soll neben dem <em>Publisher</em> aufgebaut sein</li><li><strong>Subscriber </strong>für diese (Test-)Daten</li><li>Sollte mit Monitor verbunden sein, um Daten korrekt anzeigen zu können</li></ul></td></tr><tr><td colspan="3"><p>Testet, ob mehrere Publisher korrekt mit dem Subscriber interagieren können. Ansonsten gleicher Aufbau wie in <a href="#Testaufbau_1">Test-Aufbau 1</a>.</p><p>Beide Publisher bleiben in Reichweite vom Subscriber</p></td></tr></tbody></table>
<br></br>

## **Testaufbau 3**

<table><tbody><tr><th><p>NUC-Publisher 1</p></th><th><p>NUC-Publisher 2</p></th><th><p>NUC-Subscriber</p></th></tr><tr><td colspan="3"><p>Komponenten</p></td></tr><tr><td colspan="2"><ul><li>Publisher Node</li><li>Stromversorgung</li><li><em>Antenne</em></li><li><em>Sensor(en)</em></li></ul></td><td rowspan="2"><ul><li>Publisher Node</li><li>Stromversorgung</li><li><em>Antenne</em></li></ul></td></tr><tr><td></td><td><ul><li>Langes Stromkabel</li></ul></td></tr><tr><td colspan="3"><p>Aufbau</p></td></tr><tr><td colspan="2"><ul><li>Publisher von Daten</li><li>Kein Anschluss an Monitor</li><li>Stabile Stromversorgung</li></ul></td><td rowspan="2"><ul><li><strong>Einzelner </strong>NUC, soll neben dem <em>Publisher</em> aufgebaut sein</li><li><strong>Subscriber </strong>für diese<br>(Test-)Daten</li><li>Sollte mit Monitor verbunden sein, um Daten korrekt anzeigen zu können</li></ul></td></tr><tr><td><ul><li>Bleibt in der Nähe des Subscribers</li></ul></td><td><ul><li>In größerer Entfernung Aufbauen</li><li>Langes Stromkabel ermöglicht es, NUC zu bewegen</li><li>Alternativ: NUC ausschalten um Verbindungsverlust zu simulieren</li></ul></td></tr><tr><td colspan="3"><p>Testet, ob mehrere Publisher korrekt mit dem Subscriber interagieren können.</p><p>Der Aufbau unterscheidet sich darin, dass getestet werden soll, wie sich das System verhält, wen ein Publisher außerhalb der Reichweite des Subscribers ist (oder sich dorthin bewegt). Zum Beispiel: Baut sich die Verbindung wieder korrekt neu auf, wenn der Publisher in Reichweite kommt?</p><p>Voraussetzung ist, dass wir ein entsprechend Langes Kabel bekommen. Ansonsten werden die Stahlbeton-Wände im Labor wahrscheinlich auch ausreichen um das WLAN abzuschirmen.</p></td></tr></tbody></table>
<br></br>
<br></br>

# Allgemeine Beschreibung

## Testaufbau 2
Zwei Publisher, beide senden durchgehend Daten. Subscriber verarbeitet beide.
![Aufbau 2](Test_01.gif)

## Testaufbau 3
Zwei Publisher, einer stoppt das Senden. Prüfen, ob alles funktioniert.
![Aufbau 2](Test_02.gif)

