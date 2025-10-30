# Digitales Lärmmesssystem mit senseBox MCU und DNMS-Sensor

Dieses Projekt beschreibt ein System zur Messung von Umgebungslärm mit einem digitalen Mikrofon-Sensor (DNMS) und einer senseBox MCU S2. Das System misst kontinuierlich den Schallpegel in Dezibel und überträgt die Ergebnisse an die OpenSenseMap (und andere Plattformen) zur Datenerfassung und -auswertung.

Die senseBox MCU S2 die Steuerung des Messablaufs. Sie legt fest, in welchen Zeitabständen der durchschnittliche Schalldruckpegel (LAeq) berechnet und abgefragt wird. Dieses Zeitintervall wird als Messintervall bezeichnet.
Der Teensy arbeitet dabei als Messmodul („Slave“) und misst kontinuierlich den Schallpegel. Er verarbeitet die Audiodaten des Mikrofons fortlaufend, berechnet intern die aktuellen dB(A)-Werte und hält die Ergebnisse bereit.
Sobald die senseBox eine Abfrage über die I²C-Schnittstelle sendet, liefert der Teensy die aktuellen Messwerte (LAeq, LAmax, LAmin) zurück.

Das Sensormodul DNMS (Digital Noise Measurement Sensor) wurde speziell für die dauerhafte Erfassung von Umgebungsgeräuschen entwickelt. Es besteht aus zwei zentralen Komponenten:

- Digitales Mikrofon (IM72D128 Mikrofon)
    Dieses MEMS-Mikrofon nimmt den Schall als digitale Audiodaten mit einer Abtastrate von 44,1 kHz und 16 Bit auf.

- Mikrocontroller (Teensy 4.0)
    Der Teensy empfängt die digitalen Audiodaten über eine I²S-Schnittstelle und berechnet daraus den Schallpegel in Dezibel (dB). Dafür wird das Signal zuerst A-bewertet, das heißt, es wird nach der menschlichen Hörwahrnehmung gefiltert (tiefere und sehr hohe Frequenzen werden abgeschwächt). Anschließend wird aus den gefilterten Werten der effektive Schalldruckpegel berechnet.

Die berechneten Messwerte werden über eine I²C-Schnittstelle an die senseBox MCU S2 übertragen, die sie weiterverarbeitet und an die OpenSenseMap oder andere Server sendet.

Der Soundsensor liefert drei Schallpegel-Messwerte, die jeweils in regelmäßigen Zeitabständen bereitgestellt werden. Diese Messintervall kann vom Benutzer zwischen 1 und 3600 Sekunden eingestellt werden.

- **LAeq** - Äquivalenter Dauerschallpegel (A-bewertet)  
Dies ist der über die Taktzeit gemittelte Schallpegel. Er beschreibt die gesamte Schallenergie, die während dieser Zeit auftritt, und ist der wichtigste Messwert bei der Beurteilung von Umgebungslärm (z. B. Verkehr, Baustellen, Stadtlärm).

- **LAmax** - Höchster Schallpegel innerhalb der Taktzeit  
Er gibt an, wie laut es während des Messzeitraums maximal war. Der Wert wird aus den intern alle 35 ms berechneten Einzelpegeln bestimmt.

- **LAmin** - Niedrigster Schallpegel innerhalb der Taktzeit  
Entsprechend wird hier der leiseste gemessene Wert innerhalb der Taktzeit angegeben (ebenfalls alle 35 ms überprüft).

Aus dem LAeq können die üblichen Mittelwerte wie Stundenmittel, Tagesmittel, Tages- und Nachtmittel oder der EU-Indexwert Lden ("Day-Evening-Night"-Pegel) gebildet werden.


## Flashing the Teensy

Install the Teensy Loader from: https://www.pjrc.com/teensy/loader.html

Download the HEX file from: https://github.com/hbitter/DNMS/tree/master/Firmware/Teensy/Teensy4.0

Add a jumper to connect pins on J1 on the Teensy to allow power through USB connection. 

With the Teensy connected via USB and the Teensy Loader open:  
1. click on "Choose HEX file" and choose the downloaded file
2. then double click button on Teensy to enter Boot Mode.

The program should automatically uploaded to the Teensy Board