# senseBox-loudness
Dieses Repository enthält verschiedene Programme zur digitalen Lärmmessung mit den verschiedenen senseBox MCUs (MCU, MCU S2 und S3).
Die Projekte dienen dazu, Umgebungsgeräusche mit MEMS-Mikrofonen zu erfassen, daraus (A-bewertete) Schalldruckpegel (dB(A)) zu berechnen und die Werte in unterschiedlichen Formaten auszugeben oder weiterzuverarbeiten.

Die enthaltenen Unterordner zeigen verschiedene Ansätze, wie die Messung und Darstellung erfolgen kann, entweder direkt über die senseBox oder unter Einbindung eines Teensy-Boards als Messmodul.

senseBox-Loudness  
├─ direct-connection/  
│  ├─ dBA-RGB-matrix/  
│  ├─ i2s-slm/  
│  ├─ oled_dB_meter/  
├─ with-teensy/  
│  ├─ read-dba.ino  
│  ├─ read-dba-2nd.ino  


## direct-connection/
Beinhaltet Projekte, bei denen die senseBox MCU das Audiosignal direkt vom Mikrofon verarbeitet (also ohne ein separates Messmodul wie den Teensy).
Die Unterordner enthalten verschiedene Varianten zur Berechnung und Darstellung der Lautstärkewerte (dB / dB(A)):

- i2s-slm/ – Direkter Zugriff auf das Mikrofonsignal über I²S und Berechnung des dB(A)-Werts direkt auf der senseBox.
Basierend auf dem Projekt [esp32-i2s-slm](https://github.com/ikostoski/esp32-i2s-slm).

- dBA-RGB-matrix/ – Darstellung der berechneten dB(A)-Werte auf einer RGB-LED-Matrix.

- oled_dB_meter/ – Kombiniert die Anzeige der A-bewerteten dB-Werte eines DFRobot-SLM-Sensors und der unbewerteten dB-Werte eines angeschlossenen Mikrofons auf einem OLED-Display.

Jeder Unterordner enthält eine eigene README-Datei mit Hinweisen zum Aufbau, zur Funktionsweise und zur Verwendung.

## with-teensy/
Beinhaltet Projekte, bei denen die Lärmmessung auf einem Teensy 4.0 basierend aus dem [DNMS Projekt](https://github.com/hbitter/DNMS) erfolgt.
Der Teensy übernimmt dabei die kontinuierliche Audiodatenerfassung und Berechnung der dB(A)-Werte.
Die senseBox MCU S2 dient als Master-Gerät, das die Messwerte über I²C abfragt und weiterleitet.

- read-dba.ino – Liest Messwerte (LAeq, LAmax, LAmin) für ein definiertes Messintervall aus

- read-dba-2nd.ino – Variante mit zwei parallel laufenden Messintervallen (z. B. kurz und lang)

Weitere Details zum Aufbau und Ablauf dieses Systems sind im Unterordner [with-teensy/README.md](with-teensy/README.md)