# ACE - Automated Camera Equipment

Ein Modellbau-Projekt einer an einem Seil befestigten Drohne, die autonom einem Ball folgt und das Sichtfeld der Drohne in Echtzeit auf einen Bildschirm überträgt.

## Projektbeschreibung

Dieses Projekt kombiniert Robotik, Computer Vision und Embedded Systems in einem kompakten Demonstrator. Die Drohne bewegt sich entlang eines gespannten Seils und verfolgt dabei einen Ball automatisiert. Die Bildverarbeitung sowie die Videoausgabe werden von einem Raspberry Pi (RPI) übernommen.

Ziel des Projekts ist die Entwicklung eines intelligenten Kamerasystems, das Objekte erkennen, verfolgen und die gewonnenen Bilddaten auf einem externen Bildschirm darstellen kann.

## Funktionen

- Automatische Ballerkennung
- Echtzeit-Objektverfolgung
- Live-Videoübertragung auf einen Bildschirm
- Bildverarbeitung mit Raspberry Pi
- Modellbau-Drohne mit Seilführung
- Echtzeitsteuerung und Bewegungsanpassung

## Hardware

### Komponenten

- Raspberry Pi
- Kameramodul (Raspberry Pi Camera oder USB-Kamera)
- Modellbau-Drohne
- Seil- bzw. Schienensystem
- Motorsteuerung
- Bildschirm oder Monitor
- Stromversorgung

## Software

### Verwendete Technologien

- Raspberry PI OS
- Programmiersprache C

## Projektstruktur

```text
.
├── src/
│   ├── tracking/
│   ├── drone_control/
│   └── streaming/
├── docs/
├── images/
├── requirements.txt
└── README.md
```

## Figurenfahrt ohne Kamera (`ace_figure`)

Eigenständiges Programm, das ohne Kamera und ohne OpenCV auskommt. Die Plattform
wird von Hand in die Mitte des Ankerfelds gestellt, dort setzt das Programm
seinen Nullpunkt, und danach fährt es ein X: vier Speichen Mitte → Spitze →
Mitte. Gedacht zum Einmessen der Mechanik, bevor die Bilderkennung dazukommt.

```sh
./ace_figure --plan-only            # nur rechnen, nichts bewegen
./ace_figure --size 120,120         # X mit 120 mm Speichenlänge fahren
./ace_figure --dry-run --wait 0     # Phasenfolge ausgeben, GPIO nicht anfassen
./ace_figure --mass-g 250 --loops 3 # Winden gegen 250 g prüfen, dreimal fahren
```

### Warum die dritte Dimension mitgerechnet wird

Die Seillänge ist die räumliche Strecke Anker → Plattform, also
`sqrt(dx² + dy² + h²)` — sie hängt nicht linear an x und y. Lässt man die vier
Winden stur von A nach B durchlaufen, passen ihre Längen unterwegs zu keinem
gemeinsamen Punkt im Raum mehr: jedes Seil fordert eine andere Höhe. Zwei Seile
werden lose, die Plattform sackt ab und pendelt.

Deshalb wird jede Teilfahrt in Stücke von `ACE_SEGMENT_MM` zerlegt und an jedem
Wegpunkt aus der vollen 3D-Formel neu geplant. `ace_figure` rechnet beide
Varianten vorher durch und stellt sie gegenüber — bei der Standardgeometrie:

| Fahrt Mitte → Spitze (170 mm) | quer zur Geraden | Höhenwiderspruch |
| ----------------------------- | ---------------- | ---------------- |
| naiv, ein Stück               | 0,46 mm          | **16,5 mm**      |
| geplant, 34 Stücke à 5 mm     | 0,007 mm         | 0,03 mm          |

Während der Fahrt wird derselbe Wert laufend aus den Schrittzählern gebildet und
gewarnt, sobald er `ACE_MAX_HEIGHT_SPREAD_MM` übersteigt.

### Statik

Aus dem Kräftegleichgewicht am Massepunkt folgt, dass die von den vier Seilen
getragenen Gewichtsanteile die baryzentrischen Koordinaten der Plattform im
Ankerrechteck sind. Der Zug im Seil ist um `l/h` größer als der getragene
Anteil, weil die Seile flach liegen. `ace_figure` gibt beides je Wegpunkt aus
und warnt, wenn ein Seil unter `ACE_MIN_CABLE_TENSION` fällt und damit lose
wird — dann beschreibt sein Schrittzähler die Lage nicht mehr.

## Funktionsweise

1. Die Kamera erfasst kontinuierlich das Sichtfeld.
2. Der Raspberry Pi analysiert die Videodaten.
3. Der Ball wird erkannt und lokalisiert.
4. Die Drohne bewegt sich entlang des Seils, um dem Ball zu folgen.
5. Das Live-Bild wird auf einem angeschlossenen Bildschirm angezeigt.

## PlantUML
Zur Veranschaulichung des Programmcodes wird die Funktionsweise der Unified Modelling Language (UML) verwendet.