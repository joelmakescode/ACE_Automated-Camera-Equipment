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

## Funktionsweise

1. Die Kamera erfasst kontinuierlich das Sichtfeld.
2. Der Raspberry Pi analysiert die Videodaten.
3. Der Ball wird erkannt und lokalisiert.
4. Die Drohne bewegt sich entlang des Seils, um dem Ball zu folgen.
5. Das Live-Bild wird auf einem angeschlossenen Bildschirm angezeigt.
