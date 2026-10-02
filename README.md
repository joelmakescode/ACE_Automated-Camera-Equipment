# ACE - Automated Camera Equipment

Ein Modellbau-Projekt: Eine Kameraplattform hängt an vier Seilen über einem
Spielfeld (dem „Stadion“), folgt autonom einem farbigen Ball und überträgt ihr
Sichtfeld in Echtzeit in den Browser.

📄 **Ausführliche Anleitung:** [`docs/ACE_Anleitung.docx`](docs/ACE_Anleitung.docx)
— Aufbau, Inbetriebnahme, alle Programmoptionen und eine vollständige
Funktionsreferenz.

## Projektbeschreibung

Vier Schrittmotor-Winden in den Ecken des Stadions verändern die Seillängen und
bewegen die Plattform dadurch frei in der Fläche. Ein Raspberry Pi wertet das
Bild der nach unten blickenden Kamera aus, erkennt den Ball und fährt die
Plattform so nach, dass der Ball in der Bildmitte bleibt.

Das Projekt kombiniert Robotik, Computer Vision und Embedded Systems in einem
kompakten Demonstrator.

## Funktionen

- Ballerkennung über einen HSV-Farbbereich (OpenCV, Standard: Rot)
- Echtzeit-Verfolgung mit selbstlernendem Bildmodell (Kamerawinkel und Maßstab)
- 3D-Seilkinematik: Fahrten werden in 5-mm-Stücke zerlegt und je Stück neu geplant
- Höhe halten: die tatsächliche Hängehöhe wird aus den Schrittzählern mitgeführt
- Bodenlinien (blau/grün) als absolute Referenz für Winkel, Lage und Maßstab
- Suchfahrt in Schlangenlinie, wenn der Ball verloren geht
- Erkennung und Behandlung einer durchrutschenden Winde
- Livestream (MJPEG) mit Bedienseite im Browser: Handsteuerung, Start/Stopp, Neu lernen
- Diagnose-Werkzeuge für Motoren, Farberkennung, Mechanik und Statik

## Hardware

| Komponente      | Ausführung                                                   |
| --------------- | ------------------------------------------------------------ |
| Rechner         | Raspberry Pi mit Raspberry Pi OS (Bookworm)                  |
| Kamera          | Raspberry-Pi-Kameramodul, angesteuert über `rpicam-vid`      |
| Winden          | 4 × Schrittmotor 28BYJ-48 mit Treiber ULN2003, 5 V           |
| Seiltrommeln    | Wickeldurchmesser 20 mm (je Winde einstellbar)               |
| Bodenlinien     | 2 matte Klebebänder, blau und grün, Strichteilung 50 mm      |
| Objekt          | roter Ball                                                   |

Ankerfeld 640 × 440 mm, Plattform 150 mm unter den Ankern, Kamera 300 mm über
dem Boden. Alle Maße und Grenzwerte stehen in
[`src/drone_control/geometry.h`](src/drone_control/geometry.h), die
GPIO-Belegung in [`src/drone_control/pins.h`](src/drone_control/pins.h):

| Winde | Lage         | IN1 | IN2 | IN3 | IN4 |
| ----- | ------------ | --- | --- | --- | --- |
| 0     | unten links  | 17  | 18  | 27  | 22  |
| 1     | oben links   | 23  | 24  | 25  | 4   |
| 2     | oben rechts  | 5   | 6   | 12  | 13  |
| 3     | unten rechts | 16  | 19  | 20  | 21  |

## Software

- Programmiersprachen C (C11) und C++ (C++17)
- OpenCV für die Bildverarbeitung
- CMake als Build-System
- Raspberry Pi OS, GPIO über `/dev/gpiochip0`

## Projektstruktur

```text
.
├── CMakeLists.txt
├── requirements.txt          apt-Pakete für den Pi
├── diagram.puml              UML-Diagramm
├── docs/
│   └── ACE_Anleitung.docx    ausführliche Anleitung
└── src/
    ├── tracking/             Kamera, Ball-/Linienerkennung, Stream-Server
    ├── drone_control/        Antrieb, Kinematik, Bahnplanung, Regelung
    └── streaming/            (noch leer)
```

## Installation und Build

```sh
sudo apt install build-essential cmake libopencv-dev pkg-config v4l-utils
cmake -S . -B build
cmake --build build
```

Standardmäßig wird mit höchstens zwei parallelen Übersetzungen gebaut, weil die
OpenCV-Übersetzung rund 1 GB RAM braucht. Mit mehr Speicher:
`cmake --build build --parallel 4`.

## Programme

| Programm         | Zweck                                                   | Kamera |
| ---------------- | ------------------------------------------------------- | ------ |
| `ace_motor_test` | Motoren und Verdrahtung prüfen, Wickeldurchmesser messen | nein   |
| `ace_tracking`   | Ballerkennung und Farbbereich testen (`--probe`, `--mask`) | ja   |
| `ace_figure`     | X-Figur ohne Kamera, Mechanik einmessen                 | nein   |
| `ace_drone`      | Kalibrieren, Positionen anfahren                        | ja     |
| `ace_track`      | vollständige Ballverfolgung (Hauptprogramm)             | ja     |

Alle Programme außer `ace_tracking` werden nur unter Linux gebaut. Empfohlene
Reihenfolge bei der Inbetriebnahme:

```sh
./ace_motor_test --pattern all       # jede Winde einzeln prüfen
./ace_tracking --probe               # Farbbereich des Balls messen
./ace_figure --plan-only             # Mechanik durchrechnen, dann fahren
./ace_track --stream=8080            # verfolgen, Livebild unter http://<pi>:8080/
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

### Die Höhe ist keine Konstante

Die Plattform soll sich nicht senkrecht bewegen, aber sie *hängt* — ihre Höhe
folgt allein aus den vier Seillängen. Bei dieser Geometrie sind **1 mm Seil
rund 2,8 mm Höhe** (`dL/dz = 0,361`), die Hebelwirkung ist also groß.

Für die Lage in der Fläche ist das folgenlos: in der Paarformel
`x = (ℓ₃² − ℓ₂²)/(2·span)` kürzt sich z **exakt** heraus, x und y stimmen auf
jeder Höhe. Für die Planung ist es das nicht. Wird mit der Nennhöhe geplant,
während die Plattform tiefer hängt, sind die kommandierten Seile ungleich zu
kurz — jede Fahrt in der Fläche enthält dann heimlich einen Hub, und der fällt
je Winde verschieden aus.

`kin_plan` führt die Höhe deshalb aus dem laufenden Zählerstand mit und plant
auf der **gemessenen** statt der unterstellten Höhe:

| Ausgangslage, Fahrt nach (120,120) | feste Nennhöhe | Höhe halten |
| ---------------------------------- | -------------- | ----------- |
| Plattform hängt auf 156 statt 150 mm | **+6,01 mm Hub** | −0,006 mm |
| nach `--tension 1` (Plattform auf 147,2 mm) | **+2,80 mm Hub** | +0,05 mm |

Das xy-Ziel wird in allen Fällen exakt getroffen. Abschalten mit
`kin_set_hold_height(0)`; Plausibilitätsschranken stehen in `geometry.h`
(`ACE_HOLD_HEIGHT_MIN_MM`/`MAX`), ab `ACE_HEIGHT_DRIFT_WARN_MM` Abweichung vom
Nennmaß wird gewarnt.

## Objektverfolgung mit Kamera (`ace_track`)

Hält ein farbiges Objekt in der Bildmitte. Baut auf demselben 3D-Kern auf wie
`ace_figure` und ergänzt ihn um Kamera und Bildmodell.

```sh
./ace_track                          # Lernphase, dann verfolgen
./ace_track --stream=8080            # mit Livebild im Browser
./ace_track --weak-motor 1 --preload 1.2
./ace_track --no-learn --gain 0.4    # ohne Einmessen, extra gedämpft
```

### Kamerawinkel, der sich ändert

Weil oben ein HDMI-Kabel zieht, ist die Zuordnung Bild → Fläche keine feste
Vorzeichenfrage. Sie wird als **Drehstreckung** modelliert — eine komplexe Zahl
`a = Maßstab · exp(i·Kamerawinkel)`, also zwei Parameter statt der vier einer
allgemeinen 2×2-Matrix, und damit deutlich robuster zu schätzen. Der nötige
Fahrweg ist schlicht `E / a`.

Eine Lernphase mit vier festen Probefahrten bestimmt `a` von Grund auf; danach
wird jede Fahrt als weitere Messung eingearbeitet, mit Vergessensfaktor, damit
eine langsame Drehung ankommt. Ausreißer (bewegtes Objekt, rutschende Winde)
werden abgewiesen.

Der Regelkreis ist stabil, solange der geschätzte Kamerawinkel um weniger als
`acos(gain/2)` danebenliegt — bei Gain 1,0 sind das 60°, bei den
voreingestellten 0,6 schon **72,5°**. Klingt der Bildfehler zwei Züge lang
nicht ab, hat sich die Kamera weiter gedreht als der Regler einfangen kann;
dann wird das Bildmodell automatisch neu gelernt.

### Winde, die durchrutscht

Vier Seile bei zwei Freiheitsgraden sind einfach redundant: jede Achse wird aus
zwei unabhängigen Ankerpaaren bestimmt. Nimmt man Winde 1 aus der Wertung,
bleibt x über Paar (3,2) und y über Paar (3,0) — die Lage ist **weiter
vollständig bestimmt**.

Das dreht den Defekt um. Statt den Schlupf unsichtbar in die Positionsschätzung
einzuschleppen, wird er **messbar**: die Länge, die die Lage aus den gesunden
Winden für Winde 1 fordert, gegen die Länge, die ihr Schrittzähler behauptet.
Das braucht keine Kamera. Ab `ACE_SLIP_WARN_MM` wird gewarnt, ab
`ACE_SLIP_RECOVER_MM` wird nur diese eine Winde neu referenziert
(`kin_reset_motor`), damit die Planung wieder die richtige Seillänge
kommandiert. Zusätzlich hält eine Vorspannung (`--preload`) ihr Seil straff.

Ein Vorbehalt: fällt Winde 1 ganz aus, tragen drei Seile nur noch über ihrem
Dreieck. Dessen Kante läuft bei dieser Geometrie **genau durch die Mitte des
Ankerfelds** — die halbe Fläche hängt also an der schwachen Winde.
`figure_margin_without` rechnet den Abstand zu dieser Kante aus, `ace_track`
gibt ihn beim Start aus.

### Statik

Aus dem Kräftegleichgewicht am Massepunkt folgt, dass die von den vier Seilen
getragenen Gewichtsanteile die baryzentrischen Koordinaten der Plattform im
Ankerrechteck sind. Der Zug im Seil ist um `l/h` größer als der getragene
Anteil, weil die Seile flach liegen. `ace_figure` gibt beides je Wegpunkt aus
und warnt, wenn ein Seil unter `ACE_MIN_CABLE_TENSION` fällt und damit lose
wird — dann beschreibt sein Schrittzähler die Lage nicht mehr.

## Funktionsweise

1. Die Kamera an der Plattform erfasst kontinuierlich das Spielfeld.
2. Der Raspberry Pi sucht im Bild den Ball (Farbmaske, größte Fläche).
3. Das Bildmodell rechnet die Abweichung zur Bildmitte in einen Fahrweg in mm um.
4. Die Kinematik bestimmt daraus die vier Seillängen bzw. Halbschritte der Winden.
5. Die Winden fahren synchron; die Bewegung wird nachgemessen und das Modell nachgeführt.
6. Das Live-Bild wird im Browser angezeigt.

## PlantUML

Zur Veranschaulichung des Programmcodes wird die Unified Modeling Language (UML)
verwendet. Das Diagramm liegt in [`diagram.puml`](diagram.puml).
