# Tutorial 2: Kamera-Kalibrierung mit einer ChArUco-Tafel

Damit die Kamera später **messen** kann („der Greifer steht 4 mm links vom Würfel“), muss man
wissen, wie sie Punkte aus der Welt auf Pixel abbildet. Genau das bestimmt die Kalibrierung.
In diesem Tutorial schreibst du den Kern davon in `src/vision/calibration.cpp`.

**Du lernst:** das Lochkamera-Modell und Linsenverzerrung, ChArUco-Tafeln, `cv::calibrateCamera`,
Dateien lesen/schreiben mit `cv::FileStorage`, und warum gute Kalibrierbilder wichtiger sind als
guter Code.

| Lektion | Inhalt | Tests, die grün werden |
|---|---|---|
| 1 | Die Tafel beschreiben | `BoardHasOneCornerPerInnerChessboardCrossing` |
| 2 | Ecken in einem Bild finden | `AddViewAcceptsAnImageOfTheBoard`, `AddViewRejectsAnImageWithoutBoard`, `AddViewRejectsImagesOfDifferentSize` |
| 3 | Kalibrieren | `CalibrateNeedsEnoughViews`, `RecoversTheKnownCamera` |
| 4 | Ergebnis speichern und laden | `SaveAndLoadRoundTrip`, `LoadThrowsForMissingFile` |
| 5 | Praxis: echte Kamera kalibrieren | – |

---

## Arbeitsablauf

Alles läuft direkt unter **Windows** (OpenCV ist installiert). In PowerShell im Repo-Ordner:

```powershell
powershell -ExecutionPolicy Bypass -File tools\test.ps1 "Calibration.*"
```

Das Skript baut das Projekt und führt die Tests aus. Die Tests brauchen **keine Kamera**: Sie
rendern die Tafel in künstliche Bilder, aufgenommen von einer idealen Kamera mit bekannten Werten
(`fx = fy = 600`, `cx = 320`, `cy = 240`). Deine Kalibrierung muss diese Werte wiederfinden.

Weil alle anderen Funktionen die Tafel aus Lektion 1 brauchen, scheitern am Anfang alle Tests an
`makeCharucoBoard: not implemented yet`. Das ändert sich nach Lektion 1.

---

## Hintergrund: Was wird kalibriert?

Eine Kamera bildet einen Punkt im Raum `(X, Y, Z)` (in Kamerakoordinaten, Meter) so auf ein
Pixel `(u, v)` ab (**Lochkamera-Modell**):

```
u = fx · X/Z + cx
v = fy · Y/Z + cy
```

- `fx`, `fy`: **Brennweite in Pixeln**. Groß = Teleobjektiv, klein = Weitwinkel.
- `cx`, `cy`: **Bildhauptpunkt**, wo die optische Achse den Sensor trifft (ungefähr Bildmitte,
  aber nie genau).
- Dazu kommt die **Verzerrung** der Linse: Gerade Linien erscheinen am Bildrand gebogen.
  OpenCV beschreibt sie mit 5 Zahlen `k1 k2 p1 p2 k3` (radial `k`, tangential `p`).

`fx fy cx cy` stehen in der **Kameramatrix** `K = [fx 0 cx; 0 fy cy; 0 0 1]`. Zusammen mit den
Verzerrungskoeffizienten sind das die **intrinsischen Parameter**: Sie hängen nur von Kamera,
Objektiv, Fokus und Auflösung ab, nicht davon, wo die Kamera steht.

**Die Idee der Kalibrierung:** Man fotografiert ein Objekt, dessen Geometrie man genau kennt,
aus vielen Blickwinkeln. Für jedes Bild kennt man dann Paare „3D-Punkt auf der Tafel ↔ Pixel im
Bild“. `cv::calibrateCamera` sucht die Kameraparameter (und die Lage der Tafel in jedem Bild), die
diese Paare am besten erklären.

---

## Lektion 1: Die Tafel beschreiben

**Ziel:** `makeCharucoBoard(spec)` liefert eine `cv::aruco::CharucoBoard` passend zur Beschreibung
`CharucoBoardSpec` in `include/armvision/vision/calibration.hpp`.

**Hintergrund.** Eine **ChArUco-Tafel** ist ein Schachbrett, in dessen weißen Feldern ArUco-Marker
sitzen. Gemessen werden die Ecken des Schachbretts (sehr genau, auf Bruchteile eines Pixels).
Die Marker sagen, **welche** Ecke man gerade sieht. Vorteil gegenüber einem reinen Schachbrett:
Die Tafel muss nicht vollständig im Bild sein, auch Teilansichten am Bildrand sind nutzbar,
und gerade die braucht man für die Verzerrung.

Eine Tafel mit 7 × 5 Feldern hat 6 × 4 = 24 innere Ecken. Die Tafel benutzt das Wörterbuch
`DICT_5X5_100`, nicht `DICT_4X4_50` wie die Marker am Arm, damit sich beide nie in die Quere
kommen.

**Aufgabe:** TODO 1 in `src/vision/calibration.cpp`.

<details><summary>Hinweis</summary>

```cpp
return cv::aruco::CharucoBoard(cv::Size(spec.squares_x, spec.squares_y), spec.square_length,
                               spec.marker_length, /* Wörterbuch */);
```
</details>

**Zum Nachdenken:** Der Konstruktor von `CharucoCalibrator` steht schon da:
`: board_(makeCharucoBoard(spec)), detector_(board_)`. Warum ist die Reihenfolge der Member in
der Klasse (`board_` vor `detector_`) hier wichtig?

---

## Lektion 2: Ecken in einem Bild finden

**Ziel:** `addView(image)` sucht die Tafel im Bild und speichert die gefundenen Ecken als Paare
„3D-Punkt auf der Tafel ↔ Pixel“. Zu wenige Ecken: `false`, nichts wird gespeichert.

**Hintergrund.**
- `detector_.detectBoard(image, corners, ids)` findet die Schachbrett-Ecken: `corners` sind die
  Pixelpositionen, `ids` die Nummern der Ecken (0 bis 23).
- `board_.matchImagePoints(corners, ids, object_points, image_points)` schlägt für jede Ecke die
  3D-Position auf der Tafel nach (Meter, `z = 0`, weil die Tafel flach ist) und liefert beide
  Listen in passender Reihenfolge.
- Eine Kalibrierung gilt immer für **eine** Auflösung. Bilder unterschiedlicher Größe zu mischen
  ist ein Fehler, deshalb `std::invalid_argument`.

**Aufgabe:** TODO 2. Die Schritte stehen im Kommentar.

<details><summary>Hinweis 1: Typen</summary>

```cpp
std::vector<cv::Point2f> corners;
std::vector<int> ids;
detector_.detectBoard(image, corners, ids);

std::vector<cv::Point3f> object_points;
std::vector<cv::Point2f> image_points;
board_.matchImagePoints(corners, ids, object_points, image_points);
```
</details>

<details><summary>Hinweis 2: Speichern ohne Kopie</summary>

`object_points_.push_back(std::move(object_points));` verschiebt den Vektor in die Liste, statt
ihn zu kopieren. Danach ist `object_points` leer, das ist in Ordnung, weil die Funktion endet.
</details>

**Zum Nachdenken:** Warum lohnt sich `kMinCornersPerView = 8`? (Tipp: Wie viele Punkte braucht
man mindestens, um die Lage einer Ebene im Raum zu bestimmen, und was passiert bei Punkten,
die fast auf einer Linie liegen?)

---

## Lektion 3: Kalibrieren

**Ziel:** `calibrate()` liefert die Kameraparameter und den Fehler.

**Hintergrund.** `cv::calibrateCamera` bekommt alle gesammelten Punktpaare und die Bildgröße und
gibt die Kameramatrix, die Verzerrung, die Lage der Tafel in jedem Bild (`rvecs`, `tvecs`)
und den **RMS-Reprojektionsfehler** zurück: Projiziert man die 3D-Ecken mit den gefundenen
Parametern zurück ins Bild, wie weit liegen sie im Mittel neben den gemessenen Ecken (in Pixeln)?

- **< 0,5 px**: gut
- **0,5 bis 1 px**: brauchbar
- **> 1 px**: etwas stimmt nicht (unscharfe Bilder, Tafel nicht flach, falsche Ecken)

**Aufgabe:** TODO 3.

<details><summary>Hinweis</summary>

```cpp
CameraIntrinsics result;
result.image_size = image_size_;
std::vector<cv::Mat> rvecs, tvecs;
result.rms_reprojection_error = cv::calibrateCamera(
    object_points_, image_points_, image_size_, result.camera_matrix, result.dist_coeffs,
    rvecs, tvecs);
```
</details>

**Was der Test zeigt.** Selbst mit perfekten künstlichen Bildern ist der Bildhauptpunkt
`(cx, cy)` der unsicherste Wert (im Test etwa 3 px daneben). Mit nur leicht geneigten Ansichten
lag er sogar 7 px daneben: Erst stark geneigte Ansichten (±35°) trennen „Kamera schaut schräg“
von „Hauptpunkt ist verschoben“. **Vielfalt der Ansichten ist wichtiger als ihre Anzahl.**

**Zum Nachdenken:** Wenn deine gedruckte Tafel in Wirklichkeit Felder von 28 mm statt 30 mm hat:
Welche Ergebnisse werden falsch, `fx`/`cx` oder die Abstände `tvecs`? (Tipp: Eine doppelt so
große Tafel in doppelter Entfernung sieht im Bild genau gleich aus.)

---

## Lektion 4: Ergebnis speichern und laden

**Ziel:** `save()` schreibt die Kalibrierung in eine YAML-Datei, `load()` liest sie zurück.

**Hintergrund.** Die Kalibrierung macht man einmal und benutzt sie dann in jedem Programm, das
die Kamera verwendet. `cv::FileStorage` schreibt Zahlen und `cv::Mat` als YAML (oder XML, je nach
Dateiendung):

```yaml
image_width: 640
camera_matrix: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 600., 0., 320., 0., 600., 240., 0., 0., 1. ]
```

**Aufgabe:** TODO 4, beide Funktionen.

<details><summary>Hinweis</summary>

Schreiben: `fs << "image_width" << image_size.width << "camera_matrix" << camera_matrix;`
(beliebig verkettbar). Lesen: `fs["camera_matrix"] >> c.camera_matrix;`. Fehlt ein Eintrag in der
Datei, bleibt die Matrix leer, `c.camera_matrix.empty()` ist dann `true`.
</details>

**Zum Nachdenken:** Warum wirft `load()` lieber eine Exception, als eine halb gefüllte
Kalibrierung zurückzugeben?

Jetzt sollten alle 8 `Calibration`-Tests grün sein.

---

## Lektion 5: Praxis mit einer echten Kamera

Zum Üben reicht die Laptop-Kamera. Die richtige Kalibrierung machen wir später mit der
Pi-Kamera, sobald sie fest montiert und ihr Fokus fixiert ist: **Eine Kalibrierung gilt nur
für genau diese Kamera, Auflösung und Fokuseinstellung.**

**1. Tafel drucken**

```powershell
.\build\local\apps\Debug\make_charuco_board.exe
```

`charuco_board.png` auf A4 quer mit **100 % / „Tatsächliche Größe“** drucken (nicht „An Seite
anpassen“). Auf etwas **Flaches und Steifes** kleben (Karton, Holzbrett): Eine gewellte Tafel
verfälscht alles. Ein Feld mit dem Lineal nachmessen (soll 30 mm sein).

**2. Kalibrieren**

```powershell
.\build\local\apps\Debug\calibrate_camera.exe live 0 --square 0.030
```

Grüne Punkte zeigen die erkannten Ecken. Mit **Leertaste** ein Bild übernehmen, mit **c**
kalibrieren und `camera.yaml` speichern, mit **u** das entzerrte Bild ansehen.

**3. Tipps für gute Bilder**

- **15 bis 25 Bilder**, die Tafel jedes Mal anders halten.
- **Neigen**: bis etwa 45° in alle Richtungen (siehe Lektion 3).
- **Bildecken abdecken**: Die Tafel muss auch an den Rändern und in den Ecken des Bildes
  auftauchen, dort ist die Verzerrung am stärksten. Fehlen solche Bilder, werden die
  Verzerrungswerte unsinnig (Beispiel aus einem Test mit Bildern, in denen die Tafel nur die
  Mitte füllte: `k2 = 6`, `k3 = −68` bei einer Kamera *ohne* Verzerrung, obwohl der RMS mit
  0,25 px gut aussah).
- **Ruhig halten**: Bewegungsunschärfe verschlechtert die Ecken. Gutes Licht hilft.
- **Prüfen** mit **u**: Gerade Kanten (Tischkante, Türrahmen) müssen im entzerrten Bild auch
  am Rand gerade sein.

**Erweiterung (optional):** Bei normalen Objektiven ist `k3` selten nötig und neigt zum
„Wegschwimmen“. `cv::calibrateCamera` hat einen Parameter `flags`; mit `cv::CALIB_FIX_K3` wird
`k3` auf 0 festgehalten. Baue eine Option dafür ein (z.B. ein Feld in `CharucoBoardSpec` oder
ein Parameter von `calibrate()`) und vergleiche die Ergebnisse.

---

## Wie geht es weiter?

Mit `camera.yaml` kann die Kamera messen: In M3 bestimmt `cv::solvePnP` aus einem ArUco-Marker
auf dem Tisch, wo die Kamera relativ zum Arm steht, und aus einem Marker am Greifer, wo die
Greiferspitze wirklich ist.
