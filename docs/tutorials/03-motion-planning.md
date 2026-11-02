# Tutorial 3: Bahnplanung und Sicherheit

Bisher springt der Arm mit voller Servo-Geschwindigkeit direkt zum Ziel, und nichts hält ihn
davon ab, in den Tisch zu fahren. In diesem Tutorial baust du die Schicht zwischen „wohin“ und
„Servo-Befehl“: eine Sicherheitsprüfung und Bahnen aus vielen kleinen Schritten.

**Du lernst:** `enum class` und `switch`, `std::optional` als Ergebnis, Gleitkomma-Rechnen in
Schleifen, **Templates und C++20-Concepts**, Lambdas, und den Unterschied zwischen Bewegungen im
Gelenkraum und im kartesischen Raum (inklusive einer *Singularität*).

Alles ist reine Rechnung, du brauchst keine Hardware.

| Lektion | Inhalt | Datei | Tests |
|---|---|---|---|
| 1 | Sicherheitsprüfung | `src/motion.cpp` | `HomeIsSafe`, `Detects…`, `ViolationsHaveReadableNames` |
| 2 | Kleine Schritte im Gelenkraum | `src/motion.cpp` | `InterpolateJoints…` |
| 3 | Sanftes Anfahren mit Templates | `include/armvision/motion.hpp` | `Smoothstep…`, `Eased…` |
| 4 | Gerade Linien im Raum | `src/motion.cpp` | `StraightLine…` |
| 5 | Praxis mit `arm_cli` | – | – |

```powershell
powershell -ExecutionPolicy Bypass -File tools\test.ps1 "Motion.*"
```

---

## Lektion 1: Sicherheitsprüfung

**Ziel:** `findViolation(q, cfg)` sagt, ob eine Pose gefährlich ist, und wenn ja, warum.

**Hintergrund.** Drei Regeln, in dieser Reihenfolge:

1. Alle Gelenke innerhalb ihrer Grenzen (`withinLimits`).
2. Die Greiferspitze mindestens `cfg.min_height` (1 cm) über dem Tisch (`forwardKinematics`).
3. Der Ellbogen ebenfalls (`elbowPosition`, neu in `kinematics.hpp`).

Das Ergebnis ist ein `std::optional<Violation>`: leer = sicher, sonst *welche* Regel verletzt
ist. Das ist aussagekräftiger als ein `bool`: Der Aufrufer kann eine verständliche Meldung
ausgeben, und Tests können prüfen, dass *die richtige* Regel greift.

`Violation` ist ein `enum class`: Die Werte heißen `Violation::TipBelowTable` usw., sie lassen
sich nicht versehentlich mit Zahlen oder anderen Enums vermischen.

**Aufgabe:** TODO 1a (`toString`) und TODO 1b (`findViolation`).

<details><summary>Hinweis: switch ohne default</summary>

```cpp
switch (v) {
    case Violation::JointLimit: return "a joint is outside its limits";
    // ... die anderen beiden
}
return "unknown violation";
```

Absichtlich **ohne** `default:`: Kommt später ein neuer Wert ins Enum, warnt der Compiler
(`-Wswitch`), dass er hier fehlt. Mit `default:` würde er das stillschweigend schlucken.
Das `return` nach dem `switch` braucht man trotzdem, weil der Compiler nicht weiß, dass ein
Enum nur gültige Werte enthalten kann.
</details>

**Zum Nachdenken:** Warum werden die Gelenkgrenzen *vor* der Höhe geprüft? (Tipp: Was sagt
`forwardKinematics` über eine Pose, die der Arm gar nicht einnehmen kann?)

---

## Lektion 2: Kleine Schritte im Gelenkraum

**Ziel:** `interpolateJoints(from, to, max_step_deg)` zerlegt eine Bewegung in gleich große
Schritte, sodass sich kein Gelenk pro Schritt um mehr als `max_step_deg` dreht.

**Hintergrund.** Ein SG90 kennt keine Geschwindigkeit: Er fährt mit voller Kraft zum
befohlenen Winkel. Ein großer Sprung bedeutet Ruck, Stromspitze (siehe Netzteil-Probleme!) und
Überschwingen. Viele kleine Schritte sind die einfachste Form der Geschwindigkeitsregelung.

Alle Gelenke sollen **gleichzeitig ankommen**. Deshalb bestimmt das Gelenk mit der *größten*
Bewegung die Anzahl der Schritte, und alle Gelenke bekommen dieselben Bruchteile `i / steps`.
Beispiel: Basis +30°, Schulter −30°, max. 10° pro Schritt → 3 Schritte, jedes Gelenk bewegt
sich pro Schritt um ein Drittel seines Weges.

**Aufgabe:** TODO 2.

<details><summary>Hinweis: Rechnen mit Gleitkommazahlen</summary>

- `std::ceil(largest / max_step_deg)` liefert ein `double`, für die Anzahl brauchst du ein
  `std::size_t`: `static_cast<std::size_t>(...)`.
- Berechne jeden Wegpunkt **aus dem Anteil** `i / steps` (`from + a * (to - from)`), statt einen
  Schritt immer wieder aufzuaddieren. Beim Addieren sammeln sich Rundungsfehler an, und der
  letzte Punkt landet nicht exakt auf `to` (der Test prüft das mit `==`).
- `i / steps` mit zwei Ganzzahlen ist eine Ganzzahl-Division (fast immer 0)! Erst beide in
  `double` umwandeln.
</details>

**Zum Nachdenken:** `max_step_deg` von `0` ist sinnlos (unendlich viele Schritte). Warum ist
`if (!(max_step_deg > 0))` sicherer als `if (max_step_deg <= 0)`? (Tipp: NaN.)

---

## Lektion 3: Sanftes Anfahren mit Templates

**Ziel:** Eine Interpolation, die langsam anfährt und sanft bremst, und deren Geschwindigkeitsprofil
frei wählbar ist.

**Hintergrund.** In Lektion 2 ist jeder Schritt gleich groß: Der Arm startet abrupt mit voller
Geschwindigkeit und stoppt abrupt. Besser ist eine **Easing-Funktion** `f(t)`, die den
Fortschritt `t ∈ [0, 1]` auf den Anteil des Weges abbildet:

- `linear(t) = t`: konstante Geschwindigkeit
- `smoothstep(t) = 3t² − 2t³`: Steigung 0 am Anfang und am Ende (Ableitung `6t − 6t²`)

`interpolateEased` soll mit **jeder** solchen Funktion funktionieren: mit `linear`, mit
`smoothstep`, aber auch mit einem Lambda. Dafür ist es ein **Template**:

```cpp
template <Easing E>
std::vector<JointAngles> interpolateEased(const JointAngles& from, const JointAngles& to,
                                          std::size_t steps, E ease);
```

`Easing` ist ein **Concept** (C++20): eine benannte Bedingung an den Typ `E`, hier „aufrufbar
mit einem `double`, Ergebnis in `double` umwandelbar“. Ruft jemand die Funktion mit etwas
Unpassendem auf, gibt es eine kurze, verständliche Fehlermeldung statt hunderter Zeilen
Template-Fehler. Die Tests prüfen das Concept sogar zur Compile-Zeit mit `static_assert`.

**Warum im Header?** Der Compiler erzeugt für jeden Typ `E` eine eigene Version der Funktion,
und dafür muss er ihren Code an der Stelle des Aufrufs sehen. Templates stehen deshalb fast
immer komplett im Header (`include/armvision/motion.hpp`).

**Aufgabe:** TODO 3a (`smoothstep`) und TODO 3b (`interpolateEased`) im Header.

<details><summary>Hinweis</summary>

Der Aufbau ist derselbe wie in Lektion 2, nur dass der Anteil `a` diesmal `ease(t)` ist statt
`t`. `path.reserve(steps)` vorher spart wiederholtes Umkopieren des Vektors.
</details>

**Zum Nachdenken:**
1. Könntest du `interpolateJoints` aus Lektion 2 jetzt mit `interpolateEased(..., linear)`
   schreiben? Was wäre der Vorteil?
2. Man hätte auch `std::function<double(double)> ease` als normalen Parameter nehmen können.
   Was spricht für das Template? (Tipp: Was muss `std::function` intern tun, und was kann der
   Compiler beim Template, das er bei `std::function` nicht kann?)

---

## Lektion 4: Gerade Linien im Raum

**Ziel:** `straightLine(from, target, cfg, step_m)` bewegt die **Greiferspitze** auf einer
geraden Linie zum Ziel.

**Hintergrund.** Interpoliert man die *Gelenkwinkel* (Lektion 2), fährt die Spitze einen Bogen.
Für Greifen braucht man oft gerade Bewegungen, z.B. senkrecht von oben auf ein Objekt. Dafür
teilt man die Linie im Raum in kleine Stücke und löst für jeden Punkt die **inverse Kinematik**.

Zwei Feinheiten:
- **Referenz für die IK:** Für denselben Punkt gibt es oft mehrere Lösungen (Ellbogen oben oder
  unten). `inverseKinematics` nimmt die, die am nächsten an `reference` liegt. Gibst du immer
  den *vorherigen* Wegpunkt als Referenz, bleibt der Arm in derselben Konfiguration und
  springt nicht zwischen den Lösungen hin und her.
- **Jeder Punkt wird geprüft:** Start und Ziel können sicher sein, und die Linie dazwischen
  trotzdem durch den Tisch oder aus dem Arbeitsraum führen.

**Aufgabe:** TODO 4.

**Zum Nachdenken (Singularität):** Probiere nach dem Lösen
`arm_cli --sim line 0.03 0.19 0.12`. Der erste Schritt aus der Home-Position verändert den
Ellbogen um rund 20°, die folgenden nur um 3–7°, obwohl die Spitze jedes Mal gleich weit
fährt. Warum? (Tipp: Wie sieht der Arm in der Home-Position aus, und in welche Richtung kann sich
die Spitze dort überhaupt bewegen?)

---

## Lektion 5: Praxis mit `arm_cli`

`arm_cli` benutzt deinen Code bereits:

- **Jeder Befehl wird geprüft** (`findViolation`), bevor sich etwas bewegt:
  ```powershell
  .\build\local\apps\Debug\arm_cli.exe --sim joints 90 40 90 90
  # refusing to move: the gripper tip is too close to the table
  ```
- **Neuer Befehl `line`:** gerade Linie von der aktuellen Pose zum Ziel (Meter):
  ```powershell
  .\build\local\apps\Debug\arm_cli.exe --sim line 0.03 0.19 0.12 0.01
  ```

Am echten Arm (sobald das Netzteil angeschlossen ist) einfach `--sim` durch
`--port COM7` bzw. `--port /dev/ttyACM0` ersetzen. Dabei wirst du merken, dass jeder Wegpunkt
etwa eine halbe Sekunde dauert: Die Firmware wartet nach jedem Befehl, bis die Servos angekommen
sind, und schaltet sie dann ab. Für flüssige Bewegungen bräuchte die Firmware einen
Streaming-Modus, ein gutes Thema für später („Firmware v2“ in der Roadmap).

**Erweiterung (optional):** Baue in `arm_cli` eine Option `--smooth` ein, die bei `joints` und
`home` statt eines Sprungs `interpolateEased(..., smoothstep)` mit z.B. 10 Schritten benutzt.
