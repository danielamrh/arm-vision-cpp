# Tutorial 1: Eine serielle Schnittstelle unter Linux (POSIX/termios)

In diesem Tutorial implementierst du `SerialPort` für Linux, damit der Raspberry Pi mit dem
ESP32 sprechen kann. Die Windows-Version (`src/serial_port_win.cpp`) gibt es schon; die
Schnittstelle (`include/armvision/serial_port.hpp`) bleibt gleich, nur die Implementierung in
`src/serial_port_posix.cpp` ist neu.

**Du lernst:** POSIX-Systemaufrufe (`open`, `read`, `write`, `poll`), `termios`, `errno` und
`std::system_error`, Zeitlimits mit `std::chrono`, und wie man Systemcode ohne Hardware testet.

| Lektion | Inhalt | Tests, die grün werden |
|---|---|---|
| 1 | Gerät öffnen | `ThrowsSystemErrorForMissingDevice` |
| 2 | `termios` und Baudrate | `RejectsUnsupportedBaudRate` |
| 3 | Zeilen lesen mit Zeitlimit | `ReadsALineAndStripsCrLf`, `ReadLineTimesOutWithoutData`, `SplitsSeveralLinesAndKeepsAPartialOne` |
| 4 | Schreiben | `WriteSendsBytesUnchanged` |
| 5 | Eingang verwerfen | `DiscardInputDropsBufferedData` |

---

## Arbeitsablauf

Du schreibst den Code am **Windows-PC** und testest ihn in **WSL** (Linux in Windows). Der Pi
wird erst am Ende gebraucht.

```powershell
# im Repo-Ordner, in PowerShell:
wsl bash tools/test_serial_wsl.sh                                   # alle 7 Tests
wsl bash tools/test_serial_wsl.sh SerialPortPosix.ReadsALine*       # nur ausgewählte
```

Ein Test schlägt fehl, solange eine Funktion noch `throw std::logic_error("... not implemented
yet")` enthält. Das ist Absicht: So siehst du immer, was noch fehlt.

**Wie die Tests funktionieren:** `tests/test_serial_posix.cpp` erzeugt ein *Pseudo-Terminal*
(pty), ein Paar aus zwei virtuellen Geräten. Dein `SerialPort` öffnet die eine Seite
(`/dev/pts/N`), der Test spielt auf der anderen Seite den ESP32. Für den Kernel sieht das aus wie
ein echtes serielles Gerät.

Fertig? Dann: Code-Review, committen, pushen, und auf dem Pi `git pull`, bauen, `ctest`
(siehe [Abschluss](#abschluss-auf-dem-raspberry-pi)).

---

## Lektion 1: Gerät öffnen

**Ziel:** `SerialPort("/dev/does-not-exist", 115200)` wirft `std::system_error`.

**Hintergrund.** Unter Linux ist (fast) alles eine Datei, auch eine serielle Schnittstelle
(`/dev/ttyACM0`). `::open()` liefert einen **File Descriptor** zurück, eine kleine Ganzzahl, über
die alle weiteren Zugriffe laufen. Bei einem Fehler gibt `open` `-1` zurück und schreibt den Grund
in die globale Variable **`errno`** (z.B. `ENOENT` = „Datei existiert nicht“).
`throwErrno()` verpackt `errno` in eine `std::system_error`, die eine lesbare Meldung erzeugt.

Den Descriptor speicherst du in `impl_->fd`. Geschlossen wird er automatisch im Destruktor von
`Impl` (RAII), auch wenn der Konstruktor später eine Exception wirft.

**Aufgabe:** TODO 1 in `src/serial_port_posix.cpp`.

<details><summary>Hinweis 1</summary>

Die Flags: `O_RDWR` (lesen und schreiben), `O_NOCTTY` (das Gerät soll nicht zum „steuernden
Terminal“ unseres Prozesses werden), `O_CLOEXEC` (wird bei `exec` nicht an Kindprozesse vererbt).
Sie werden mit `|` kombiniert. Details: `man 2 open`.
</details>

<details><summary>Hinweis 2 (fast die Lösung)</summary>

```cpp
impl_->fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
if (impl_->fd < 0) throwErrno("cannot open " + port);
```
</details>

**Zum Nachdenken:** `throwErrno` liest `errno` erst *nach* dem Zusammenbauen des Strings
`"cannot open " + port`. Warum könnte das theoretisch ein Problem sein, und wie würdest du es
vermeiden?

---

## Lektion 2: termios und Baudrate

**Ziel:** Der Port ist im „Raw-Modus“ mit der richtigen Baudrate, und eine unbekannte Baudrate
wirft `std::invalid_argument`.

**Hintergrund.** Ein Terminal-Gerät verarbeitet Daten standardmäßig für menschliche Eingabe: Es
wartet auf Enter, gibt Zeichen als Echo zurück, macht aus `\n` beim Senden `\r\n`, behandelt
Strg+C … Für ein Protokoll mit dem ESP32 wollen wir die Bytes **unverändert**. Das stellt man mit
der Struktur `termios` ein:

1. `tcgetattr(fd, &tio)` liest die aktuellen Einstellungen in deine lokale Kopie `tio`.
2. Du änderst die Kopie: `cfmakeraw(&tio)` schaltet die ganze Verarbeitung ab; dazu
   `tio.c_cflag |= CLOCAL | CREAD` (Modem-Leitungen ignorieren, Empfänger einschalten).
3. `c_cc[VMIN] = 0`, `c_cc[VTIME] = 0`: `read()` blockiert nie und liefert sofort, was da ist
   (auch 0 Bytes). Gewartet wird stattdessen mit `poll()` in Lektion 3.
4. `cfsetispeed` / `cfsetospeed` setzen die Baudrate für Empfangen und Senden.
5. Erst `tcsetattr(fd, TCSANOW, &tio)` wendet alles an. Danach `tcflush(fd, TCIOFLUSH)`, um
   Datenreste aus der Zeit vor dem Öffnen zu verwerfen.

**Baudrate:** `termios` will keine Zahl wie `115200`, sondern eine Konstante vom Typ `speed_t`
(`B115200`, deren Wert intern etwas ganz anderes ist). Du brauchst also eine Übersetzung.

**Aufgabe:** TODO 2. Entferne danach `(void)port; (void)baud;` und das
`throw std::logic_error(...)` am Ende des Konstruktors.

<details><summary>Hinweis 1: Aufbau</summary>

Eine Hilfsfunktion im anonymen Namespace (neben `throwErrno`):

```cpp
/// Maps a baud rate in bit/s to the termios constant, or nullopt if unsupported.
std::optional<speed_t> toSpeed(unsigned baud) {
    switch (baud) {
        case 9600: return B9600;
        // ... 19200, 38400, 57600, 115200, 230400
        default: return std::nullopt;
    }
}
```

Im Konstruktor **vor** `open()` aufrufen („fail fast“: bei falschem Argument gar nicht erst das
Gerät öffnen). Bei `std::nullopt` eine `std::invalid_argument` werfen, z.B. mit
`"unsupported baud rate: " + std::to_string(baud)`.
</details>

<details><summary>Hinweis 2: Fehlerprüfung</summary>

`tcgetattr`, `cfsetispeed`, `cfsetospeed` und `tcsetattr` geben bei Erfolg `0` zurück, sonst
`-1` mit `errno`. Prüfe jeden Aufruf, z.B.
`if (::tcsetattr(fd, TCSANOW, &tio) != 0) throwErrno("tcsetattr");`
</details>

**Zum Nachdenken:** Warum ist es egal, ob die Exception *vor* oder *nach* `open()` geworfen wird,
was das Schließen des Descriptors angeht? (Tipp: Wer besitzt `fd`?)

---

## Lektion 3: Zeilen lesen mit Zeitlimit

**Ziel:** `readLine(timeout)` liefert die nächste Zeile ohne `\r\n`, hebt angefangene Zeilen für
später auf und gibt nach Ablauf des Zeitlimits `std::nullopt` zurück.

**Hintergrund.** Der Rahmen von `readLine` ist schon da: Er schaut zuerst in `pending`, ob dort
bereits eine vollständige Zeile liegt (ein `read()` kann mehrere Zeilen auf einmal liefern, oder
nur eine halbe). Fehlt nur noch das **Warten auf neue Daten**:

- `poll()` wartet, bis ein Descriptor lesbar ist, höchstens `timeout` Millisekunden. Rückgabe:
  `> 0` = Daten da, `0` = Zeit abgelaufen, `-1` = Fehler (`errno`).
- Wird der Prozess währenddessen von einem Signal unterbrochen, kommt `-1` mit
  `errno == EINTR`. Das ist kein echter Fehler: einfach die Schleife weiterlaufen lassen.
- Danach liest `read()` die verfügbaren Bytes in einen Puffer, und du hängst sie an `pending` an.

**Aufgabe:** TODO 4.

<details><summary>Hinweis 1: Restzeit berechnen</summary>

`deadline - now` ist eine `std::chrono::duration`. `poll` will ein `int` in Millisekunden.
Achtung beim Runden: `duration_cast` rundet **ab**. Bleiben z.B. 0,4 ms übrig, wird daraus
`poll(..., 0)`, das sofort zurückkehrt, und die Schleife dreht leer, bis die Deadline erreicht
ist. Besser: `std::chrono::ceil<std::chrono::milliseconds>(deadline - now).count()`.
</details>

<details><summary>Hinweis 2: Gerüst</summary>

```cpp
pollfd pfd{impl_->fd, POLLIN, 0};
const int r = ::poll(&pfd, 1, /* Restzeit in ms */);
if (r == 0) return std::nullopt;          // Zeit abgelaufen
if (r < 0) { /* EINTR: continue; sonst throwErrno("poll") */ }

char buf[256];
const ssize_t got = ::read(impl_->fd, buf, sizeof(buf));
// got > 0: an pending anhängen (Achtung: ssize_t -> std::size_t umwandeln)
// got < 0: EAGAIN/EINTR ignorieren, sonst throwErrno("read")
```
</details>

**Zum Nachdenken:** Warum wartet man mit `poll()` statt `VMIN`/`VTIME` von `termios` zu
benutzen? (Tipp: Was passiert mit dem Zeitlimit, wenn Daten tröpfchenweise ankommen?)

---

## Lektion 4: Schreiben

**Ziel:** `write("90,120,90,90\n")` kommt Byte für Byte unverändert beim ESP32 an.

**Hintergrund.** `::write()` darf **weniger** Bytes schreiben, als man ihm gibt, und gibt die
tatsächlich geschriebene Anzahl zurück. Robuster Code schreibt deshalb in einer Schleife, bis
alles draußen ist. Auch hier gilt: `-1` mit `EINTR` heißt „nochmal versuchen“.

**Aufgabe:** TODO 3.

<details><summary>Hinweis</summary>

`std::string_view` hat `remove_prefix(n)`: Damit „verbrauchst“ du die geschriebenen Bytes und
schreibst in der nächsten Runde den Rest. Die Schleife läuft, solange `!data.empty()`.
</details>

Schlägt `WriteSendsBytesUnchanged` fehl und kommt `"...\r\n"` an statt `"...\n"`, fehlt der
Raw-Modus aus Lektion 2.

---

## Lektion 5: Eingang verwerfen

**Ziel:** Nach `discardInput()` sind alle bisher empfangenen, aber noch nicht gelesenen Daten weg.

**Hintergrund.** Ungelesene Daten liegen an **zwei** Stellen: im Puffer des Kernels und in
unserem eigenen `pending`. Beide müssen geleert werden. Der Treiber benutzt das nach dem
`READY` des ESP32, damit alte Meldungen nicht als Antwort auf einen neuen Befehl gelesen werden.

**Aufgabe:** TODO 5 (`tcflush` mit `TCIFLUSH`, und `pending.clear()`).

Jetzt sollten alle 7 Tests grün sein und die Warnung „`throwErrno` defined but not used“ ist weg.

---

## Abschluss auf dem Raspberry Pi

Nach Code-Review, Commit und Push, per SSH auf dem Pi:

```bash
cd ~/arm-vision-cpp
git pull
cmake --build --preset linux
ctest --preset linux
```

Alle 21 Tests sollten grün sein. Danach der echte Test mit dem ESP32 am USB-Port des Pi:

```bash
./build/linux/apps/arm_cli --port /dev/ttyACM0 home
```
