# Final Wiring Plan — HW-104 PAM8403 + 5 White LEDs

## Power Rails
- **5V (high-current rail):** PAM8403 amplifier, 5 × white LEDs (via MOSFET)  
- **5V (logic rail):** Arduino Uno R4, mic sensor, and any buttons using external pull-downs (most use `INPUT_PULLUP`)  
- **3.3V:** MAX30102, ADXL345 (most breakouts also accept 5V on VIN; see note below)  
- **GND:** all components share a **common ground**

**Supply note:**  
If total current may exceed ~500 mA, power both the Arduino and PAM8403 from the **same external 5 V, ≥2 A** supply.  
Tie all grounds together.

---

## I²C Bus (shared)
- **SDA → A4**  
- **SCL → A5**

---

## Already-Set Pins
| Component | Pin | Notes |
|------------|-----|-------|
| Blue LED (+ 220 Ω) | D13 | Status indicator |
| Button 1 | D8 | Active-LOW with `INPUT_PULLUP` |
| Button 2 | D9 | Active-LOW with `INPUT_PULLUP` |

---

## Suggested Assignments for Remaining Parts

### 5 × White LEDs (group control via MOSFET)
- **D6 → 100 Ω → MOSFET Gate** (logic-level N-MOSFET e.g. AO3400 / BS170 / 2N7000)  
- **10 kΩ Gate pulldown:** gate → GND  
- **Each LED has its own series resistor** (330–1 kΩ depending on brightness)  
- Wiring per LED (5 copies in parallel to MOSFET drain):  
  - **5V (high-current)** → LED anode  
  - **LED cathode → series resistor → MOSFET drain**  
- **MOSFET source → GND**  
- Optional: **100 µF electrolytic** across 5V/GND near LED cluster

---

### Microphone (KY-037 / LM393)
- **AO → A0** (analog level)  
- **DO → D2** (digital threshold / clap detect; interrupt-friendly)  
- **VCC → 5V**, **GND → GND**

---

### PAM8403 Audio Amplifier (HW-104 bare-board) + 2 × 8 Ω Speakers
> ⚠️ You must solder header pins or wires to the pads.

Pads (silkscreen): `5V`, `GND`, `L IN`, `R IN`, `L+`, `L−`, `R+`, `R−`

| Pad | Connect To | Notes |
|------|-------------|-------|
| **5V** | 5 V (high-current rail) | Add 100 µF capacitor near board |
| **GND** | Common GND | Shared with Arduino and sensors |
| **L IN** | D11 → 1 kΩ → 0.1–1 µF (series cap) | Left audio input |
| **R IN** | D11 → 1 kΩ → 0.1–1 µF (series cap) | Right audio input |
| **L+ / L−** | Speaker 1 | 3 W max @ 5 V (4 Ω) |
| **R+ / R−** | Speaker 2 | 3 W max @ 5 V (4 Ω) |

Keep input leads short; avoid routing speaker lines near I²C or mic signals.

---

### ADXL345 Accelerometer (I²C mode)
- **VCC → 3.3V**  
- **GND → GND**  
- **SDA → A4**, **SCL → A5**  
- **CS → 3.3V** (forces I²C mode)  
- **SDO → GND** (I²C address 0x53; tie to 3.3V for 0x1D)

---

### MAX30102 Heart-Rate Sensor
- If breakout has **VIN (3.3–5 V):** **VIN → 5V**, **GND → GND**, **SDA → A4**, **SCL → A5**  
- If breakout has only **3.3V VCC:** **VCC → 3.3V** (most boards include 4.7 k pull-ups to 3.3 V — ideal for shared I²C)

---

### Remaining Buttons (6 extra, total 8)
- Use **D3–D7, D10** (with D8/D9 already used; D2 is mic DO)  
- One side → pin, other → **GND**  
- In code: `pinMode(pin, INPUT_PULLUP)` and treat **pressed = LOW**  
- No external resistors needed

---

## Minimal Hook-Up List (by Part)
| Part | Connections |
|------|--------------|
| **MAX30102** | VIN → 5V (or VCC → 3.3V), GND → GND, SDA → A4, SCL → A5 |
| **ADXL345** | VCC → 3.3V, GND → GND, SDA → A4, SCL → A5, CS → 3.3V, SDO → GND |
| **KY-037 / LM393 Mic** | VCC → 5V, GND → GND, AO → A0, DO → D2 |
| **5 × White LEDs (grouped)** | 5V → each LED anode; each LED cathode → resistor (330–1 kΩ) → MOSFET drain; MOSFET source → GND; D6 → 100 Ω → gate; 10 kΩ gate→GND; optional 100 µF near LEDs |
| **PAM8403 (HW-104) + Speakers** | 5V → 5V (high-current), GND → GND, D11 → 1 kΩ → 0.1–1 µF → L IN, D11 → 1 kΩ → 0.1–1 µF → R IN, L+/L− → Speaker 1, R+/R− → Speaker 2, 100 µF across 5V/GND at board |
| **Buttons (×8)** | D3–D7, D8, D9, D10 (one side → pin, other → GND, `INPUT_PULLUP`) |
| **Blue LED** | D13 → 220 Ω → LED anode, LED cathode → GND |

---

## Soldering Notes (HW-104)
- Use **2.54 mm right-angle male headers** (best for breadboard) or **24–26 AWG stranded wires** (strain-relieved with hot glue).  
- Keep speaker wires short and twisted if possible; route away from I²C and mic lines.  
- Double-check pad labels before soldering — some clones swap pad order.