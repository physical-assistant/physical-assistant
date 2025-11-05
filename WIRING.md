# Final Wiring Plan

## Power Rails
- **5V (high-current rail):** WS2812B strip + PAM8403 amplifier  
- **5V (logic rail):** Arduino Uno R4, mic sensor, and some buttons if using external pull-downs  
- **3.3V:** MAX30102, ADXL345 (most breakouts can also take 5V on VIN; see note below)  
- **GND:** all components share a common ground  

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
### WS2812B LED Strip  
- **DIN → D6** (through 330–470 Ω series resistor)  
- **5V → 5V (high-current rail)**  
- **GND → common GND**  
- Add 1000 µF capacitor across 5V/GND at strip start  

### Microphone (KY-037 / LM393)  
- For threshold/clap detect: **DO → D2** (interrupt-friendly)  
- For analog level reading: **AO → A0**  
- **VCC → 5V**, **GND → GND**

### PAM8403 Audio Amplifier (+ 2 × 8 Ω Speakers)  
- **VCC → 5V (high-current rail)**  
- **GND → common GND**  
- **D11 → 1 kΩ → 0.1–1 µF → L IN**  
- **D11 → 1 kΩ → 0.1–1 µF → R IN**  
- **L+/L− → Speaker 1**, **R+/R− → Speaker 2**

### ADXL345 Accelerometer (I²C mode)  
- **VCC → 3.3V**  
- **GND → GND**  
- **SDA → A4**, **SCL → A5**  
- **CS → 3.3V** (forces I²C mode)  
- **SDO → GND** (I²C address = 0x53; tie to 3.3V for 0x1D)

### MAX30102 Heart-Rate Sensor  
- If breakout has VIN (3.3–5V): **VIN → 5V**, **GND → GND**, **SDA → A4**, **SCL → A5**  
- If only 3.3V VCC: **VCC → 3.3V** (most boards have 4.7k pull-ups to 3.3V on SDA/SCL — ideal for shared I²C)  

### Remaining Buttons (6 extra, total 8)  
- Use **D2–D7** and **D10** for the other six buttons (two already on D8/D9)  
- One side → pin, other side → GND  
- In code: `pinMode(pin, INPUT_PULLUP)` and treat pressed = LOW  
- No external resistors needed  

---

## Minimal Hook-Up List (by Part)
| Part | Connections |
|------|--------------|
| **MAX30102** | VIN → 5V (or VCC → 3.3V), GND → GND, SDA → A4, SCL → A5 |
| **ADXL345** | VCC → 3.3V, GND → GND, SDA → A4, SCL → A5, CS → 3.3V, SDO → GND |
| **KY-037 / LM393 Mic** | VCC → 5V, GND → GND, AO → A0 and/or DO → D2 |
| **WS2812B Strip** | 5V → 5V (high-current), GND → common GND, DIN → D6 (through 330–470 Ω resistor), 1000 µF capacitor across 5V/GND |
| **PAM8403 + Speakers** | VCC → 5V (high-current rail), GND → common GND, D11 → 1 kΩ → 0.1–1 µF → L IN / R IN, L+/L− → Speaker 1, R+/R− → Speaker 2 |
| **Buttons (×8)** | Pins: D2–D7, D8, D9. One side → pin, other → GND. Use `INPUT_PULLUP`. |
| **Blue LED** | D13 → 220 Ω → LED anode, LED cathode → GND |