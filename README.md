# Hardware Test Build

## Current Wiring Plan
| Component | Arduino Pin | Notes |
|------------|-------------|-------|
| Blue LED | D13 | Turns ON/OFF via buttons |
| Button 1 | D8 | LED ON when pressed |
| Button 2 | D9 | LED OFF when pressed |
| MAX30102 Heart-Rate Sensor | SDA → A4, SCL → A5 | I²C connection |

---

## Status
**Version:** Hardware Test Build 1.0  
**Purpose:** Verifying that all hardware components are wired correctly and responding.  
**Features Tested:**
- ✅ LED output toggle  
- ✅ Button input detection  
- ✅ MAX30102 sensor communication and raw IR/Red data output  

**Next Steps:**
- Refine sensor data processing (calculate heart rate, detect peaks)  
- Integrate display or output system (optional)  
- Begin main project logic after hardware validation  

---

*Last updated: Nov 5, 2025*
