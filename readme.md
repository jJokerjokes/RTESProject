# Parkinson's Disease Monitoring System
### "Shake, Rattle, and Roll... and Freeze"

**Course:** ECE-GY 6483 - Real Time Embedded Systems (Fall 2025)  
**Group:** 30  
**Lead Author:** Bai, Gengyuan; Zhang, Charles

**Team Members:**
1. Bai, Gengyuan
2. Hua, Bin
3. Huang, Yanjie
4. Lee, Kylie
5. Zhang, Charles

---

## 📝 Project Progress (TODO List)

### ✅ Completed & Verified
- [x] **Hardware Setup:** MCU, Accelerometer, and LED control verified.
- [x] **Data Acquisition:** Non-blocking sampling at 52Hz via LSM6DSL.
- [x] **Signal Processing:** DC Removal (Gravity subtraction) and FFT algorithm working correctly.
- [x] **Symptom Detection (Tremor):** Successfully detected 3-5 Hz oscillations.
- [x] **Symptom Detection (Dyskinesia):** Successfully detected 5-7 Hz movements.
- [x] **Visual Feedback:** LED patterns implemented for all states.
- [x] **Fault Tolerance:** System auto-recovers from sensor errors without crashing.

### 🔄 Updates (by Zhang, Charles)
- **Parameter Tuning:** Adjusted FOG variance thresholds, drop ratio, and signal magnitude threshold for improved sensitivity.
- **Initialization:** Added hardware self-check (LEDs), sensor ID verification, and register-level configuration.
- **Robustness:** Added safety flags (`imuOk`, `bleOk`) to prevent system crashes on hardware failure.
- **Dependencies:** Added LoRa library support.

### ⏳ In Progress / To Be Verified
- [x] **BLE Connectivity:** Code implemented, verified by Zhang, Charles.
- [x] **FOG Simulation:** The "Freezing of Gait" algorithm (Variance drop detection) is implemented and tuned by Zhang, Charles.

---

## 📖 Project Overview

This project is an embedded system designed to monitor and detect symptoms of Parkinson's Disease using the **ST B-L475E-IOT01A** development board.

By utilizing the onboard **LSM6DSL** accelerometer and gyroscope, the system analyzes movement data in real-time using **FFT (Fast Fourier Transform)** and statistical analysis to detect three specific conditions:
1.  **Tremor:** Rhythmic shaking (Frequency: 3-5 Hz).
2.  **Dyskinesia:** Involuntary dance-like movements (Frequency: 5-7 Hz).
3.  **Freezing of Gait (FOG):** Sudden cessation of movement during walking (Detected via variance drop + low frequency).

---

## 💡 LED Status Indicators (Visual Feedback)

The system uses the onboard LEDs to provide immediate visual feedback.
*(Please refer to the attached diagram for physical LED locations)*.

> **Note:** The board has a permanently lit **Red LED** (Power) and a **Green LED** (3.3V) that remain ON whenever the device is powered. These are hardware indicators and not part of the software logic.

### Detection Modes

![State Machine to LED Mapping for Parkinson's Monitoring](Figure1_State-Machine_to_LED_Mapping_for_Parkinson's_Monitoring.jpeg)

| Condition | Color Pattern | Animation Behavior |
| :--- | :--- | :--- |
| **Standby**<br>(Monitoring) | **Blue (Solid) + Green (Blinking)** | The **Blue light** stays ON continuously.<br>The **Green light** blinks slowly (Breathing effect). |
| **Tremor**<br>(3-5 Hz) | **Two Greens (Strobe) + Blue (Solid)** | The **Blue light** stays ON.<br>**Two adjacent Green lights** strobe rapidly back and forth (High frequency flash). |
| **Dyskinesia**<br>(5-7 Hz) | **Green / Yellow / Blue** | Alternating sequence involving Green, Yellow, and Blue lights to simulate chaotic movement. |
| **FOG**<br>(Freezing) | **Two Greens + Yellow (Solid)** | **Two adjacent Green lights** and the **Yellow light** turn SOLID ON.<br>The **Blue light** turns OFF.<br>(Represents a "Lock-up" state). |

> **Hold Time:** Once a symptom (Tremor, Dyskinesia, or FOG) is detected, the LED pattern is held for **2 seconds** to ensure the user can see the alert clearly.

---

## 🛠 Implementation Details

### 1. Non-Blocking Architecture
The system operates on a strictly **non-blocking** loop using `millis()` scheduling.
*   **Sampling Rate:** 52 Hz (approx. every 19ms).
*   **Concurrency:** LED animations, BLE polling, and sensor data collection run concurrently without `delay()` blocking.

### 2. Signal Processing Pipeline
*   **Data Buffering:** Captures 256 samples.
*   **DC Removal:** Automatically subtracts the gravity component (mean value) to isolate pure movement signals.
*   **FFT Analysis:** Applies Hamming window and computes peak frequency.
*   **FOG Logic:** Uses a Finite State Machine (FSM) tracking movement variance.

---

## 📡 Bluetooth Low Energy (BLE) Guide

The device advertises as a custom peripheral.

*   **Device Name:** `PD_Monitor`
*   **Service UUID:** `19B10000-E8F2-537E-4F6C-D104768A1214`

### Characteristics
| Characteristic | UUID | Type | Function |
| :--- | :--- | :--- | :--- |
| **Tremor** | `...1214` (ends in 1) | Read / Notify | 1 = Detected, 0 = None |
| **Dyskinesia** | `...1214` (ends in 2) | Read / Notify | 1 = Detected, 0 = None |
| **FOG** | `...1214` (ends in 3) | Read / Notify | 1 = Detected, 0 = None |

### ⚠️ Important for iOS Users
Do **NOT** try to pair via the iPhone system Settings. iOS hides custom data devices by default.
1.  Download **nRF Connect** or **LightBlue** from the App Store.
2.  Scan for `PD_Monitor`.
3.  Connect and subscribe to characteristics to see real-time updates.

---

## 🚀 How to Build & Run

1.  **Setup Hardware:** Connect B-L475E-IOT01A via USB ST-LINK port to PC.
2.  **Open Project:** Open this folder in VS Code with PlatformIO extension installed.
3.  **Build:** Click the Checkmark (✔) icon or run `pio run`.
4.  **Upload:** Click the Arrow (→) icon or run `pio run --target upload`.
5.  **Monitor:** Click the Plug (🔌) icon to open Serial Monitor (Baud: 115200) for debug logs.

---

## 📊 Serial Monitor Output Example

Below is a sample output from the serial monitor showing the real-time analysis:

```
<<<<<<<<<<<<< Author: Bai, Gengyuan; Zhang, Charles >>>>>>>>>>>>>

********** Buffer Full **********
Time elapsed: 4845 ms
Expected: 4864 ms

========== FFT Analysis ==========
Variance: 0.0000
Std Dev: 0.0011
Mean (DC component): 1.0173 g
Removing DC component...
Mean after DC removal: -0.000000 g (should be ~0)
Variance after DC removal: 0.0000
Peak Frequency: 2.00 Hz
Peak Magnitude: 0.02
Low signal power (0.02 < 1.5), ignoring...
(Likely just noise or very subtle motion)

--- FOG Detection Analysis ---
Previous Moving State: NO
Current Moving State: NO

========== Status Summary ==========
Tremor: NO
Dyskinesia: NO
FOG (Freezing of Gait): NO
====================================

Ready for next data collection...
```

**Key Metrics Explained:**
*   **Variance:** Measures movement intensity (higher = more movement).
*   **Peak Frequency:** Dominant frequency in the acceleration signal.
*   **Peak Magnitude:** Signal strength (must exceed 1.5 for valid detection).
*   **FOG Detection:** Tracks sudden drops in variance during movement.

---

## 📄 License & Credits
Project created for NYU Tandon ECE-GY 6483.
Copyright © 2025 Group 30. All rights reserved.