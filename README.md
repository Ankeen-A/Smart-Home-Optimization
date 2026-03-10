# Smart-Home-Optimization
Arduino-based smart home optimization system that minimizes energy consumption using algorithmic control and fuzzy logic. Coordinates blinds, lighting, HVAC, and appliance scheduling to reduce power usage while maintaining indoor comfort.

# Smart Home Optimization (Algorithmic Control + Fuzzy Logic)

This repository contains my Master’s thesis project: an Arduino-based smart home system that reduces energy use by coordinating **blinds, LED lighting, and HVAC behavior** with an optimization algorithm, and managing a **washing machine controller** using **Mamdani fuzzy logic** with time-of-use scheduling.

📄 Full thesis report: `docs/Smart_Home_System_Thesis_Ankeen_Arestakesyan.pdf`

---

## What this system does

### 1) Energy optimization (lighting + blinds + HVAC)
The controller continuously searches for the best combination of:
- **Blind openness** (servo-controlled)
- **LED brightness** (PWM)

It minimizes **total electrical power** while maintaining comfort targets:
- **Indoor temperature setpoint** (default: 21°C)
- **Target illuminance** (default: 400 lux)

The model includes:
- Solar heat gain through windows (with blind transmission)
- Conductive heat transfer (walls + windows)
- LED heat contribution
- HVAC power proxy using COP modeling (and fan scaling with temperature error)

### 2) Appliance control (washing machine) using fuzzy logic
A Type-1 Mamdani fuzzy controller takes:
- Load size
- Soil level
- Fabric type (softness)

and outputs:
- Wash cycle type
- Wash time

The system then schedules the wash to run during lower electricity-rate windows (time-of-use logic).

---

## Hardware (prototype)
- **Arduino Mega 2560**
- DHT11 temperature sensor
- Photoresistor (LDR) for sunlight measurement
- Potentiometer (used to simulate outdoor temperature in testing)
- Servo motor (blind position)
- LEDs (room lighting)
- DC motor + MOSFET (HVAC fan simulation)
- IR receiver + LCD (user input + status display)
- RGB LED (washer status indicator)

---

## Software / Tools
- Arduino IDE (C/C++)
- MATLAB Fuzzy Logic Designer (controller design)
- `Fuzzy.h` library (Arduino implementation)

---

## How it works (high level)

### Optimization loop (climate + lighting)
At each refresh interval, the controller evaluates a grid of blind/LED settings (1% steps) and computes:
- indoor illuminance from sun + LEDs
- total heat gain (sun + conduction + LEDs)
- estimated HVAC power requirement

A penalty is applied if illuminance falls below the minimum target, and the lowest-cost configuration is applied (servo position, LED PWM, fan PWM).

### Fuzzy washer logic
User inputs are mapped into fuzzy membership functions and defuzzified to produce a wash time and cycle. Scheduling logic compares the required runtime against low-rate windows and decides whether to start immediately or delay.

---

## Results (from thesis simulations)
Compared to a manually operated home model, the system achieved:
- ~33% energy savings (cold weather simulation)
- ~23% energy savings (mild weather simulation)
- ~34% energy savings (hot weather simulation)

Appliance scheduling during low-rate periods showed up to ~37% cost savings depending on usage patterns.

---

## Notes / Limitations
This project uses simplified thermal and lighting models for demonstration and simulation. Future improvements include:
- real-time clock integration for scheduling
- more detailed solar position / irradiance modeling
- expanded appliance coverage (dryer, dishwasher)
- real sensor calibration and deployment in a real room

---

## License
MIT (see `LICENSE`)
