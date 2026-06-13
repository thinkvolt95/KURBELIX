# KURBELIX

A fork of `Svenbosma/DIY-Garmin-Powermeter`.

---

## KURBELIX 2 (Stable)

* **Status:** Current working version
* **Tested with:** Garmin Edge 830

### Key Improvements & Changes

* **Sensor Fusion Cadence:** Calculates cadence using a sensor fusion of the gyroscope and accelerometer via a complementary filter.
* **Position-Based Drift Correction:** Corrects the gyro angle using the accelerometer angle, utilizing a weighting function based on the crank's physical position (strong correction at 90°/270°, weak to zero correction at 0°/180°, using linear interpolation).
* **Cadence-Based Weighting:** Adjusts the gyro/accelerometer correction dynamically based on cadence (high weight at 40 RPM, low weight at 100 RPM).
* **Absolute Angle Tracking:** Uses the accelerometer to determine the absolute crank angle, ensuring 0° always represents Top Dead Center (12 o'clock).
* **Gyro Drift Correction:** Automatic calibration/drift correction when the crank is at a standstill.
* **Improved Dead Center Handling:** Smoother transitions through dead spots.
* **Spike Filtering:** Implements filters for both torque and power spikes.
* **Weighted Torque Accumulation:** Accumulates torque using a weighting function based on the angular velocity at each specific measurement point.
* **Coasting Optimization:** Improved handling of values during coasting to ensure the Garmin head unit calculates accurate mean and max values for power and cadence.
* **BLE CPS Fix:** Fixed the Measurement Characteristic length issue within the Bluetooth Low Energy stack.
* **Timestamp Fix:** Resolved a crank timestamp overflow bug.

---

## KURBELIX 3 (Work in Progress)

* **Status:** Active development version 

### Advanced BLE Metrics Support
The Cycling Power Service (CPS) and Cycling Power Profile (CPP) protocols define several advanced metrics beyond standard cadence and power. This version calculates and transmits these additional values over BLE.

> ⚠️ **Note on Garmin Compatibility:** Garmin devices generally do not process these advanced metrics over BLE, restricting them to ANT+ connections. However, other head units (e.g., Wahoo, Hammerhead) or third-party apps (e.g., Zwift) may support them over Bluetooth.

### Newly Implemented Features (CPP/CPS)
* **Cycling Power Vector:** Added full profile support.
* **Extreme Magnitudes & Angles:** Tracking and transmission of peak force values and their respective angular positions.
* **Dead Spot Tracking:** Detection and transmission of Top and Bottom Dead Spot angles.
* **Accumulated Energy:** Real-time calculation and reporting of energy expenditure.

---

## License

This is an open-hardware and open-source project.  
Feel free to modify, improve, and build your own version!
