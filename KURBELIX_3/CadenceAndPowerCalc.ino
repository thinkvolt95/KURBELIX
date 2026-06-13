bool shouldAccumulateTorqueSample(float torqueNm, float correctedGyroZ_dps) {
  // 1. Gyro muss vorwärts drehen
  if (correctedGyroZ_dps < 2.0f) return false;

  // 2. Winkelgeschwindigkeit berechnen
  float angVel = correctedGyroZ_dps * (PI / 180.0f);
  if (!isfinite(angVel)) return false;

  // 3. Totpunkt-Rauschen vermeiden (< ~10 rpm)
  if (angVel < 1.0f) return false;

  // 4. Unrealistische Torque-Spikes filtern
  if (fabs(torqueNm) > 120.0f) return false;

  // 5. Unrealistische Power-Spikes filtern
  float samplePower = torqueNm * angVel * 2.0f;
  if (!isfinite(samplePower)) return false;
  if (fabs(samplePower) > MAX_VALID_POWER_W) return false;

  return true;
}

void CadencePowerCalc(uint32_t revolutionUs) {
  lastRevUs = revolutionUs;

  // Average all torque samples collected since the previous revolution.
  float avgTorque = (torqueSampleCount > 0.0f) ? (sumTorqueNm / torqueSampleCount) : 0.0f;

  logPrint("torqueSampleCount= ");
  logPrint(torqueSampleCount);
  logPrint(" avg torque=");
  logPrint(avgTorque, 1);
  logPrint(" gyroZ=");
  logPrint(lastGyroZ_dps, 2);

  // Cadence and power come from the same revolution-to-revolution period that
  // Garmin will infer from the CPS crank event timestamps.
  uint32_t nowUs = revolutionUs;
  float dt = 0.0f;
  float cadence_rpm = 0.0f;
  float angVelRad = 0.0f;


  if (previousCadenceRevUs != 0) {
    dt = (nowUs - previousCadenceRevUs) / 1000000.0f;  // seconds

    // Clamp to a plausible upper cadence limit.
    if (dt < 0.2f) dt = 0.2f;
    if (dt > 2.0f) dt = 2.0f;

    cadence_rpm = 60.0f / dt;
    estimatedCadenceRpm = cadence_rpm;

    // Convert one revolution per dt into angular speed in rad/s.
    float angVelRadLocal = (2.0f * PI) / dt;
    angVelRad = angVelRadLocal;

    if (cadence_rpm >= MIN_POWER_CADENCE_RPM) {
      // This meter reads one crank arm, so report estimated total power multiplied by two.
      powerW = (avgTorque * angVelRadLocal) * 2.0f;
    } else {
      powerW = 0.0f;
    }

    if (powerW > MAX_VALID_POWER_W) {
      powerW = 0.0f;
    }
  }

  if (powerW < 0.0f) {
    powerW = 0.0f;
  }

  instantaneousPower = powerW;

  previousCadenceRevUs = nowUs;

  cumulativeCrankRevs++;

  // BLE Cycling Power uses 1/1024 second event timestamps.
  lastCrankEventTime = (uint16_t)(((uint64_t)nowUs * 1024ULL) / 1000000ULL);

 

  // Start a fresh averaging window for the next crank revolution.
  sumTorqueNm = 0.0f;
  torqueSampleCount = 0.0f;

  logPrint(" Cadence=");
  logPrint(cadence_rpm, 1);
  logPrint(" rpm  Power=");
  logPrintln(powerW, 1);
}

static uint32_t lastZeroPowerSendMs = 0;

void sendZeroPowerIfStopped() {
  uint32_t nowMs = millis();
  if (nowMs - lastZeroPowerSendMs < 1000) return;
  lastZeroPowerSendMs = nowMs;

  // Minimal CPS: Power=0, keine Crank-Daten
  sendCyclingPowerMeasurement_CPS(
    0,                     // instantaneousPower
    false,                 // crankDataPresent
    0,                     // cumulativeCrankRevs (ignoriert)
    0,                     // lastCrankEventTime (ignoriert)
    0.0f,                  // maxTorqueNm (ignoriert)
    0.0f,                  // minTorqueNm (ignoriert)
    0,                     // tdsAngleDeg (ignoriert)
    0,                     // bdsAngleDeg (ignoriert)
    accumulatedEnergy_kJ,  // Energie bleibt erhalten
    false,
    false
  );



  // Nur Torque-Akkumulation zurücksetzen
  sumTorqueNm = 0.0f;
  torqueSampleCount = 0.0f;
}

void handleRevolution(uint32_t revolutionUs) {

  // 1) Kadenz & Leistung
  CadencePowerCalc(revolutionUs);

  // 2) Vector-Metriken
  float maxTorqueNm = 0.0f;
  float minTorqueNm = 0.0f;
  uint16_t tdsAngleDeg = 0;
  uint16_t bdsAngleDeg = 0;

  computeCpsMetricsForRevolution(
    maxTorqueNm,
    minTorqueNm,
    tdsAngleDeg,
    bdsAngleDeg,
    accumulatedEnergy_kJ,
    (int16_t)instantaneousPower,
    revolutionUs,
    lastRevUs,
    torqueVector,
    vectorCount,
    firstSampleAngle,
    VECTOR_ANGLE_STEP_DEG
  );

  // 3) crankDataPresent-Logik
  bool crankDataPresent = (estimatedCadenceRpm > 5.0f);

  // 4) CPS-Messung senden
  bool haveVectorMetrics = (vectorCount > 0);  // oder: maxTorqueNm/minTorqueNm sinnvoll

sendCyclingPowerMeasurement_CPS(
  (int16_t)instantaneousPower,
  crankDataPresent,
  cumulativeCrankRevs,
  lastCrankEventTime,
  maxTorqueNm,
  minTorqueNm,
  tdsAngleDeg,
  bdsAngleDeg,
  accumulatedEnergy_kJ,
  false,
  haveVectorMetrics
);


  // 5) Vector-Buffer für nächste Umdrehung leeren
  vectorCount = 0;
  firstSampleAngle = 0;
  lastVectorSampleAngle = -1.0f;
}
