bool shouldAccumulateTorqueSample(float torqueNm, float correctedGyroZ_dps) {

  // 1. Reject reverse pedaling or noise at dead centers
  // if (torqueNm < -2.0f) return false;

  // 2. Gyroscope must indicate forward rotation
  if (correctedGyroZ_dps < 2.0f) return false;

  // 3. Compute angular velocity and check for valid float representation
  float angVel = correctedGyroZ_dps * (PI / 180.0f);
  if (!isfinite(angVel)) return false;

  // 4. Avoid noise at dead centers (ignores low cadence below ~10 rpm)
  if (angVel < 1.0f) return false;

  // 5. Filter out unrealistic torque spikes
  if (fabs(torqueNm) > 120.0f) return false;

  // 6. Filter out unrealistic power spikes
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
  float powerW = 0.0f;

  // Handle micros() wrap-around: if the current timestamp is less than or equal
  // to the previous one, resynchronize the timer and ignore this revolution
  // for cadence/power calculations.
  if (previousCadenceRevUs != 0 && nowUs <= previousCadenceRevUs) {
    previousCadenceRevUs = nowUs;
    // Reset the torque averaging window anyway to prevent value accumulation overflow
    sumTorqueNm = 0.0f;
    torqueSampleCount = 0.0f;
    return;
  }

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

  // Sanity check for negative power!
  if (powerW < 0.0f) {
    powerW = 0.0f;
  }

  previousCadenceRevUs = nowUs;

  cumulativeCrankRevs++;

  // BLE Cycling Power uses 1/1024 second event timestamps.
  lastCrankEventTime = (uint16_t)(((uint64_t)nowUs * 1024ULL) / 1000000ULL);

  bool crankDataPresent = (estimatedCadenceRpm > 5.0f);

  sendCyclingPowerMeasurement(
    (int16_t)powerW,
    crankDataPresent,
    cumulativeCrankRevs,
    lastCrankEventTime,
    false);

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

  // Minimal CPS (Cycling Power Service): Power=0, no crank data present
  sendCyclingPowerMeasurement(
    0,      // instantaneousPower
    false,  // crankDataPresent
    0,      // cumulativeCrankRevs (ignoriert)
    0,      // lastCrankEventTime (ignoriert)
    false);

  // Only reset the torque accumulation window
  sumTorqueNm = 0.0f;
  torqueSampleCount = 0.0f;
}
