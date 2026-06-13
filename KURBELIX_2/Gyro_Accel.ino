// ---------- IMU / Cadence Engine ----------
void configureActiveIMU() {
  myIMU.settings.gyroEnabled = 1;
  myIMU.settings.gyroSampleRate = 208;
  myIMU.settings.gyroFifoEnabled = 0;
  myIMU.settings.gyroFifoDecimation = 0;

  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = 4;
  myIMU.settings.accelSampleRate = 208;
  myIMU.settings.accelFifoEnabled = 0;
  myIMU.settings.accelFifoDecimation = 0;

  myIMU.settings.tempEnabled = 0;
  myIMU.settings.fifoModeWord = 0;
  myIMU.settings.fifoSampleRate = 0;
}

// Weighting based on angular zone (strong weight at 90°/270°, weak weight at 0°/180°)
float angleWeight(float angleDeg) {
  float a = wrapAngle(angleDeg);

  // Calculate shortest distance to the power phases (90° and 270°)
  float d90  = fabs(a - 90.0f);
  float d270 = fabs(a - 270.0f);
  float d = fmin(d90, d270);

  const float INNER = 10.0f;   // Full weight/correction boundary
  const float OUTER = 35.0f;   // Zero weight/correction boundary

  // Linear interpolation (fade out) between INNER and OUTER thresholds
  if (d >= OUTER) return 0.0f;
  if (d <= INNER) return 1.0f;

  float t = (d - INNER) / (OUTER - INNER);
  return 1.0f - t;
}

// Weighting based on cadence (high weight at 40 rpm, low weight at 100 rpm)
float cadenceWeight(float cadence_rpm) {
  const float F_LOW  = 40.0f;
  const float F_HIGH = 100.0f;

  // Full weight at or below F_LOW, zero weight at or above F_HIGH
  if (cadence_rpm <= F_LOW)  return 1.0f;
  if (cadence_rpm >= F_HIGH) return 0.0f;

  // Linear interpolation (fade out) between F_LOW and F_HIGH
  float t = (cadence_rpm - F_LOW) / (F_HIGH - F_LOW);
  return 1.0f - t;
}

float wrapAngle(float a) {
  while (a >= 360.0f) a -= 360.0f;
  while (a < 0.0f) a += 360.0f;
  return a;
}


// Integrate filtered gyro magnitude and emit one event per full crank rotation.
bool detectRevolution(uint32_t sampleMicros, float degZ, float accX, float accY)
{
  // ---------- 1. Initialization ----------
  if (!ce_initialized) {

    ce_initialized = true;
    ce_lastMicros = sampleMicros;

    // Absolute start angle from accelerometer (true 12-o'clock reference)
    float accelAngle = atan2(accY, accX) * 180.0f / PI + 90.0f;
    accelAngle = wrapAngle(accelAngle);

    ce_angleDeg = accelAngle;
    ce_gyroBias_degps = gyroBiasZ_dps;

    currentCrankAngle = ce_angleDeg;
    revolutionTimestampUs = 0;
    lastRevUs = 0;

    return false;
  }

  // ---------- 2. Delta Time (dt) ----------
  int32_t dUs = (int32_t)(sampleMicros - ce_lastMicros);
  if (dUs <= 0 || dUs > 2000000) {
    ce_lastMicros = sampleMicros;
    return false;
  }
  float dt = dUs * 1e-6f;
  ce_lastMicros = sampleMicros;

  // ---------- 3. Gyro Rate Calculation ----------
  float gyroRate = degZ - ce_gyroBias_degps;

  // ---------- 4. Gyro Integration ----------
  float prevAngle = ce_angleDeg;
  ce_angleDeg += gyroRate * dt;
  ce_angleDeg = wrapAngle(ce_angleDeg);

  // ---------- 5. Accelerometer Correction ----------
  float accelAngle = atan2(accY, accX) * 180.0f / PI + 90.0f;
  accelAngle = wrapAngle(accelAngle);

  // Compute shortest angular distance (handle 360-degree wrap-around)
  float diff = accelAngle - ce_angleDeg;
  if (diff > 180.0f) diff -= 360.0f;
  if (diff < -180.0f) diff += 360.0f;

  // Apply dynamic weights based on current angle zone and cadence
  float wAngle   = angleWeight(ce_angleDeg);
  float wCadence = cadenceWeight(estimatedCadenceRpm);
  float wTotal   = wAngle * wCadence;

  // Complementary filter update if weighting criteria are met
  if (wTotal > 0.0f) {
    ce_angleDeg = wrapAngle(ce_angleDeg + diff * (0.01f * wTotal));
  }

  currentCrankAngle = ce_angleDeg;

  // ---------- 6. Gyro Bias Update at Standstill ----------
  if (fabs(gyroRate) < 1.0f &&
      lastRevUs != 0 &&
      (sampleMicros - lastRevUs) > 2000000UL)
  {
    ce_gyroBias_degps =
        ce_gyroBias_degps * (1.0f - CE_BIAS_ALPHA) +
        degZ          * CE_BIAS_ALPHA;
  }

  // ---------- 7. Revolution Detection ----------
  bool revolutionDetected = false;

  // Check for the 360° -> 0° boundary crossing (passing 12-o'clock)
  if (prevAngle > 300.0f &&
      ce_angleDeg < 60.0f &&
      gyroRate > 10.0f)
  {
    revolutionTimestampUs = sampleMicros;
    lastRevUs = sampleMicros;
    revolutionDetected = true;
  }

  return revolutionDetected;
}
