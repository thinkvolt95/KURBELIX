void configureActiveIMU() {
  // Active riding only needs Z gyro data for cadence detection.
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

// Gewichtung nach Winkelzone (90°/270° stark, 0°/180° schwach)
float angleWeight(float angleDeg) {
  float a = wrapAngle(angleDeg);

  float d90  = fabs(a - 90.0f);
  float d270 = fabs(a - 270.0f);
  float d = fmin(d90, d270);

  const float INNER = 10.0f;   // volle Korrektur
  const float OUTER = 35.0f;   // keine Korrektur

  if (d >= OUTER) return 0.0f;
  if (d <= INNER) return 1.0f;

  float t = (d - INNER) / (OUTER - INNER);
  return 1.0f - t;
}

// Gewichtung nach Kadenz (viel bei 40 rpm, wenig bei 100 rpm)
float cadenceWeight(float cadence_rpm) {
  const float F_LOW  = 40.0f;
  const float F_HIGH = 100.0f;

  if (cadence_rpm <= F_LOW)  return 1.0f;
  if (cadence_rpm >= F_HIGH) return 0.0f;

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
  // ---------- 1. Init ----------
  if (!ce_initialized) {

    ce_initialized = true;
    ce_lastMicros = sampleMicros;

    // Absoluter Startwinkel aus Accel (echte 12-Uhr-Referenz)
    float accelAngle = atan2(accY, accX) * 180.0f / PI + 90.0f;
    accelAngle = wrapAngle(accelAngle);

    ce_angleDeg = accelAngle;
    ce_gyroBias_degps = gyroBiasZ_dps;

    currentCrankAngle = ce_angleDeg;
    revolutionTimestampUs = 0;
    lastRevUs = 0;

    return false;
  }

  // ---------- 2. dt ----------
  int32_t dUs = (int32_t)(sampleMicros - ce_lastMicros);
  if (dUs <= 0 || dUs > 2000000) {
    ce_lastMicros = sampleMicros;
    return false;
  }
  float dt = dUs * 1e-6f;
  ce_lastMicros = sampleMicros;

  // ---------- 3. Gyro-Rate ----------
  float gyroRate = degZ - ce_gyroBias_degps;

  // ---------- 4. Gyro-Integration ----------
  float prevAngle = ce_angleDeg;
  ce_angleDeg += gyroRate * dt;
  ce_angleDeg = wrapAngle(ce_angleDeg);

  // ---------- 5. Accel-Korrektur ----------
  float accelAngle = atan2(accY, accX) * 180.0f / PI + 90.0f;
  accelAngle = wrapAngle(accelAngle);

  float diff = accelAngle - ce_angleDeg;
  if (diff > 180.0f) diff -= 360.0f;
  if (diff < -180.0f) diff += 360.0f;

  float wAngle   = angleWeight(ce_angleDeg);
  float wCadence = cadenceWeight(estimatedCadenceRpm);
  float wTotal   = wAngle * wCadence;

  if (wTotal > 0.0f) {
    ce_angleDeg = wrapAngle(ce_angleDeg + diff * (0.01f * wTotal));
  }

  currentCrankAngle = ce_angleDeg;

  // ---------- 6. Bias-Update im Stillstand ----------
  if (fabs(gyroRate) < 1.0f &&
      lastRevUs != 0 &&
      (sampleMicros - lastRevUs) > 2000000UL)
  {
    ce_gyroBias_degps =
        ce_gyroBias_degps * (1.0f - CE_BIAS_ALPHA) +
        degZ          * CE_BIAS_ALPHA;
  }

  // ---------- 7. Revolutionserkennung ----------
  bool revolutionDetected = false;

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
