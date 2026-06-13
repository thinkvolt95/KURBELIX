void sendCyclingPowerVector(int16_t* torqueArray,
                            uint8_t arraySize,
                            uint16_t firstAngle) {
  if (!enablePowerVector) return;
  if (!cpVectorChar.notifyEnabled()) return;
  if (arraySize == 0) return;

  if (arraySize > 6) arraySize = 6;

  uint8_t buf[19] = { 0 };
    uint8_t pos = 0;

    uint16_t flags = 0;

    flags |= (1 << 0);  // Crank Revolution Data Present
    flags |= (1 << 1);  // First Crank Measurement Angle Present
    flags |= (1 << 3);  // Instantaneous Torque Magnitude Array Present

    buf[pos++] = flags & 0xFF;
    buf[pos++] = flags >> 8;

    // Crank Revolution Data
    buf[pos++] = cumulativeCrankRevs & 0xFF;
    buf[pos++] = cumulativeCrankRevs >> 8;

    buf[pos++] = lastCrankEventTime & 0xFF;
    buf[pos++] = lastCrankEventTime >> 8;

    // First Crank Measurement Angle
    buf[pos++] = firstAngle & 0xFF;
    buf[pos++] = firstAngle >> 8;

    // Torque Magnitude Array (1/32 Nm)
    for (uint8_t i = 0; i < arraySize; i++) {
        buf[pos++] = torqueArray[i] & 0xFF;
        buf[pos++] = torqueArray[i] >> 8;
    }

  cpVectorChar.notify(buf, 7 + (arraySize * 2));
}

void recordVectorSample(float torqueNm, float crankAngleDeg) {
  if (vectorCount >= MAX_VECTOR_SAMPLES) {
    return;
  }

  // Erstes Sample: Winkel merken
  if (vectorCount == 0) {
    firstSampleAngle = (uint16_t)wrapAngle(crankAngleDeg);
    lastVectorSampleAngle = crankAngleDeg;
  } else {
    float delta = crankAngleDeg - lastVectorSampleAngle;

    // Winkel auf [-180, 180] normalisieren
    if (delta > 180.0f)  delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;

    // Noch nicht weit genug gedreht → kein neues Sample
    if (fabsf(delta) < VECTOR_ANGLE_STEP_DEG * 0.7f) {
      return;
    }

    lastVectorSampleAngle = crankAngleDeg;
  }

  torqueVector[vectorCount] = (int16_t)(torqueNm * 32.0f);  // 1/32 Nm
  vectorCount++;
}

// -------------------------------------------------------------
// Compute CPS Metrics for one revolution:
// - Extreme Torque Magnitudes
// - Dead Spot Angles (TDS/BDS)
// - Accumulated Energy (kJ)
// -------------------------------------------------------------
void computeCpsMetricsForRevolution(
    float& maxTorqueNm,
    float& minTorqueNm,
    uint16_t& tdsAngleDeg,
    uint16_t& bdsAngleDeg,
    float& accumulatedEnergy_kJ,
    int16_t instantaneousPower,
    uint32_t revolutionTimestampUs,
    uint32_t lastRevUs,
    int16_t* torqueVector,
    uint8_t vectorCount,
    uint16_t firstSampleAngle,
    float angleStepDeg   // tatsächlicher Winkelabstand
) {
    // ---------------------------------------------------------
    // 1. Keine Vector-Daten → alles auf 0
    // ---------------------------------------------------------
    if (vectorCount == 0) {
        maxTorqueNm = 0.0f;
        minTorqueNm = 0.0f;
        tdsAngleDeg = 0;
        bdsAngleDeg = 0;
        return;
    }

    // ---------------------------------------------------------
    // 2. Extreme Torque Magnitudes + Dead Spot Angles
    // ---------------------------------------------------------
    maxTorqueNm = -9999.0f;
    minTorqueNm =  9999.0f;

    float minTorqueForDS = 9999.0f;

    for (uint8_t i = 0; i < vectorCount; i++) {

        float tq = torqueVector[i] / 32.0f;  // zurück in Nm

        // ECHTER Winkel: firstSampleAngle + i * angleStepDeg
        float angle = wrapAngle(firstSampleAngle + i * angleStepDeg);

        // Extremwerte
        if (tq > maxTorqueNm) maxTorqueNm = tq;
        if (tq < minTorqueNm) minTorqueNm = tq;

        // Dead Spot = minimaler Torque
        if (tq < minTorqueForDS) {
            minTorqueForDS = tq;
            tdsAngleDeg = (uint16_t)angle;
        }
    }

    // BDS = 180° versetzt
    bdsAngleDeg = wrapAngle(tdsAngleDeg + 180.0f);

    // ---------------------------------------------------------
    // 3. Accumulated Energy (kJ)
    // ---------------------------------------------------------
    float dt_s = (revolutionTimestampUs - lastRevUs) * 1e-6f;
    accumulatedEnergy_kJ += (instantaneousPower * dt_s) / 1000.0f;
}