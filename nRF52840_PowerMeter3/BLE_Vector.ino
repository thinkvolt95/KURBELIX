void sendCyclingPowerVector(int16_t* torqueArray, uint8_t arraySize, uint16_t firstAngle) {
  if (arraySize > 6) arraySize = 6; 

  uint8_t buf[19] = { 0 };

  // Byte 0: Flags (According to table 3.7: Bit 0=1, Bit 1=1, Bit 3=1 -> 0x0B)
  buf[0] = 0x0B;

  // Byte 1-2: Cumulative Crank Revolutions (16-Bit)
  buf[1] = cumulativeCrankRevs & 0xFF;
  buf[2] = (cumulativeCrankRevs >> 8) & 0xFF;

  // Byte 3-4: Last Crank Event Time (16-Bit, 1/1024s)
  uint16_t eventTime = (uint16_t)(((uint64_t)micros() * 1024ULL) / 1000000ULL);
  buf[3] = eventTime & 0xFF;
  buf[4] = (eventTime >> 8) & 0xFF;

  // Byte 5-6: First Crank Measurement Angle (16-Bit, 1° resolution)
  buf[5] = firstAngle & 0xFF;
  buf[6] = (firstAngle >> 8) & 0xFF;

  // Byte 7-18: Instantaneous Torque Magnitude Array
  for (uint8_t i = 0; i < arraySize; i++) {
    buf[7 + (i * 2)] = torqueArray[i] & 0xFF;
    buf[8 + (i * 2)] = (torqueArray[i] >> 8) & 0xFF;
  }

  // Send only if Garmin can handle the values (CCCD = 0x0001)
  if (cpVectorChar.notifyEnabled()) {
    cpVectorChar.notify(buf, 7 + (arraySize * 2));
  }

  vectorCount = 0;
}

void recordVectorSample(float torqueNm) {
  if (vectorCount < MAX_VECTOR_SAMPLES) {
    // According to specs: Unit is Newton-meters with a resolution of 1/32 Nm
    torqueVector[vectorCount] = (int16_t)(torqueNm * 32.0f);
    vectorCount++;
  }
}