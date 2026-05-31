void hx711ISR() { hx711ReadyFlag = true; }

static float crankLengthHalfMmToMeters(uint16_t halfMm) {
  return ((float)halfMm * 0.5f) / 1000.0f;
}

long averageCounts(int n, unsigned long timeoutMs) {
  long sum = 0; int k=0;
  unsigned long t0 = millis();
  while (k<n && (millis()-t0)<timeoutMs) {
    if (scale.is_ready()) { sum+=scale.read(); k++; }
  }
  return (k>0) ? (sum/k) : 0L;
}

void accumulateTorque(float torqueNm) {
  sumTorqueNm += torqueNm;
  torqueSampleCount++;
}

