void configureActiveIMU() {
  // Active riding only needs Z gyro data for cadence detection.
  myIMU.settings.gyroEnabled = 1;
  myIMU.settings.gyroSampleRate = 104;
  myIMU.settings.gyroFifoEnabled = 0;
  myIMU.settings.gyroFifoDecimation = 0;

  // Activate Acceleration sensor
  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = 2;
  myIMU.settings.accelSampleRate = 104;
  myIMU.settings.accelFifoEnabled = 0;
  myIMU.settings.accelFifoDecimation = 0;

  myIMU.settings.tempEnabled = 0;
  myIMU.settings.fifoModeWord = 0;
  myIMU.settings.fifoSampleRate = 0;
}

// Integrate filtered gyro magnitude and emit one event per full crank rotation.
bool detectRevolution(uint32_t sampleMicros, float degZ)
{
  static float angle = 0.0f;
  static uint32_t lastMicros = 0;

  static float lowPassedGyroRate = 0.0f;  // variable for low pass filter
  const float alpha = 0.20f;              // low pass filter coefficent (0.20 = good start value)
  const float deadband = 3.0f;
  
  // Initialization
  if (lastMicros == 0) { 
    lastMicros = sampleMicros; 
    return false; 
  }
  
  // Calculate time difference
  uint32_t dtUs = sampleMicros - lastMicros;
  float dt = dtUs / 1000000.0f;
  lastMicros = sampleMicros;

  // 1. Correct gyro rate with offset
  float gyroRate = degZ - gyroBiasZ_dps;

  // Low pass filter
  if (lowPassedGyroRate == 0.0f && fabs(gyroRate) > 0.1f) {
    lowPassedGyroRate = gyroRate; 
  } else {
    lowPassedGyroRate = (alpha * gyroRate) + ((1.0f - alpha) * lowPassedGyroRate);
  }

  // 2. drift compensation using acceleration sensor
  float accX = myIMU.readFloatAccelX();
  float accY = myIMU.readFloatAccelY();
  float accelAngleDeg = (atan2(accY, accX) * 180.0f / PI) + 90.0f;
  if (accelAngleDeg < 0.0f) accelAngleDeg += 360.0f;

  logPrint(" accelAngleDeg=");
  logPrint(accelAngleDeg, 2);

  // shortest way for compensation
  float angleDiff = accelAngleDeg - angle;
  if (angleDiff > 180.0f) angleDiff -= 360.0f;
  if (angleDiff < -180.0f) angleDiff += 360.0f;

  // Combine intergral of gyro with accel angle
  // filtering with deadband filter
  float integratedDegZ = (fabs(lowPassedGyroRate) < deadband) ? 0.0f : (lowPassedGyroRate * dt);
  
  angle += integratedDegZ + (0.005f * angleDiff);

  // angle normalisation
  if (angle >= 360.0f) angle -= 360.0f;
  if (angle < 0.0f)    angle += 360.0f;
  
  currentCrankAngle = angle;

  float deltaAngle = lowPassedGyroRate * dt;

  if (angle > 350.0f && (angle + deltaAngle) >= 360.0f) {
     float overshoot = (angle + deltaAngle) - 360.0f;
     float fraction = (deltaAngle - overshoot) / deltaAngle;
     
     // calculate time stamp for revolution
     revolutionTimestampUs = sampleMicros - (uint32_t)(dt * fraction * 1000000.0f);
     return true;
  }

  return false;
}