/*
  KURBELIX v3.0
  - HX711 interrupt-driven torque sensor
  - BLE Cycling Power Service + UART calibration/tare
  - Cadence from LSM6DS3 gyro integration
  - Wake-on-motion to exit SYSTEMOFF


  Development Version with:
    BLE Power Vector Vector (switchable with flag and via UART command "v")
    Advanced BLE Feautures:
      Dead Spot Angles
      Extreme Torque Magnitudes
      Accumulated Energy

 */


// --- Feature flags ---
bool enablePowerVector = true;  // oder false als Default

#include <bluefruit.h>
#include <Arduino.h>
#include <Wire.h>
#include "HX711.h"
#include <LSM6DS3.h>
#include "nrf_gpio.h"
#include <nrf_sdm.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <stdint.h>

using namespace Adafruit_LittleFS_Namespace;

#define CAL_FILE "/calibration.txt"
#define TARE_FILE "/tare.txt"
#define GYRO_TARE_FILE "/gyro_tare.txt"
#define CRANK_LENGTH_FILE "/crank_length.txt"
#define GARMIN_OFFSET_REF_FILE "/garmin_offset_ref.txt"

File calFile(InternalFS);
File tareFile(InternalFS);
File gyroTareFile(InternalFS);

// ---------- config ----------
#define DEFAULT_CRANK_LENGTH_M 0.1725f
#define DEFAULT_CRANK_LENGTH_HALF_MM 345U
#define MIN_POWER_CADENCE_RPM 25.0f
#define MAX_VALID_POWER_W 2000.0f
#define SLEEP_TIMEOUT_MS 60000UL
#define TARE_SAMPLES 200
#define CAL_SAMPLES 200
#define NUM_BATT_SAMPLES 10
#define BATTERY_UPDATE_INTERVAL 12000UL

// ---------- pins ----------
#define HX_DT 2
#define HX_SCK 3
#define MOTION_INT_PIN PIN_LSM6DS3TR_C_INT1

// Battery state
unsigned long lastBattMs = 0;
uint8_t battPercent = 100;

// IMU
LSM6DS3 myIMU(I2C_MODE, 0x6A);
float gyroBiasY_dps = 0.0f;
float gyroBiasZ_dps = 0.0f;
float lastGyroZ_dps = 0.0f;
float correctedGyroZ_dps = 0.0f;

// Measurement state
float sumTorqueNm = 0.0f;
float torqueSampleCount = 0.0f;
uint32_t lastRevUs = 0;
uint32_t cumulativeCrankRevs = 0;
uint32_t previousCadenceRevUs = 0;
uint16_t lastCrankEventTime = 0;
uint32_t revolutionTimestampUs = 0;
volatile bool calibrationActive = false;
float powerW = 0.0f;

// Cadence Engine intern
static bool ce_initialized = false;
static uint32_t ce_lastMicros = 0;
static float ce_angleDeg = 0.0f;
static float ce_gyroBias_degps = 0.0f;
static const float CE_BIAS_ALPHA = 0.00002f;  // sehr langsame Bias-Anpassung
float wrapAngle(float a);
float angleWeight(float angleDeg);
float cadenceWeight(float cadence_rpm);
float estimatedCadenceRpm = 0.0f;

// --- Global variables for dead center weighting & Garmin metrics ---
float currentCrankAngle = 0.0f;

// -- variables for power vector
#define MAX_VECTOR_SAMPLES 6
static int16_t torqueVector[MAX_VECTOR_SAMPLES];
static uint8_t vectorCount = 0;
static uint16_t firstSampleAngle = 0;
static float lastVectorSampleAngle = -1.0f;
static const float VECTOR_ANGLE_STEP_DEG = 360.0f / MAX_VECTOR_SAMPLES;
void recordVectorSample(float torqueNm, float crankAngleDeg);
void sendCyclingPowerVector(int16_t* torqueArray, uint8_t arraySize, uint16_t firstAngle);

// Extreme Torque Magnitudes, Accumulated Energy & Dead Spots
float instantaneousPower = 0.0f;
float accumulatedEnergy_kJ = 0.0f;

float maxTorqueNm = 0.0f;
float minTorqueNm = 0.0f;

uint16_t tdsAngleDeg = 0;
uint16_t bdsAngleDeg = 0;

// HX711
HX711 scale;
long zeroOffsetCounts = 0;
long garminOffsetReferenceCounts = 0;
bool garminOffsetReferenceValid = false;
float scaleFactor_counts_per_N = NAN;
volatile bool hx711ReadyFlag = false;
uint16_t crankLengthHalfMm = DEFAULT_CRANK_LENGTH_HALF_MM;
float crankLengthM = DEFAULT_CRANK_LENGTH_M;

// Function prototypes
void hx711ISR();
bool detectRevolution(uint32_t sampleMicros, float degZ, float accX, float accY);
void accumulateTorque(float torqueNm, float correctedGyroZ_dps);
bool shouldAccumulateTorqueSample(float torqueNm, float correctedGyroZ_dps);
void configureActiveIMU();
void doTareGyro();
void setupBLE();
void serviceCyclingPowerControlPoint();
void doTare(bool updateGarminReference);
int16_t doGarminOffsetCompensation();
void runCalibration(float knownMassKg);
void sendCyclingPowerMeasurement(int16_t powerWatts,
                                 bool crankDataPresent,
                                 uint16_t cumulativeCrankRevs,
                                 uint16_t lastCrankEventTime,
                                 bool offsetCompActive);
void sendCyclingPowerMeasurement_CPS(int16_t instantaneousPower,
                                     bool crankDataPresent,
                                     uint16_t cumulativeCrankRevs,
                                     uint16_t lastCrankEventTime,
                                     float maxTorqueNm,
                                     float minTorqueNm,
                                     uint16_t tdsAngleDeg,
                                     uint16_t bdsAngleDeg,
                                     float accumulatedEnergy_kJ,
                                     bool offsetCompActive,
                                     bool haveVectorMetrics);
void goToSystemOff();
void batteryCheckAndLED();
void CadencePowerCalc(uint32_t revolutionUs);
void loadFlashValues();
void saveCalibration();
void saveTare();
void saveGyroTare();
void saveCrankLength();
void saveGarminOffsetReference();
bool setCrankLengthHalfMm(uint16_t halfMm, bool persist);
int16_t getGarminDisplayedOffset();
void ensureGarminOffsetReference();
void uartRXWriteCallback(uint16_t conn_hdl,
                         BLECharacteristic* chr,
                         uint8_t* data,
                         uint16_t len);
void serviceUARTCommands();
void processUARTCommand(String cmd);
void setupWakeUpInterrupt();
void sendZeroPowerIfStopped();

void logPrint(const String& msg);
void logPrintln(const String& msg);
void logPrint(const char* msg) {
  logPrint(String(msg));
}
void logPrintln(const char* msg) {
  logPrintln(String(msg));
}
void logPrint(long v) {
  logPrint(String(v));
}
void logPrintln(long v) {
  logPrintln(String(v));
}
void logPrint(unsigned long v) {
  logPrint(String(v));
}
void logPrintln(unsigned long v) {
  logPrintln(String(v));
}
void logPrint(int v) {
  logPrint(String(v));
}
void logPrintln(int v) {
  logPrintln(String(v));
}
void logPrint(float v, int digits = 2) {
  logPrint(String(v, digits));
}
void logPrintln(float v, int digits = 2) {
  logPrintln(String(v, digits));
}

void cpControlPointWriteCallback(uint16_t conn_hdl,
                                 BLECharacteristic* chr,
                                 uint8_t* data,
                                 uint16_t len);

void setup() {
  logPrintln("KURBELIX v3.0 start");
  Wire.begin();
  
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);  

  // Battery measurement and charge-current pin setup for the XIAO board.
  analogReadResolution(12);
  analogReference(AR_INTERNAL_2_4);
  pinMode(PIN_CHARGING_CURRENT, OUTPUT);
  digitalWrite(PIN_CHARGING_CURRENT, LOW);
  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, LOW);
  pinMode(PIN_VBAT, INPUT);

  // HX711 runs from the data-ready interrupt; keep SCK low when active.
  pinMode(HX_SCK, OUTPUT);
  digitalWrite(HX_SCK, LOW);
  pinMode(HX_DT, INPUT_PULLUP);
  scale.begin(HX_DT, HX_SCK);
  delay(75);
  attachInterrupt(digitalPinToInterrupt(HX_DT), hx711ISR, FALLING);

  delay(1000);
  logPrintln("Starting IMU...");
  configureActiveIMU();
  int imuStatus = myIMU.begin();
  logPrint("IMU begin result = ");
  logPrintln(String(imuStatus));

  if (imuStatus != 0) {
    logPrintln("IMU init failed!");
    while (1)
      ;
  }

  // Load persisted tare/calibration before starting BLE service.
  loadFlashValues();
  setupBLE();
  Bluefruit.autoConnLed(false);  // Verhindert, dass die BLE-Bibliothek die LED wieder einschaltet
  lastRevUs = micros();
  logPrintln("Ready. Commands via UART: 'c'=calib, 't'=tare, 'm <kg>'=custom calib, 'k <wert>'=set scale.");
}

void loop() {
  serviceUARTCommands();
  serviceCyclingPowerControlPoint();

  // Each HX711 sample updates torque and provides the pacing for the main loop.
  if (hx711ReadyFlag) {
    hx711ReadyFlag = false;
    long raw = scale.read();
    uint32_t sampleMicros = micros();
    float rawGyroZ_dps = -myIMU.readFloatGyroZ();
    float accX = myIMU.readFloatAccelX();
    float accY = myIMU.readFloatAccelY();
    float correctedGyroZ_dps = rawGyroZ_dps - gyroBiasZ_dps;
    long net = raw - zeroOffsetCounts;

    if (isfinite(scaleFactor_counts_per_N) && fabs(scaleFactor_counts_per_N) > 1e-6f) {
      float forceN = (float)net / scaleFactor_counts_per_N;
      float torqueNm = forceN * crankLengthM;
      if (shouldAccumulateTorqueSample(torqueNm, correctedGyroZ_dps)) {
        accumulateTorque(torqueNm, correctedGyroZ_dps);
        if (enablePowerVector) {
          recordVectorSample(torqueNm, currentCrankAngle);
        }
      }
    }
    lastGyroZ_dps = rawGyroZ_dps;
    float degZ = rawGyroZ_dps;
    if (detectRevolution(sampleMicros, degZ, accX, accY)) {
      handleRevolution(revolutionTimestampUs);
      if (enablePowerVector && vectorCount > 0) {
        sendCyclingPowerVector(torqueVector, vectorCount, firstSampleAngle);
      }
    }

    else {
      uint32_t nowUs = micros();
      bool isStopped = (lastRevUs != 0 && (nowUs - lastRevUs) > 2000000UL && fabs(correctedGyroZ_dps) < 5.0f);

      if (isStopped) {
        sendZeroPowerIfStopped();
      }
    }
  }

  if ((micros() - lastRevUs) > (SLEEP_TIMEOUT_MS * 1000UL) && !calibrationActive) {
    logPrintln("No pedaling -> deep sleep");
    goToSystemOff();
  }

  batteryCheckAndLED();
  sd_app_evt_wait();
}
