#include <Arduino_LSM9DS1.h>
#include <PDM.h>
#include <ArduinoBLE.h>
#include <Fuzzy.h>

// BLE UUID 설정
BLEService accelService("1101");
BLECharacteristic hitChar("2101", BLERead | BLENotify, 20);

#define SAMPLE_WINDOW 20
#define SAMPLE_INTERVAL_MS 20

short micBuffer[256];
volatile int micSamplesRead;
float axBuffer[SAMPLE_WINDOW], ayBuffer[SAMPLE_WINDOW], azBuffer[SAMPLE_WINDOW];
float gxBuffer[SAMPLE_WINDOW], gyBuffer[SAMPLE_WINDOW], gzBuffer[SAMPLE_WINDOW];
int sampleIndex = 0;
unsigned long lastSampleTime = 0;
unsigned long relativeTime = 0;
int hitCount = 0;

// 퍼지 시스템 생성
Fuzzy* fuzzy = new Fuzzy();

FuzzySet *micLow = new FuzzySet(0.0, 0.02, 0.03, 0.05);
FuzzySet *micHigh = new FuzzySet(0.04, 0.07, 0.08, 0.1);

FuzzySet *accLow = new FuzzySet(0.0, 0.2, 0.3, 0.4);
FuzzySet *accHigh = new FuzzySet(0.35, 0.45, 0.6, 0.7);

FuzzySet *gyroLow = new FuzzySet(0.0, 0.05, 0.08, 0.12);
FuzzySet *gyroHigh = new FuzzySet(0.1, 0.15, 0.2, 0.25);

FuzzySet *outputLow = new FuzzySet(0, 20, 30, 40);
FuzzySet *outputHigh = new FuzzySet(50, 60, 80, 100);

void setupFuzzyLogic() {
  FuzzyInput* micInput = new FuzzyInput(1);
  micInput->addFuzzySet(micLow);
  micInput->addFuzzySet(micHigh);
  fuzzy->addFuzzyInput(micInput);

  FuzzyInput* accInput = new FuzzyInput(2);
  accInput->addFuzzySet(accLow);
  accInput->addFuzzySet(accHigh);
  fuzzy->addFuzzyInput(accInput);

  FuzzyInput* gyroInput = new FuzzyInput(3);
  gyroInput->addFuzzySet(gyroLow);
  gyroInput->addFuzzySet(gyroHigh);
  fuzzy->addFuzzyInput(gyroInput);

  FuzzyOutput* riskOutput = new FuzzyOutput(1);
  riskOutput->addFuzzySet(outputLow);
  riskOutput->addFuzzySet(outputHigh);
  fuzzy->addFuzzyOutput(riskOutput);

  // Rule: IF micHigh AND (accHigh OR gyroHigh) THEN outputHigh
  FuzzyRuleAntecedent* accHighOrGyroHigh = new FuzzyRuleAntecedent();
  accHighOrGyroHigh->joinWithOR(accHigh, gyroHigh);

  FuzzyRuleAntecedent* micAndMotion = new FuzzyRuleAntecedent();
  micAndMotion->joinWithAND(micHigh, accHighOrGyroHigh);

  FuzzyRuleConsequent* thenHit = new FuzzyRuleConsequent();
  thenHit->addOutput(outputHigh);

  FuzzyRule* rule1 = new FuzzyRule(1, micAndMotion, thenHit);
  fuzzy->addFuzzyRule(rule1);
}

// RMS 계산
float calcRMS(float* x, float* y, float* z, int N, float t) {
  float sum = 0.0;
  for (int i = 0; i < N; i++) {
    sum += x[i] * x[i] + y[i] * y[i] + z[i] * z[i];
  }
  float val = sqrt(sum / (3.0 * N));
  return (val >= t) ? 1.0 : val / t;
}

float calcMicRMS(short* buffer, int N, float t) {
  float sum = 0.0;
  for (int i = 0; i < N; i++) {
    sum += buffer[i] * buffer[i];
  }
  float val = sqrt(sum / (float)N);
  return (val >= t) ? 1.0 : val / t;
}

void setup() {
  Serial.begin(115200);
  //while (!Serial);
  //↑ maybe this blocks the bluetooth when i connected the power bank


  if (!IMU.begin()) {
    Serial.println("IMU 초기화 실패");
    while (1);
  }

  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, 16000)) {
    Serial.println("PDM 마이크 시작 실패");
    while (1);
  }

  if (!BLE.begin()) {
    Serial.println("BLE 시작 실패");
    while (1);
  }

  BLE.setLocalName("Nano33BLE_Hit");
  BLE.setAdvertisedService(accelService);
  accelService.addCharacteristic(hitChar);
  BLE.addService(accelService);
  hitChar.writeValue("Ready");
  BLE.advertise();

  setupFuzzyLogic();
  lastSampleTime = millis();
  Serial.println("timestamp,accRMS,gyroRMS,micRMS");
}

void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected: ");
    Serial.println(central.address());

    while (central.connected()) {
      senseAndSendHit(central);
    }

    Serial.println("Disconnected");
    BLE.advertise();
  }
}

void senseAndSendHit(BLEDevice central) {
  float ax, ay, az, gx, gy, gz;
  if (millis() - lastSampleTime < SAMPLE_INTERVAL_MS) return;
  lastSampleTime = millis();
  relativeTime += SAMPLE_INTERVAL_MS;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    axBuffer[sampleIndex] = ax;
    ayBuffer[sampleIndex] = ay;
    azBuffer[sampleIndex] = az;
    gxBuffer[sampleIndex] = gx;
    gyBuffer[sampleIndex] = gy;
    gzBuffer[sampleIndex] = gz;
    sampleIndex++;
  }

  if (sampleIndex >= SAMPLE_WINDOW && micSamplesRead > 0) {
    float accRMS = calcRMS(axBuffer, ayBuffer, azBuffer, SAMPLE_WINDOW, 2.5);
    float gyroRMS = calcRMS(gxBuffer, gyBuffer, gzBuffer, SAMPLE_WINDOW, 1000.0);
    float micRMS = calcMicRMS(micBuffer, micSamplesRead, 8000.0);

    fuzzy->setInput(1, micRMS);
    fuzzy->setInput(2, accRMS);
    fuzzy->setInput(3, gyroRMS);
    fuzzy->fuzzify();
    float fuzzyOutput = fuzzy->defuzzify(1);

    Serial.print(relativeTime); Serial.print(",");
    Serial.print(accRMS, 4); Serial.print(",");
    Serial.print(gyroRMS, 4); Serial.print(",");
    Serial.print(micRMS, 4); Serial.print(",");
    Serial.println(fuzzyOutput);



/////////////////////////////fuzzy logic value///////////////////////////////
    if (fuzzyOutput >= 5.0) {
      hitCount++;
      Serial.print("Hit Detected: "); Serial.println(hitCount);
      if (central.connected()) {
        char buf[20];
        snprintf(buf, sizeof(buf), "Hit %d", hitCount);
        hitChar.writeValue(buf);
      }
    }

    sampleIndex = 0;
    micSamplesRead = 0;
  }
}

void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(micBuffer, bytesAvailable);
  micSamplesRead = bytesAvailable / 2;
}
