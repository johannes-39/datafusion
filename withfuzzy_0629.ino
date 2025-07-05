#include <Arduino_LSM9DS1.h>
#include <PDM.h>
#include <ArduinoBLE.h>

#define SAMPLE_WINDOW 20
#define SAMPLE_INTERVAL_MS 20

// ▲ 감도 낮춘 퍼지 임계값
float acc_low_a = 0.1,  acc_low_b = 0.2,  acc_low_c = 0.3,  acc_low_d = 0.4;
float acc_med_a = 0.35, acc_med_b = 0.45, acc_med_c = 0.55, acc_med_d = 0.65;
float acc_high_a = 0.6, acc_high_b = 0.7, acc_high_c = 1.0, acc_high_d = 1.0;

float gyro_low_a = 0.05, gyro_low_b = 0.1,  gyro_low_c = 0.15, gyro_low_d = 0.2;
float gyro_med_a = 0.18, gyro_med_b = 0.25, gyro_med_c = 0.3,  gyro_med_d = 0.35;
float gyro_high_a = 0.32, gyro_high_b = 0.4, gyro_high_c = 1.0, gyro_high_d = 1.0;

float mic_low_a = 0.02, mic_low_b = 0.03, mic_low_c = 0.045, mic_low_d = 0.06;
float mic_med_a = 0.05, mic_med_b = 0.06, mic_med_c = 0.08, mic_med_d = 0.1;
float mic_high_a = 0.09, mic_high_b = 0.11, mic_high_c = 1.0, mic_high_d = 1.0;

// 버퍼
short micBuffer[256];
volatile int micSamplesRead;
float axBuffer[SAMPLE_WINDOW], ayBuffer[SAMPLE_WINDOW], azBuffer[SAMPLE_WINDOW];
float gxBuffer[SAMPLE_WINDOW], gyBuffer[SAMPLE_WINDOW], gzBuffer[SAMPLE_WINDOW];

int sampleIndex = 0;
unsigned long lastSampleTime = 0;
unsigned long relativeTime = 0;
int hitCount = 0;

// BLE 설정
BLEService accelService("1101");
BLECharacteristic hitChar("2101", BLERead | BLENotify, 20);

// 트라페조이드 퍼지 함수
float trapezoidMF(float x, float a, float b, float c, float d) {
  if (x <= a || x >= d) return 0.0;
  else if (x >= b && x <= c) return 1.0;
  else if (x > a && x < b) return (x - a) / (b - a);
  else if (x > c && x < d) return (d - x) / (d - c);
  return 0.0;
}

// RMS 계산
float calcRMS(float *x, float *y, float *z, int N, float t) {
  float sum = 0.0;
  for (int i = 0; i < N; i++) {
    sum += x[i] * x[i] + y[i] * y[i] + z[i] * z[i];
  }
  float returnValue = sqrt(sum / (3.0 * N));
  return (returnValue >= t) ? 1.0 : returnValue / t;
}

float calcMicRMS(short *buffer, int N, float t) {
  float sum = 0.0;
  for (int i = 0; i < N; i++) {
    sum += buffer[i] * buffer[i];
  }
  float returnValue = sqrt(sum / (float)N);
  return (returnValue >= t) ? 1.0 : returnValue / t;
}

// 퍼지 로직: 감도 낮춰서 false positive 줄이기
bool isHitFuzzy(float micRMS, float accRMS, float gyroRMS) {
  float mic_low = trapezoidMF(micRMS, mic_low_a, mic_low_b, mic_low_c, mic_low_d);
  float mic_med = trapezoidMF(micRMS, mic_med_a, mic_med_b, mic_med_c, mic_med_d);
  float mic_high = trapezoidMF(micRMS, mic_high_a, mic_high_b, mic_high_c, mic_high_d);

  float acc_med = trapezoidMF(accRMS, acc_med_a, acc_med_b, acc_med_c, acc_med_d);
  float acc_high = trapezoidMF(accRMS, acc_high_a, acc_high_b, acc_high_c, acc_high_d);

  float gyro_med = trapezoidMF(gyroRMS, gyro_med_a, gyro_med_b, gyro_med_c, gyro_med_d);
  float gyro_high = trapezoidMF(gyroRMS, gyro_high_a, gyro_high_b, gyro_high_c, gyro_high_d);

  bool mic_trigger = (mic_med > 0.5) || (mic_high > 0.5);
  bool motion_trigger = (acc_med > 0.5) || (acc_high > 0.5) || (gyro_med > 0.5) || (gyro_high > 0.5);

  return mic_trigger && motion_trigger;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to start PDM microphone!");
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
  Serial.println("BLE Advertising gestartet");

  Serial.println("timestamp,accRMS,gyroRMS,micRMS");
  lastSampleTime = millis();
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()) {
      senseAndSendHit(central);
    }

    Serial.println("Disconnected from central");
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
    float micRMS  = calcMicRMS(micBuffer, micSamplesRead, 8000.0);

    Serial.print(relativeTime); Serial.print(',');
    Serial.print(accRMS, 4); Serial.print(',');
    Serial.print(gyroRMS, 4); Serial.print(',');
    Serial.print(micRMS, 4); Serial.print(',');

    if (isHitFuzzy(micRMS, accRMS, gyroRMS)) {
      hitCount++;
      Serial.println("1");
      Serial.println(hitCount);

      if (central.connected()) {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Hit %d", hitCount);
        hitChar.writeValue(buffer);
      }
    } else {
      Serial.println("0");
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
