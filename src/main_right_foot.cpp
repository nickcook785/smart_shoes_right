#include <Arduino.h>
#include "global.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "esp_sleep.h"
#include <ArduinoJson.h>  // ✅ JSON 라이브러리 추가

#define NUM_FSR 5
#define FSR1_PIN 4
#define FSR2_PIN 5
#define FSR3_PIN 6
#define FSR4_PIN 7
#define FSR5_PIN 15

#define I2C_SDA 8
#define I2C_SCL 9

Adafruit_MPU6050 mpu;
bool deviceConnected = false;
BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
BLECharacteristic* pWriteCharacteristic = nullptr;

#define SERVICE_UUID                "87654321-4321-6789-4321-0fedcba98765"
#define CHARACTERISTIC_UUID         "fedcba01-4321-6789-4321-0fedcba98765"
#define WRITE_CHARACTERISTIC_UUID   "fedcba02-4321-6789-4321-0fedcba98765"

float yawAngle = 0.0;
unsigned long prevTime = 0;
float gyroOffset = 0.0;          // 자이로 오프셋 저장용

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        deviceConnected = true;
        Serial.println("✅ BLE 클라이언트 연결됨!");
    }

    void onDisconnect(BLEServer* pServer) override {
        deviceConnected = false;
        Serial.println("🔌 BLE 클라이언트 연결 끊김. 재연결 대기 중...");
        BLEDevice::startAdvertising();
    }
};

class WriteCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) override {
        std::string value = pCharacteristic->getValue();
        if (value == "reset") {
            yawAngle = 0.0;
            Serial.println("🌀 yawAngle 수동 초기화됨 (reset 명령)");
        }
    }
};

void setupFSRPins() {
    pinMode(FSR1_PIN, INPUT);
    pinMode(FSR2_PIN, INPUT);
    pinMode(FSR3_PIN, INPUT);
    pinMode(FSR4_PIN, INPUT);
    pinMode(FSR5_PIN, INPUT);
    Serial.println("✅ FSR 센서 핀 설정 완료!");
}

float filteredYawRate = 0.0;
float alpha = 0.9;  // 저역통과 필터 계수 (0.0 ~ 1.0)

void calculateYawRate(float &yawRate) {
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
        // 오프셋을 뺀 raw 값 (deg/s)
        float rawYawRate = (g.gyro.z - gyroOffset) * 180.0 / PI;
        // 저역통과 필터
        filteredYawRate = alpha * filteredYawRate + (1 - alpha) * rawYawRate;
        yawRate = filteredYawRate;
        // 죽은 구간(Deadzone) 적용: 작은 값은 0으로 처리
        if (abs(yawRate) < 1.975) {
            yawRate = 0.0;
        }
    } else {
        Serial.println("⚠️ MPU6050 데이터 읽기 실패");
        yawRate = 0.0;
    }
}

void calibrateGyro() {
    float sum = 0.0;
    int samples = 500;  // 자이로 오프셋 보정을 위한 샘플 수
    Serial.println("🌀 자이로 오프셋 보정 시작...");

    for (int i = 0; i < samples; i++) {
        sensors_event_t a, g, temp;
        if (mpu.getEvent(&a, &g, &temp)) {
            sum += g.gyro.z;  // z축 자이로 값 누적
        } else {
            Serial.println("⚠️ MPU6050 데이터 읽기 실패 (센서 미연결일 수 있음)");
            return;
        }
        delay(2);  // 샘플링 간격 (약 2ms)
    }
    gyroOffset = sum / samples;  // 평균을 오프셋으로 저장
    Serial.print("🎯 보정된 gyroOffset: ");
    Serial.println(gyroOffset, 6);
}

float gaussianNormalize(int x, float mu, float sigma) {
    return exp(-pow(x - mu, 2) / (2 * pow(sigma, 2)));
}

String evaluateSquat(float* norm) {
    float front = (norm[2] + norm[3] + norm[4]) / 3.0;
    float center = norm[1];
    float rear = norm[0];

    if (rear > 0.65 && front < 0.3 && center >= 0.2 && center <= 0.4) {
        return "족압 분포가 고릅니다.";
    } else if (front >= 0.3) {
        return "앞쪽에 족압이 강합니다.";
    } else if (rear > 0.65) {
        return "뒤쪽 족압이 너무 강합니다.";
    } else {
        return "UNCLEAR";
    }
} 

void setup() {
    Serial.begin(115200);
    delay(2000);

    setupFSRPins();

    Wire.begin(I2C_SDA, I2C_SCL);
    if (!mpu.begin()) {
        Serial.println("❌ MPU6050 연결 실패 (연결 확인 필요)");
    } else {
        Serial.println("✅ MPU6050 초기화 완료!");
        calibrateGyro();  // 초기 오프셋 보정
    }

    BLEDevice::init("ESP32-S3 BLE right shoes");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService* pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pCharacteristic->setValue("FSR Init");

    pWriteCharacteristic = pService->createCharacteristic(
        WRITE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pWriteCharacteristic->setCallbacks(new WriteCallbacks());

    pService->start();

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);

    BLEDevice::startAdvertising();
    Serial.println("📡 BLE 서버 광고 시작");

    prevTime = millis();
}

void loop() {
    if (deviceConnected) {
        int fsrValues[NUM_FSR];
        float normalizedValues[NUM_FSR];
        float finalNormalized[NUM_FSR];
        const char* postureResults[NUM_FSR];

        // 1) FSR 읽기 및 가우시안 정규화
        fsrValues[0] = analogRead(FSR1_PIN);
        fsrValues[1] = analogRead(FSR2_PIN);
        fsrValues[2] = analogRead(FSR3_PIN);
        fsrValues[3] = analogRead(FSR4_PIN);
        fsrValues[4] = analogRead(FSR5_PIN);

        float mu = 1500.0;
        float sigma = 700.0;

        float sumNormalized = 0.0;
        for (int i = 0; i < NUM_FSR; i++) {
            normalizedValues[i] = gaussianNormalize(fsrValues[i], mu, sigma);
            sumNormalized += normalizedValues[i];

            if (fsrValues[i] > 200) postureResults[i] = "ACTIVE";
            else if (fsrValues[i] > 50) postureResults[i] = "WEAK";
            else postureResults[i] = "INACTIVE";
        }
        for (int i = 0; i < NUM_FSR; i++) {
            finalNormalized[i] = (sumNormalized > 0.0) ? (normalizedValues[i] / sumNormalized) : 0.0;
        }

        // 2) 자이로(Yaw) 속도 및 누적 각도 계산
        float yawRate;
        calculateYawRate(yawRate);
        unsigned long now = millis();
        float dt = (now - prevTime) / 1000.0;
        yawAngle += yawRate * dt;
        prevTime = now;
        if (yawAngle > 180.0) yawAngle -= 360.0;
        else if (yawAngle < -180.0) yawAngle += 360.0;

        // 3) 스쿼트 평가
        String squatPosture = evaluateSquat(finalNormalized);

        // 4) JSON 생성 및 BLE 전송
        StaticJsonDocument<700> doc;
        JsonArray fsrArray = doc.createNestedArray("fsr_right");
        JsonArray finalArray = doc.createNestedArray("final_normalized_right");
        JsonArray postureArray = doc.createNestedArray("posture_right");

        for (int i = 0; i < NUM_FSR; i++) {
            fsrArray.add(fsrValues[i]);
            finalArray.add(finalNormalized[i]);
            postureArray.add(postureResults[i]);
        }

        doc["yaw_rate"]    = yawRate;
        doc["yaw_angle"]   = yawAngle;
        doc["squat_posture"] = squatPosture;

        String jsonString;
        serializeJson(doc, jsonString);

        Serial.println(jsonString);
        pCharacteristic->setValue(jsonString.c_str());
        pCharacteristic->notify();

        delay(50);
    } else {
        delay(50);
    }
}
