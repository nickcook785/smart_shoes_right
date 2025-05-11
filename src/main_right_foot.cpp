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

float gaussianNormalize(int x, float mu, float sigma) {
    return exp(-pow(x - mu, 2) / (2 * pow(sigma, 2)));
}

void setupFSRPins() {
    pinMode(FSR1_PIN, INPUT);
    pinMode(FSR2_PIN, INPUT);
    pinMode(FSR3_PIN, INPUT);
    pinMode(FSR4_PIN, INPUT);
    pinMode(FSR5_PIN, INPUT);
    Serial.println("✅ FSR 센서 핀 설정 완료!");
}

void calculateAngles(float &pitch, float &roll) {
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
        float ax = a.acceleration.x;
        float ay = a.acceleration.y;
        float az = a.acceleration.z;

        pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
        roll = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
    } else {
        Serial.println("⚠️ MPU6050 데이터 읽기 실패 (센서 미연결일 수 있음)");
        pitch = 0.0;
        roll = 0.0;
    }

}
String evaluateSquat(float* norm) {
    float front = (norm[0] + norm[1] + norm[2]) / 3.0;
    float center = norm[3];
    float rear = norm[4];

    if (rear > 0.4 && front < 0.3 && center >= 0.2 && center <= 0.4) {
        return "GOOD";
    } else if (front >= 0.3) {
        return "LEANING_FORWARD";
    } else if (rear < 0.2) {
        return "NO_HEEL_PRESSURE";
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
    }
    BLEDevice::init("ESP32-S3 BLE right shoe");
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
    pService->start();

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);

    BLEDevice::startAdvertising();
    Serial.println("📡 BLE 서버 광고 시작");
}

void loop() {
    if (deviceConnected) {
        int fsrValues[NUM_FSR];
        float normalizedValues[NUM_FSR];  // 가우시안 정규화 값
        float finalNormalized[NUM_FSR];   // 총합 1로 다시 정규화된 값
        const char* postureResults[NUM_FSR];
        String squatPosture = evaluateSquat(finalNormalized);

        fsrValues[0] = analogRead(FSR1_PIN);
        fsrValues[1] = analogRead(FSR2_PIN);
        fsrValues[2] = analogRead(FSR3_PIN);
        fsrValues[3] = analogRead(FSR4_PIN);
        fsrValues[4] = analogRead(FSR5_PIN);

        float mu = 2000.0;   // 중심점
        float sigma = 500.0; // 퍼짐 정도

        float sumNormalized = 0.0;
        for (int i = 0; i < NUM_FSR; i++) {
            normalizedValues[i] = gaussianNormalize(fsrValues[i], mu, sigma);
            sumNormalized += normalizedValues[i];

            if (fsrValues[i] > 200) postureResults[i] = "ACTIVE";
            else if (fsrValues[i] > 50) postureResults[i] = "WEAK";
            else postureResults[i] = "INACTIVE";
        }

        // 총합이 0일 경우 나눗셈 방지 → 전체를 0으로 설정
        if (sumNormalized > 0.0) {
            for (int i = 0; i < NUM_FSR; i++) {
                finalNormalized[i] = normalizedValues[i] / sumNormalized;
            }
        } else {
            for (int i = 0; i < NUM_FSR; i++) {
                finalNormalized[i] = 0.0;
            }
        }

        float pitch, roll;
        calculateAngles(pitch, roll);

        StaticJsonDocument<600> doc;
        JsonArray fsrArray = doc.createNestedArray("fsr_right");  // 원본 값
        JsonArray normArray = doc.createNestedArray("normalized_right");  // 가우시안 정규화 (0~1)
        JsonArray finalArray = doc.createNestedArray("final_normalized_right");  // 총합 1로 다시 정규화된 값
        JsonArray postureArray = doc.createNestedArray("posture_right");

        for (int i = 0; i < NUM_FSR; i++) {
            fsrArray.add(fsrValues[i]);
            normArray.add(normalizedValues[i]);         // 원래 가우시안 정규화
            finalArray.add(finalNormalized[i]);         // 총합 1 정규화
            postureArray.add(postureResults[i]);
        }

        doc["pitch"] = pitch;
        doc["roll"] = roll;
        doc["squat_posture"] = squatPosture; 

        String jsonString;
        serializeJson(doc, jsonString);

        Serial.println(jsonString);  // 확인용 출력
        pCharacteristic->setValue(jsonString.c_str());
        pCharacteristic->notify();

        delay(1000);  // 주기
    } else {
        delay(500);
    }
}
