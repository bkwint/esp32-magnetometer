#include <Wire.h>
#include <Adafruit_MLX90393.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Generate your own UUIDs at uuidgenerator.net if you like
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define SEND_INTERVAL_MS 200  // how often to push a value

BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;
bool deviceConnected = false;
bool wasConnected = false;
unsigned long lastSend = 0;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    deviceConnected = true;
    Serial.println("Phone connected");
  }
  void onDisconnect(BLEServer *server) override {
    deviceConnected = false;
    Serial.println("Phone disconnected");
  }
};

Adafruit_MLX90393 sensor = Adafruit_MLX90393();

float readSensor() {
  float x, y, z;

  if (sensor.readData(&x, &y, &z)) {
    // The library outputs data in microTeslas (uT)
    // 1 Gauss = 100 microTeslas (uT)
    float gaussX = x / 100.0;
    float gaussY = y / 100.0;
    float gaussZ = z / 100.0;

    // Calculate total net magnetic field magnitude
    float totalGauss = sqrt(gaussX * gaussX + gaussY * gaussY + gaussZ * gaussZ);

    // Determine polarity based on dominant Z-axis vector direction
    if (gaussZ < 0) {
      totalGauss = -totalGauss;
    }

    return totalGauss;
  }

  return 0;
}

void initSensor() {
  Serial.println("Initializing MLX90393 Gauss Meter...");

  Wire.begin(6, 7);

  if (!sensor.begin_I2C(0x18)) {
    Serial.println("Sensor not found. Check wiring!");
    while (1);
  }

  // Optimize gain and resolution for fine measurements
  sensor.setGain(MLX90393_GAIN_1X);
  sensor.setResolution(MLX90393_X, MLX90393_RES_16);
  sensor.setResolution(MLX90393_Y, MLX90393_RES_16);
  sensor.setResolution(MLX90393_Z, MLX90393_RES_16);
}

void setup() {
  Serial.begin(9600);
  while (!Serial) { delay(10); }

  initSensor();

  BLEDevice::init("ESP32 - MagnetoSensor");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());  // lets the phone enable notifications
  pCharacteristic->setValue("0.00");

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("Advertising as ESP32 - MagnetoSensor. Connect from your phone.");
}

void loop() {
  if (deviceConnected && millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();

    float value = readSensor();
    char buf[16];
    snprintf(buf, sizeof(buf), "%.2f", value);   // send as readable text

    pCharacteristic->setValue((uint8_t *)buf, strlen(buf));
    pCharacteristic->notify();

    Serial.print("Sent: ");
    Serial.println(buf);
  }

  // Restart advertising after a disconnect so the phone can reconnect
  if (!deviceConnected && wasConnected) {
    delay(300);
    BLEDevice::startAdvertising();
    Serial.println("Advertising restarted");
  }
  wasConnected = deviceConnected;
}