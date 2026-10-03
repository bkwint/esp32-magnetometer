#include <Wire.h>
#include <Adafruit_MLX90393.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>

// Generate your own UUIDs at uuidgenerator.net if you like
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"  // sensor data (read/notify)
#define NAME_CHAR_UUID      "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // device name (read/write)

#define SEND_INTERVAL_MS 200  // how often to push a value
#define DEFAULT_NAME     "ESP32 - MagnetoSensor"
#define MAX_NAME_LEN     29   // keeps the name within the 31-byte BLE packet limit

BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;
BLECharacteristic *pNameCharacteristic = nullptr;
Preferences prefs;

bool deviceConnected = false;
bool wasConnected = false;
bool restartPending = false;
unsigned long restartAt = 0;
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

class NameCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    String newName = String(pChar->getValue().c_str());
    newName.trim();

    if (newName.length() == 0) {
      Serial.println("Ignored empty name");
      return;
    }
    if (newName.length() > MAX_NAME_LEN) {
      newName = newName.substring(0, MAX_NAME_LEN);
    }

    prefs.begin("ble", false);
    prefs.putString("name", newName);
    prefs.end();

    pChar->setValue(newName.c_str());

    Serial.print("New device name saved: ");
    Serial.println(newName);

    // Restart shortly so the write response goes out first,
    // then the device comes back advertising under the new name
    restartPending = true;
    restartAt = millis() + 1000;
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

  initSensor();

  // Load saved device name (falls back to default)
  prefs.begin("ble", true);
  String deviceName = prefs.getString("name", DEFAULT_NAME);
  prefs.end();

  BLEDevice::init(deviceName.c_str());

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Sensor data characteristic
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());  // lets the phone enable notifications
  pCharacteristic->setValue("0.00");

  // Device name characteristic (phone can read the current name and write a new one)
  pNameCharacteristic = pService->createCharacteristic(
    NAME_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );
  pNameCharacteristic->setCallbacks(new NameCallbacks());
  pNameCharacteristic->setValue(deviceName.c_str());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.print("Advertising as ");
  Serial.print(deviceName);
  Serial.println(". Connect from your phone.");
}

void loop() {
  if (restartPending && millis() >= restartAt) {
    Serial.println("Restarting to apply new name...");
    ESP.restart();
  }

  if (deviceConnected && millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();

    float value = readSensor(); // 0.699151576
    char buf[24];
    // %.9g = 9 significant digits: the full precision of a 32-bit float,
    // with no padding zeros and no wasted characters. The receiver decides
    // how many digits to display.
    snprintf(buf, sizeof(buf), "%.9g", value);

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