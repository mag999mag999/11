#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>
#include <Adafruit_AHTX0.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <Updater.h>
#include <espnow.h> // Додано бібліотеку ESP-NOW

const char* ssid = "?";
const char* password = "?";

ESP8266WebServer server(80);
Adafruit_AHTX0 aht;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7200, 60000);

const int LED_PIN = D6;
const int BUTTON_PIN = D7;

struct SensorData {
  float temperature = 0.0;
  float humidity = 0.0;
  bool ledState = false;
  bool buttonState = false;
} sensorData;

// Структура для даних, отриманих через ESP-NOW
struct RemoteSensorData {
  float remoteTemperature = 0.0;
  float remoteHumidity = 0.0;
  float remotePressure = 0.0;
} remoteData;

const int DATA_SIZE = 60;
float tempData[DATA_SIZE];
float humidData[DATA_SIZE];
String timeStamps[DATA_SIZE];
int dataIndex = 0;
bool dataFull = false;

struct Extremes {
  float maxTemp = -1000.0;
  String maxTempTime = "00:00:00";
  float minTemp = 1000.0;
  String minTempTime = "00:00:00";
  float maxHumid = -1000.0;
  String maxHumidTime = "00:00:00";
  float minHumid = 1000.0;
  String minHumidTime = "00:00:00";
} extremes;

unsigned int bootCount = 0;
const char* bootCountFile = "/bootcount.txt";

unsigned long lastDataSave = 0;
const unsigned long INTERVAL = 600000;

const char* firmwareUsername = "admin";
const char* firmwarePassword = "admin";

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  if (!LittleFS.begin()) {
    Serial.println("Помилка LittleFS");
    while (true) yield();
  }
  Serial.println("LittleFS ініціалізовано");
  Dir dir = LittleFS.openDir("/");
  Serial.println("Перевірка вмісту LittleFS:");
  while (dir.next()) {
    Serial.println("Файл: " + dir.fileName());
  }

  loadBootCount();
  bootCount++;
  saveBootCount();

  // Ініціалізація ESP-NOW
  WiFi.mode(WIFI_STA); // Режим станції для ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Помилка ініціалізації ESP-NOW");
    while (true) yield();
  }
  esp_now_register_recv_cb(OnDataRecv); // Реєстрація обробника отриманих даних

  WiFi.begin(ssid, password);
  Serial.print("Підключення до Wi-Fi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nНе вдалося підключитися до Wi-Fi");
    while (true) yield();
  }
  Serial.println("\nWiFi підключено: " + WiFi.localIP().toString());

  if (!aht.begin()) {
    Serial.println("Не вдалося знайти AHT10");
    while (true) yield();
  }
  delay(2000);

  timeClient.begin();
  timeClient.update();

  for (int i = 0; i < DATA_SIZE; i++) {
    tempData[i] = 0.0;
    humidData[i] = 0.0;
    timeStamps[i] = "00:00:00";
  }

  server.on("/firmware", HTTP_GET, []() { serveFile("/firmware.html", "text/html"); });
  server.on("/firmware", HTTP_POST, handleFirmwareUpdate, handleFirmwareUpload);
  server.on("/", HTTP_GET, handleRoot);
  server.on("/index.html", HTTP_GET, handleRoot);
  server.on("/info.html", HTTP_GET, handleInfo);
  server.on("/data", HTTP_GET, handleData);
  server.on("/led", HTTP_GET, handleLed);
  server.on("/fs", HTTP_GET, []() { serveFile("/filesystem.html", "text/html"); });
  server.on("/fs/list", HTTP_GET, handleFileList);
  server.on("/fs/upload", HTTP_POST, []() { server.send(200); }, handleFileUploadToFS);
  server.on("/fs/download", HTTP_GET, handleFileDownload);
  server.on("/fs/delete", HTTP_DELETE, handleFileDelete);
  server.on("/graphData", HTTP_GET, handleGraphData);
  server.on("/humidityTrend", HTTP_GET, handleHumidityTrend);
  server.on("/temperatureTrend", HTTP_GET, handleTemperatureTrend);
  server.on("/reset", HTTP_GET, handleReset);
  server.on("/systemInfo", HTTP_GET, handleSystemInfo);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("Сервер запущено");
}

void loop() {
  server.handleClient();
  timeClient.update();

  unsigned long currentMillis = millis();
  if (currentMillis - lastDataSave >= INTERVAL) {
    saveSensorData();
    lastDataSave = currentMillis;
  }
}

// Обробник отриманих даних через ESP-NOW
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&remoteData, incomingData, sizeof(remoteData));
  Serial.printf("Отримано через ESP-NOW: Темп: %.1f°C, Вологість: %.1f%%, Тиск: %.1f mm", 
                remoteData.remoteTemperature, remoteData.remoteHumidity, remoteData.remotePressure);
}

void handleFileList() {
  DynamicJsonDocument doc(2048);
  JsonArray files = doc.createNestedArray("files");

  Dir dir = LittleFS.openDir("/");
  Serial.println("Список файлів у LittleFS (маршрут /fs/list):");
  while (dir.next()) {
    JsonObject file = files.createNestedObject();
    file["name"] = dir.fileName();
    File f = dir.openFile("r");
    file["size"] = f.size();
    Serial.println("Файл: " + dir.fileName() + ", розмір: " + String(f.size()) + " байт");
    f.close();
  }
  if (files.size() == 0) {
    Serial.println("Файли не знайдено");
  }

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleFirmwareUpdate() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_END) {
    server.send(200, "text/plain", "Прошивка успішно оновлена. Перезавантаження...");
    delay(1000);
    ESP.restart();
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    server.send(500, "text/plain", "Помилка оновлення: завантаження перервано");
  }
}

void handleFirmwareUpload() {
  HTTPUpload& upload = server.upload();

  if (!server.hasArg("username") || !server.hasArg("password") ||
      server.arg("username") != firmwareUsername || server.arg("password") != firmwarePassword) {
    server.send(401, "text/plain", "Неправильний логін або пароль");
    return;
  }

  if (upload.status == UPLOAD_FILE_START) {
    Serial.println("Початок оновлення прошивки: " + upload.filename);
    if (!Update.begin(upload.totalSize)) {
      Serial.println("Помилка початку оновлення: " + String(Update.getError()));
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Serial.println("Помилка запису: " + String(Update.getError()));
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      Serial.println("Оновлення завершено: " + String(upload.totalSize) + " байт");
    } else {
      Serial.println("Помилка завершення оновлення: " + String(Update.getError()));
    }
  }
}

void handleFileUploadToFS() {
  HTTPUpload& upload = server.upload();
  static File fsUploadFile;

  if (upload.status == UPLOAD_FILE_START) {
    String filename = "/" + upload.filename;
    if (!filename.startsWith("/")) filename = "/" + filename;
    Serial.println("Завантаження файлу: " + filename);
    
    if (LittleFS.exists(filename)) {
      LittleFS.remove(filename);
    }
    
    fsUploadFile = LittleFS.open(filename, "w");
    if (!fsUploadFile) {
      Serial.println("Не вдалося відкрити файл для запису");
      server.send(500, "text/plain", "Не вдалося відкрити файл");
      return;
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (fsUploadFile) {
      fsUploadFile.write(upload.buf, upload.currentSize);
      Serial.printf("Записано %d байт\n", upload.currentSize);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (fsUploadFile) {
      fsUploadFile.close();
      Serial.println("Завантаження завершено: " + String(upload.totalSize) + " байт");
      server.send(200, "text/plain", "Файл успішно завантажено: " + upload.filename);
    } else {
      server.send(500, "text/plain", "Помилка при завантаженні файлу");
    }
  }
}

void loadBootCount() {
  File file = LittleFS.open(bootCountFile, "r");
  if (file) {
    bootCount = file.readString().toInt();
    file.close();
  } else {
    bootCount = 0;
  }
}

void saveBootCount() {
  File file = LittleFS.open(bootCountFile, "w");
  if (file) {
    file.print(bootCount);
    file.close();
  } else {
    Serial.println("Помилка збереження bootCount");
  }
}

void saveSensorData() {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  sensorData.temperature = round(temp.temperature * 10) / 10.0;
  sensorData.humidity = round(humidity.relative_humidity * 10) / 10.0;

  tempData[dataIndex] = sensorData.temperature;
  humidData[dataIndex] = sensorData.humidity;
  timeStamps[dataIndex] = timeClient.getFormattedTime();

  if (sensorData.temperature > extremes.maxTemp) {
    extremes.maxTemp = sensorData.temperature;
    extremes.maxTempTime = timeStamps[dataIndex];
  }
  if (sensorData.temperature < extremes.minTemp) {
    extremes.minTemp = sensorData.temperature;
    extremes.minTempTime = timeStamps[dataIndex];
  }
  if (sensorData.humidity > extremes.maxHumid) {
    extremes.maxHumid = sensorData.humidity;
    extremes.maxHumidTime = timeStamps[dataIndex];
  }
  if (sensorData.humidity < extremes.minHumid) {
    extremes.minHumid = sensorData.humidity;
    extremes.minHumidTime = timeStamps[dataIndex];
  }

  dataIndex = (dataIndex + 1) % DATA_SIZE;
  if (dataIndex == 0) dataFull = true;

  Serial.printf("Температура: %.1f°C, Вологість: %.1f%%, Час: %s\n", 
                sensorData.temperature, sensorData.humidity, timeStamps[dataIndex].c_str());
}

void handleRoot() {
  serveFile("/index.html", "text/html");
}

void handleInfo() {
  serveFile("/info.html", "text/html");
}

void handleData() {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  sensorData.temperature = round(temp.temperature * 10) / 10.0;
  sensorData.humidity = round(humidity.relative_humidity * 10) / 10.0;
  sensorData.buttonState = !digitalRead(BUTTON_PIN);
  sensorData.ledState = digitalRead(LED_PIN);

  DynamicJsonDocument doc(512); // Збільшено розмір для додаткових даних
  doc["temperature"] = sensorData.temperature;
  doc["humidity"] = sensorData.humidity;
  doc["buttonState"] = sensorData.buttonState;
  doc["ledState"] = sensorData.ledState;
  doc["timestamp"] = timeClient.getFormattedTime();
  doc["remoteTemperature"] = remoteData.remoteTemperature;
  doc["remoteHumidity"] = remoteData.remoteHumidity;
  doc["remotePressure"] = round(remoteData.remotePressure * 10) / 10.0;

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleGraphData() {
  DynamicJsonDocument doc(2048);
  JsonArray tempArray = doc.createNestedArray("temperature");
  JsonArray humidArray = doc.createNestedArray("humidity");
  JsonArray timeArray = doc.createNestedArray("timestamps");

  int count = dataFull ? DATA_SIZE : dataIndex;
  int startIdx = dataFull ? dataIndex : 0;
  for (int i = 0; i < count; i++) {
    int idx = (startIdx + i) % DATA_SIZE;
    tempArray.add(round(tempData[idx] * 10) / 10.0);
    humidArray.add(round(humidData[idx] * 10) / 10.0);
    timeArray.add(timeStamps[idx]);
  }

  doc["maxTemp"] = extremes.maxTemp;
  doc["minTemp"] = extremes.minTemp;
  doc["maxHumid"] = extremes.maxHumid;
  doc["minHumid"] = extremes.minHumid;
  doc["maxTempTime"] = extremes.maxTempTime;
  doc["minTempTime"] = extremes.minTempTime;
  doc["maxHumidTime"] = extremes.maxHumidTime;
  doc["minHumidTime"] = extremes.minHumidTime;

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleLed() {
  String state = server.arg("state");
  if (state == "on") {
    sensorData.ledState = true;
    digitalWrite(LED_PIN, HIGH);
  } else if (state == "off") {
    sensorData.ledState = false;
    digitalWrite(LED_PIN, LOW);
  }
  DynamicJsonDocument doc(64);
  doc["ledState"] = sensorData.ledState;
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleFileDownload() {
  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "Не вказано файл для завантаження");
    return;
  }

  String fileName = server.arg("file");
  if (!fileName.startsWith("/")) fileName = "/" + fileName;

  File file = LittleFS.open(fileName, "r");
  if (!file) {
    server.send(404, "text/plain", "Файл не знайдено: " + fileName);
    return;
  }

  String contentType = getContentType(fileName);
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + fileName.substring(1) + "\"");
  server.streamFile(file, contentType);
  file.close();
}

void handleFileDelete() {
  if (server.hasArg("file")) {
    String fileName = server.arg("file");
    if (LittleFS.remove(fileName)) {
      server.send(200, "text/plain", "Файл видалено: " + fileName);
    } else {
      server.send(400, "text/plain", "Помилка видалення файлу: " + fileName);
    }
  } else {
    server.send(400, "text/plain", "Не вказано файл для видалення");
  }
}

void handleHumidityTrend() {
  DynamicJsonDocument doc(128);
  JsonArray trend = doc.createNestedArray("trend");
  int count = (dataFull ? DATA_SIZE : dataIndex) >= 3 ? 3 : (dataFull ? DATA_SIZE : dataIndex);
  for (int i = 0; i < count; i++) {
    int idx = (dataIndex - 1 - i + DATA_SIZE) % DATA_SIZE;
    trend.add(round(humidData[idx] * 10) / 10.0);
  }
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleTemperatureTrend() {
  DynamicJsonDocument doc(128);
  JsonArray trend = doc.createNestedArray("trend");
  int count = (dataFull ? DATA_SIZE : dataIndex) >= 3 ? 3 : (dataFull ? DATA_SIZE : dataIndex);
  for (int i = 0; i < count; i++) {
    int idx = (dataIndex - 1 - i + DATA_SIZE) % DATA_SIZE;
    trend.add(round(tempData[idx] * 10) / 10.0);
  }
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleNotFound() {
  String path = server.uri();
  Serial.println("Запит до: " + path);
  serveFile(path, getContentType(path));
}

void handleReset() {
  server.send(200, "text/plain", "Скидання ESP8266...");
  delay(1000);
  ESP.restart();
}

void handleSystemInfo() {
  FSInfo fs_info;
  LittleFS.info(fs_info);

  float freeHeapPercent = (float)ESP.getFreeHeap() / 81920 * 100.0;
  float freeStackPercent = (float)ESP.getFreeContStack() / 4096 * 100.0;
  float sketchSizePercent = (float)ESP.getSketchSize() / (ESP.getFreeSketchSpace() + ESP.getSketchSize()) * 100.0;
  float fsUsagePercent = (float)fs_info.usedBytes / fs_info.totalBytes * 100.0;

  DynamicJsonDocument doc(1024);
  doc["rssi"] = WiFi.RSSI();
  doc["localIp"] = WiFi.localIP().toString();
  doc["macAddress"] = WiFi.macAddress();
  doc["freeHeapPercent"] = round(freeHeapPercent * 10) / 10.0;
  doc["totalHeap"] = 81920;
  doc["freeStackPercent"] = round(freeStackPercent * 10) / 10.0;
  doc["totalStack"] = 4096;
  doc["sketchSizePercent"] = round(sketchSizePercent * 10) / 10.0;
  doc["totalSketch"] = ESP.getFreeSketchSpace() + ESP.getSketchSize();
  doc["uptime"] = millis() / 1000;
  doc["bootCount"] = bootCount;
  doc["resetReason"] = ESP.getResetReason();
  doc["cpuFreq"] = ESP.getCpuFreqMHz();
  doc["fsUsagePercent"] = round(fsUsagePercent * 10) / 10.0;
  doc["firmwareVersion"] = "1.0.1";
  doc["coreVersion"] = ESP.getCoreVersion();
  doc["clientCount"] = WiFi.softAPgetStationNum();
  doc["vcc"] = ESP.getVcc() / 1000.0;
  doc["runtime"] = timeClient.getFormattedTime();

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void serveFile(String path, String contentType) {
  File file = LittleFS.open(path, "r");
  if (!file) {
    server.send(404, "text/plain", "Файл не знайдено: " + path);
    return;
  }
  server.streamFile(file, contentType);
  file.close();
}

String getContentType(String filename) {
  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".txt")) return "text/plain";
  else if (filename.endsWith(".bin")) return "application/octet-stream";
  else if (filename.endsWith(".pdf")) return "application/pdf";
  else if (filename.endsWith(".json")) return "application/json";
  return "application/octet-stream";
}
