#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <WiFiManager.h>
#include <SocketIoClient.h>

// Thông tin server
#define SERVER "maydokhongkhi.com"
#define PORT 80
String serverUrl2 = "https://mmsso.com/update";
#define TOPIC_UPDATE "message"
#define TOPIC_CONTROL "control"

// Pin định nghĩa cho ESP32-C3 Super Mini
#define MQ135_PIN 0 // GPIO4 (ADC1_CH4)
#define UV_PIN 1    // GPIO5 (ADC1_CH5)
// #define SDA_PIN 20  // GPIO20 cho I2C SDA
// #define SCL_PIN 21  // GPIO21 cho I2C SCL

// Khởi tạo đối tượng
Adafruit_AHTX0 aht10;
SocketIOclient socketIO;

// Biến toàn cục lưu dữ liệu cảm biến
float temperature = 0.0;
float humidity = 0.0;
int mq135_value = 0;
int uv_value = 0;

// Task đọc cảm biến AHT10
void aht10Task(void *pvParameters)
{
  while (1)
  {
    sensors_event_t hum, temp;
    if (aht10.getEvent(&hum, &temp))
    {
      temperature = temp.temperature;
      humidity = hum.relative_humidity;
    }
    else
    {
      Serial.println("Failed to read AHT10");
    }
    vTaskDelay(pdMS_TO_TICKS(2000)); // Đọc mỗi 2 giây
  }
}

// Task đọc cảm biến analog (MQ135 và UV)
void analogSensorTask(void *pvParameters)
{
  while (1)
  {
    mq135_value = analogRead(MQ135_PIN);
    uv_value = analogRead(UV_PIN);
    vTaskDelay(pdMS_TO_TICKS(2000)); // Đọc mỗi 2 giây
  }
}

// Task gửi dữ liệu lên server qua GET
void sendDataTask(void *pvParameters)
{
  while (1)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      JsonDocument doc;
      doc.add(TOPIC_UPDATE);
      JsonObject data = doc.add<JsonObject>();
      data["TEMP"] = temperature;
      data["HUM"] = humidity;
      data["CO2"] = mq135_value;
      data["UV"] = uv_value;
      String output;
      doc.shrinkToFit(); // optional
      serializeJson(doc, output);

      // Gửi dữ liệu dưới dạng sự kiện Socket.IO
      socketIO.sendEVENT(output);
      Serial.println("Sent data: " + output);
    }
    else
    {
      Serial.println("WiFi disconnected");
    }
    vTaskDelay(pdMS_TO_TICKS(10000)); // Gửi mỗi 10 giây
  }
}

// Kết nối WiFi
void connectWiFi()
{
  WiFiManager wm;
  bool res = wm.autoConnect("ESP32-C3-Super-Mini", "66668888"); // Tên và mật khẩu WiFi
  if (!res)
  {
    Serial.println("Failed to connect to WiFi");
    // Reset ESP32
    ESP.restart();
  }
  Serial.println("Connected to WiFi");
}

#define USE_SERIAL Serial
void socketIOEvent(socketIOmessageType_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case sIOtype_DISCONNECT:
  {
    USE_SERIAL.printf("[IOc] Disconnected!\n");
    break;
  }
  case sIOtype_CONNECT:
  {
    USE_SERIAL.printf("[IOc] Connected to url: %s\n", payload);
    // join default namespace (no auto join in Socket.IO V3)
    socketIO.send(sIOtype_CONNECT, "/");
    break;
  }
  case sIOtype_EVENT:
  {
    String temp = String((char *)payload);
    if (temp.indexOf(TOPIC_CONTROL) != -1)
    {
      JsonDocument doc;
      deserializeJson(doc, temp);
      JsonObject data = doc["data"];
      int button = data["button"];
    }
  }
  break;
  case sIOtype_ACK:
    USE_SERIAL.printf("[IOc] get ack: %u\n", length);
    break;
  case sIOtype_ERROR:
    USE_SERIAL.printf("[IOc] get error: %u\n", length);
    break;
  case sIOtype_BINARY_EVENT:
    USE_SERIAL.printf("[IOc] get binary: %u\n", length);
    break;
  case sIOtype_BINARY_ACK:
    USE_SERIAL.printf("[IOc] get binary ack: %u\n", length);
    break;
  }
}

void setup()
{
  Serial.begin(115200);

  // Khởi tạo AHT10
  if (!aht10.begin())
  {
    Serial.println("Could not find AHT10? Check wiring");
  }
  Serial.println("AHT10 found");

  // Kết nối WiFi
  connectWiFi();

  // server address, port and URL
  socketIO.begin(SERVER, PORT, "/socket.io/?EIO=4");

  // event handler
  socketIO.onEvent(socketIOEvent);

  // Tạo các task
  xTaskCreate(aht10Task, "AHT10 Task", 2048, NULL, 1, NULL);
  xTaskCreate(analogSensorTask, "Analog Sensor Task", 2048, NULL, 1, NULL);
  xTaskCreate(sendDataTask, "Send Data Task", 20096, NULL, 1, NULL);
}

void loop()
{
  // Không cần code trong loop khi dùng FreeRTOS
  socketIO.loop();
}