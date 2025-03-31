#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <ArduinoJson.h>

// Thông tin WiFi
const char *ssid = "YOUR_WIFI_SSID";
const char *password = "YOUR_WIFI_PASSWORD";

// Thông tin server
const char *host = "abc.com";
const int httpsPort = 443;

// Pin định nghĩa cho ESP32-C3 Super Mini
#define MQ135_PIN 0 // GPIO4 (ADC1_CH4)
#define UV_PIN 1    // GPIO5 (ADC1_CH5)
// #define SDA_PIN 20  // GPIO20 cho I2C SDA
// #define SCL_PIN 21  // GPIO21 cho I2C SCL

// Khởi tạo đối tượng
Adafruit_AHTX0 aht10;
WiFiClientSecure client;

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
    vTaskDelay(2000 / portTICK_PERIOD_MS); // Đọc mỗi 2 giây
  }
}

// Task đọc cảm biến analog (MQ135 và UV)
void analogSensorTask(void *pvParameters)
{
  while (1)
  {
    mq135_value = analogRead(MQ135_PIN);
    uv_value = analogRead(UV_PIN);
    vTaskDelay(2000 / portTICK_PERIOD_MS); // Đọc mỗi 2 giây
  }
}

// Task gửi dữ liệu lên server
void sendDataTask(void *pvParameters)
{
  while (1)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      if (client.connect(host, httpsPort))
      {
        // Tạo JSON bằng ArduinoJson v7
        StaticJsonDocument<200> doc;
        doc["temperature"] = temperature;
        doc["humidity"] = humidity;
        doc["mq135"] = mq135_value;
        doc["uv"] = uv_value;

        String payload;
        serializeJson(doc, payload);

        // Gửi HTTP POST request
        client.println("POST /your_endpoint HTTP/1.1"); // Thay /your_endpoint bằng endpoint thực tế
        client.println("Host: " + String(host));
        client.println("Content-Type: application/json");
        client.println("Content-Length: " + String(payload.length()));
        client.println();
        client.print(payload);

        // Đọc response (tuỳ chọn)
        while (client.connected())
        {
          String line = client.readStringUntil('\n');
          if (line == "\r")
            break;
        }
        String response = client.readString();
        Serial.println("Server response: " + response);

        client.stop();
      }
      else
      {
        Serial.println("Connection to server failed");
      }
    }
    vTaskDelay(10000 / portTICK_PERIOD_MS); // Gửi mỗi 10 giây
  }
}

// Kết nối WiFi
void connectWiFi()
{
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Serial.println("IP: " + WiFi.localIP().toString());
}

void setup()
{
  Serial.begin(115200);

  // Khởi tạo I2C và AHT10
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!aht10.begin())
  {
    Serial.println("Could not find AHT10? Check wiring");
    while (1)
      delay(10);
  }
  Serial.println("AHT10 found");

  // Kết nối WiFi
  connectWiFi();

  // Không cần verify SSL certificate (cho test)
  client.setInsecure();

  // Tạo các task
  xTaskCreate(aht10Task, "AHT10 Task", 2048, NULL, 1, NULL);
  xTaskCreate(analogSensorTask, "Analog Sensor Task", 2048, NULL, 1, NULL);
  xTaskCreate(sendDataTask, "Send Data Task", 4096, NULL, 1, NULL);
}

void loop()
{
  // Không cần code trong loop khi dùng FreeRTOS
}