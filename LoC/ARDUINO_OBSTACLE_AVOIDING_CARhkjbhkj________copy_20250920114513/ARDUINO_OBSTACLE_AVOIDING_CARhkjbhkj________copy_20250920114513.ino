#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "Ya";
const char* password = "12345678";
const char* mqtt_server = "test.mosquitto.org";

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsg = 0;
bool motorState = false; // стан мотора

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  
  String message = "";
  for (int i = 0; i < length; i++) message += (char)payload[i];
  Serial.println(message);

  // Якщо прийшла команда для мотора
  if (String(topic) == "esp32/motor") {
    if (message == "1") motorState = true;
    else if (message == "0") motorState = false;

    Serial.print("Motor state changed: ");
    Serial.println(motorState ? "ON" : "OFF");
  }
}

void reconnect() {
  // Підключення до брокера MQTT
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe("esp32/motor");  // 🔹 підписка на керування мотором
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > 2000) { // кожні 2 секунди
    lastMsg = now;
    float pressure = random(100, 200);   // тестові дані
    float flow = random(10, 30) / 10.0;  // тестові дані
    float temp = random(25, 35);         // тестові дані

    // Формат повідомлення: t,pressure,flow,temp,motor
    String payload = String(now / 1000.0, 2) + "," +
                     String(pressure) + "," +
                     String(flow, 2) + "," +
                     String(temp, 2) + "," +
                     String(motorState ? 1 : 0);

    Serial.print("Publishing: ");
    Serial.println(payload);

    client.publish("esp32/demo", payload.c_str()); // 🔹 надсилаємо в LoC.py
  }
}