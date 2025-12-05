#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* ssid = " Your_SSID ";
const char* password = " Your_Password ";
const char* mqtt_server = "broker.netpie.io";
const int mqtt_port = 1883;
const char* mqtt_Client = " Client_ID ";
const char* mqtt_username = " Token ";
const char* mqtt_password = " Secret ";

WiFiClient espClient;
PubSubClient client(espClient);

int temp = 0, humi = 0, light = 0;

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 3000;

void setup_wifi() {
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("");
    Serial.println("WiFi connection failed!");
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect(mqtt_Client, mqtt_username, mqtt_password)) {
      Serial.println("MQTT connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      delay(2000);
    }
  }
}

void sendDataToNetpie() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  StaticJsonDocument<200> doc;
  doc["data"]["temperature"] = temp;
  doc["data"]["humidity"] = humi;
  doc["data"]["light"] = light;

  char buffer[200];
  serializeJson(doc, buffer);

  Serial.print("Publishing JSON: ");
  Serial.println(buffer);

  if (client.publish("@shadow/data/update", buffer)) {
    Serial.println("Publish OK");
  } else {
    Serial.println("Publish FAILED");
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 18, 17);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (Serial1.available()) {
    String line = Serial1.readStringUntil('\n');
    Serial.println(line);

    int t, h, l;
    if (sscanf(line.c_str(), "T:%d,H:%d,L:%d", &t, &h, &l) == 3) {
      temp = t;
      humi = h;
      light = l;
    }
  }

  unsigned long currentMillis = millis();
  if (currentMillis - lastSendTime >= sendInterval) {
    lastSendTime = currentMillis;
    sendDataToNetpie();
  }
}
