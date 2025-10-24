// ======= GATEWAY ESP32 LoRa <-> MQTT (con repetición de comandos) =======
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>

// --- WiFi ---
const char* WIFI_SSID = "iPhone_de_Fernando2.4";
const char* WIFI_PASS = "tobias6510";

// --- MQTT ---
const char* MQTT_SERVER = "test.mosquitto.org";   // o IP local de tu broker
const uint16_t MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "ESP32_Gateway_LoRa";
const char* TOPIC_UP   = "iot/telemetry/up";      // publica telemetría
const char* TOPIC_DOWN = "iot/commands/down";     // recibe comandos
const char* TOPIC_HB   = "iot/gateway/heartbeat"; // opcional: latido

WiFiClient espClient;
PubSubClient mqtt(espClient);

// --- LoRa (SX1278) ---
#define LORA_SS    5
#define LORA_RST   14
#define LORA_DIO0  26
#define LORA_BAND  433E6
#define LORA_SYNC  0x12   // <--- igual que en el robot

// --- UTIL ---
void sendLoRaCommandRepeat(const String& jsonCmd, int n=6, int gap_ms=150) {
  for (int i=0; i<n; i++) {
    LoRa.beginPacket();
    LoRa.print(jsonCmd);
    LoRa.endPacket();
    Serial.print("[CMD TX] "); Serial.println(jsonCmd);
    delay(gap_ms);
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.print("[MQTT RX] "); Serial.println(msg);
  // Retransmite a LoRa con repetición para evitar pérdidas
  sendLoRaCommandRepeat(msg, 6, 150);
}

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[WiFi] Conectando");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\n[WiFi] Conectado ✅");
  Serial.print("[IP] "); Serial.println(WiFi.localIP());
}

void reconnectMQTT() {
  while (!mqtt.connected()) {
    Serial.print("[MQTT] Conectando...");
    if (mqtt.connect(MQTT_CLIENT_ID)) {
      Serial.println(" conectado ✅");
      mqtt.subscribe(TOPIC_DOWN);
      Serial.print("[MQTT] Suscrito a "); Serial.println(TOPIC_DOWN);
    } else {
      Serial.print(" fallo rc="); Serial.print(mqtt.state());
      Serial.println(" reintento en 3s");
      delay(3000);
    }
  }
}

void setupLoRa() {
  SPI.begin(18, 19, 23, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("[LoRa] FALLO begin()");
    while (true) delay(1000);
  }
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(LORA_SYNC);   // <--- importante
  Serial.println("[LoRa] OK @ 433 MHz");
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("===== GATEWAY ESP32 LoRa <-> MQTT =====");
  setupWiFi();
  setupLoRa();

  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
}

void loop() {
  if (!mqtt.connected()) reconnectMQTT();
  mqtt.loop();

  // RX LoRa -> publica a MQTT
  int psize = LoRa.parsePacket();
  if (psize) {
    String payload;
    while (LoRa.available()) payload += (char)LoRa.read();
    Serial.print("[RX LoRa] "); Serial.println(payload);

    bool ok = mqtt.publish(TOPIC_UP, payload.c_str());
    Serial.print("[MQTT TX] ");
    Serial.println(ok ? "Publicado ✔️" : "FALLO ❌");
  }

  // Heartbeat cada 10 s (diagnóstico)
  static unsigned long t0 = 0;
  if (millis() - t0 > 10000) {
    t0 = millis();
    mqtt.publish(TOPIC_HB, "alive");
  }

  // Opcional: teclas por Serial para enviar comandos sin MQTT
  if (Serial.available()) {
    char k = Serial.read();
    if (k == 'a') sendLoRaCommandRepeat("{\"cmd\":\"SET_MODE\",\"mode\":\"AUTO\"}");
    if (k == 'm') sendLoRaCommandRepeat("{\"cmd\":\"SET_MODE\",\"mode\":\"MANUAL\"}");
    if (k == 'p') sendLoRaCommandRepeat("{\"cmd\":\"PING\"}");
    if (k == '1') sendLoRaCommandRepeat("{\"cmd\":\"SET_RATE\",\"period_ms\":1000}");
    if (k == '3') sendLoRaCommandRepeat("{\"cmd\":\"SET_RATE\",\"period_ms\":3000}");
  }
}
