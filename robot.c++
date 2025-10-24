// ======= ROBOT ESP32 LoRa + GPS (bidireccional) =======
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// --- LoRa ---
#define LORA_SS    5
#define LORA_RST   14
#define LORA_DIO0  26
#define LORA_BAND  433E6
#define LORA_SYNC  0x12   // <--- igual en gateway

// --- GPS (NEO-6M) ---
#define GPS_RX 16   // RX del ESP32 (a TX del GPS)
#define GPS_TX 17   // TX del ESP32 (a RX del GPS)

const char* ROBOT_ID = "R1";

TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

// Estado
String MODE = "AUTO";
unsigned long PERIOD_MS = 2000;  // se puede cambiar por comando
unsigned long lastSend = 0;

// ---- UTIL ----
void sendTelemetry(float lat, float lon, float alt, float spd, int sats) {
  char buf[300];
  snprintf(buf, sizeof(buf),
    "{\"robotId\":\"%s\",\"ts\":%lu,"
    "\"pos\":{\"type\":\"Point\",\"coordinates\":[%.6f,%.6f]},"
    "\"alt\":%.2f,\"temp\":0.00,\"pres\":0,\"spd\":%.2f,"
    "\"mode\":\"%s\",\"sats\":%d,\"bat\":100}",
    ROBOT_ID, millis()/1000, lon, lat, alt, spd, MODE.c_str(), sats
  );
  LoRa.beginPacket();
  LoRa.print(buf);
  LoRa.endPacket();
  Serial.print("[TX LoRa] "); Serial.println(buf);
}

void processCommand(const String& cmd) {
  Serial.print("[CMD RX] "); Serial.println(cmd);

  if (cmd.indexOf("\"cmd\":\"SET_MODE\"") >= 0) {
    if (cmd.indexOf("\"mode\":\"AUTO\"") >= 0) {
      MODE = "AUTO";
      Serial.println("[ACT] MODO -> AUTO");
    } else if (cmd.indexOf("\"mode\":\"MANUAL\"") >= 0) {
      MODE = "MANUAL";
      Serial.println("[ACT] MODO -> MANUAL");
    }
  } else if (cmd.indexOf("\"cmd\":\"SET_RATE\"") >= 0) {
    int i = cmd.indexOf("\"period_ms\":");
    if (i >= 0) {
      unsigned long ms = cmd.substring(i + 12).toInt();
      if (ms >= 500 && ms <= 10000) {
        PERIOD_MS = ms;
        Serial.printf("[ACT] NUEVO PERIODO = %lu ms\n", ms);
      }
    }
  } else if (cmd.indexOf("\"cmd\":\"PING\"") >= 0) {
    delay(100);
    LoRa.beginPacket();
    LoRa.print("{\"ack\":\"PONG\"}");
    LoRa.endPacket();
    Serial.println("[ACT] PONG enviado");
  }
}

void setup() {
  Serial.begin(115200);
  delay(400);

  // GPS
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("[GPS] OK");

  // LoRa
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

void loop() {
  // Leer GPS continuamente para no perder NMEA
  while (SerialGPS.available()) gps.encode(SerialGPS.read());

  // Envío periódico
  if (millis() - lastSend > PERIOD_MS) {
    lastSend = millis();

    float lat = gps.location.isValid() ? gps.location.lat() : 0;
    float lon = gps.location.isValid() ? gps.location.lng() : 0;
    float alt = gps.altitude.meters();
    float spd = gps.speed.mps();
    int sats  = gps.satellites.value();

    sendTelemetry(lat, lon, alt, spd, sats);
    Serial.printf("[GPS] sats=%d hdop=%.2f\n", sats, gps.hdop.hdop());

    // --- Ventana de bajada: escucha comandos 700 ms ---
    unsigned long t0 = millis();
    while (millis() - t0 < 700) {
      int p = LoRa.parsePacket();
      if (p) {
        String cmd;
        while (LoRa.available()) cmd += (char)LoRa.read();
        processCommand(cmd);
      }
      while (SerialGPS.available()) gps.encode(SerialGPS.read());
    }
  }

  // También escucha fuera de la ventana (por si acaso)
  int p = LoRa.parsePacket();
  if (p) {
    String cmd;
    while (LoRa.available()) cmd += (char)LoRa.read();
    processCommand(cmd);
  }
}
