// ======= ROBOT ESP32: LoRa + GPS + microSD + Motores (DRV8833) =======
//
// REQUIERE LIBRERÍAS:
//   - LoRa (Sandeep Mistry)
//   - TinyGPSPlus
//   - SD (ESP32)
//   - FS (viene con ESP32 core)
//
// HARDWARE:
//   LoRa SX1278: 433 MHz
//   GPS NEO-6M
//   microSD por SPI (CS en GPIO4)
//   Driver motores DRV8833
//
// COMANDOS QUE RECIBE POR LORA (desde MQTT vía gateway):
//   {"cmd":"DRIVE","dir":"FWD"}
//   {"cmd":"DRIVE","dir":"BACK"}
//   {"cmd":"DRIVE","dir":"LEFT"}
//   {"cmd":"DRIVE","dir":"RIGHT"}
//   {"cmd":"DRIVE","dir":"STOP"}
//   {"cmd":"SET_MODE","mode":"AUTO"}
//   {"cmd":"SET_MODE","mode":"MANUAL"}
//   {"cmd":"SET_RATE","period_ms":1000}
//   {"cmd":"PING"}
//
// TELEMETRÍA QUE ENVÍA POR LORA:
//   { robotId, ts, pos{lon,lat}, alt, spd, mode, sats, bat }

#include <SPI.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <FS.h>
#include <SD.h>

// -------- LoRa config --------
#define LORA_SS    5
#define LORA_RST   14
#define LORA_DIO0  26
#define LORA_BAND  433E6
#define LORA_SYNC  0x12

// -------- GPS config --------
#define GPS_RX 16   // ESP32 RX <- GPS TX
#define GPS_TX 17   // ESP32 TX -> GPS RX (a veces no se usa, pero lo dejamos)
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

// -------- microSD config --------
#define SD_CS   4   // comparte bus SPI VSPI (CLK=18, MISO=19, MOSI=23)
bool sdOK = false;

// -------- Motores DRV8833 --------
// Motor A (izquierdo)
#define AIN1 27
#define AIN2 32
// Motor B (derecho)
#define BIN1 21
#define BIN2 22

// Ajuste PWM
// Usaremos 2 canales PWM por motor (AIN1 y AIN2 / BIN1 y BIN2).
// En ESP32 usamos ledc (PWM por hardware).
const int PWM_CH_AIN1 = 0;
const int PWM_CH_AIN2 = 1;
const int PWM_CH_BIN1 = 2;
const int PWM_CH_BIN2 = 3;
const int PWM_FREQ   = 20000;  // 20 kHz para que el motor no silbe feo
const int PWM_RES    = 8;      // 8 bits -> 0..255

// -------- Estado general robot --------
const char* ROBOT_ID = "R1";
String MODE = "AUTO";          // "AUTO" o "MANUAL" (llega por comando)
unsigned long PERIOD_MS = 2000; // tasa telemetría (ajustable)
unsigned long lastSend   = 0;

String currentDateYMD = "";  // nombre actual de archivo CSV
File logFile;
File evtFile;

// ========= UTILIDADES DE TIEMPO/FECHA =========
String twoDigits(int v){ if(v<10)return "0"+String(v); return String(v); }
String fourDigits(int v){ char b[5]; snprintf(b,sizeof(b),"%04d",v); return String(b); }

// genera timestamp ISO8601 (UTC si GPS válido)
String timestampISO8601() {
  if (gps.date.isValid() && gps.time.isValid()) {
    int Y = gps.date.year();
    int M = gps.date.month();
    int D = gps.date.day();
    int h = gps.time.hour();
    int m = gps.time.minute();
    int s = gps.time.second();
    char buf[25];
    snprintf(buf,sizeof(buf),"%04d-%02d-%02dT%02d:%02d:%02dZ",Y,M,D,h,m,s);
    return String(buf);
  }
  // fallback si aún sin fix de hora
  return String("1970-01-01T00:00:") + String((millis()/1000)%60) + "Z";
}

// genera la fecha "YYYYMMDD" para nombre de archivo. Si el GPS aún no tiene fecha válida usamos UNKWDATE
String dateYMD_fromGPS(){
  if (gps.date.isValid()) {
    int Y = gps.date.year();
    int M = gps.date.month();
    int D = gps.date.day();
    return fourDigits(Y)+twoDigits(M)+twoDigits(D); // "20251024"
  }
  return "UNKWDATE";
}

// ========= LOG A microSD =========
void ensureLogFiles() {
  if (!sdOK) return;

  String ymd = dateYMD_fromGPS();
  if (ymd != currentDateYMD) {
    if (logFile) logFile.close();
    if (evtFile) evtFile.close();

    currentDateYMD = ymd;
    String logName = "/"+String(ROBOT_ID)+"_"+currentDateYMD+".csv";
    String evtName = "/"+String(ROBOT_ID)+"_"+currentDateYMD+"_evt.csv";

    bool existedLog = SD.exists(logName);
    bool existedEvt = SD.exists(evtName);

    logFile = SD.open(logName, FILE_APPEND);
    evtFile = SD.open(evtName, FILE_APPEND);

    if (!logFile || !evtFile){
        Serial.println("[SD] Error abriendo CSV");
        sdOK=false;
        return;
    }

    if (!existedLog){
      logFile.println("ts,robotId,lat,lon,alt_m,spd_mps,mode,sats,bat,temp,pres");
      logFile.flush();
    }
    if (!existedEvt){
      evtFile.println("ts,robotId,event,payload");
      evtFile.flush();
    }

    Serial.print("[SD] Archivos activos: ");
    Serial.print(logName); Serial.print(" , "); Serial.println(evtName);
  }
}

void appendTelemetryCSV(float lat,float lon,float alt,float spd,int sats){
  if(!sdOK)return;
  ensureLogFiles();
  String ts = timestampISO8601();
  // bat=100, temp=0, pres=0 por ahora
  String line = ts + "," + ROBOT_ID + "," +
                String(lat,6)+","+String(lon,6)+","+
                String(alt,2)+","+String(spd,2)+","+
                MODE + "," + String(sats) +
                ",100,0,0";
  logFile.println(line);
  logFile.flush();
}

void appendEventCSV(const String& event,const String& payload){
  if(!sdOK)return;
  ensureLogFiles();
  String ts = timestampISO8601();
  String safe = payload;
  safe.replace("\n"," ");
  safe.replace(",",";");
  String line = ts + "," + ROBOT_ID + "," + event + ",\"" + safe + "\"";
  evtFile.println(line);
  evtFile.flush();
}

// ========= CONTROL DE MOTORES (DRV8833) =========
// Vamos a implementar helpers con PWM en las líneas del puente H.
// Velocidad base:
const uint8_t DRIVE_PWM = 200; // 0..255

// Para DRV8833, para avanzar adelante motor A:
//   AIN1 = PWM, AIN2 = LOW
// Atrás motor A:
//   AIN1 = LOW, AIN2 = PWM
//
// Igual motor B con BIN1/BIN2.

void motorA_stop() {
  ledcWrite(PWM_CH_AIN1, 0);
  ledcWrite(PWM_CH_AIN2, 0);
}

void motorB_stop() {
  ledcWrite(PWM_CH_BIN1, 0);
  ledcWrite(PWM_CH_BIN2, 0);
}

void motorA_forward(uint8_t duty) {
  ledcWrite(PWM_CH_AIN1, duty);
  ledcWrite(PWM_CH_AIN2, 0);
}

void motorA_back(uint8_t duty) {
  ledcWrite(PWM_CH_AIN1, 0);
  ledcWrite(PWM_CH_AIN2, duty);
}

void motorB_forward(uint8_t duty) {
  ledcWrite(PWM_CH_BIN1, duty);
  ledcWrite(PWM_CH_BIN2, 0);
}

void motorB_back(uint8_t duty) {
  ledcWrite(PWM_CH_BIN1, 0);
  ledcWrite(PWM_CH_BIN2, duty);
}

// Acciones de alto nivel:
void driveSTOP(){
  motorA_stop();
  motorB_stop();
  Serial.println("[DRIVE] STOP");
  appendEventCSV("DRIVE","STOP");
}

void driveFWD(){
  motorA_forward(DRIVE_PWM);
  motorB_forward(DRIVE_PWM);
  Serial.println("[DRIVE] FWD");
  appendEventCSV("DRIVE","FWD");
}

void driveBACK(){
  motorA_back(DRIVE_PWM);
  motorB_back(DRIVE_PWM);
  Serial.println("[DRIVE] BACK");
  appendEventCSV("DRIVE","BACK");
}

void driveLEFT(){
  // giro sobre eje: izq atrás, der adelante
  motorA_back(DRIVE_PWM);
  motorB_forward(DRIVE_PWM);
  Serial.println("[DRIVE] LEFT");
  appendEventCSV("DRIVE","LEFT");
}

void driveRIGHT(){
  // giro sobre eje: izq adelante, der atrás
  motorA_forward(DRIVE_PWM);
  motorB_back(DRIVE_PWM);
  Serial.println("[DRIVE] RIGHT");
  appendEventCSV("DRIVE","RIGHT");
}

// ========= TELEMETRÍA LoRa =========
void sendTelemetry(float lat,float lon,float alt,float spd,int sats){
  char buf[360];
  snprintf(buf,sizeof(buf),
    "{\"robotId\":\"%s\",\"ts\":\"%s\","
    "\"pos\":{\"type\":\"Point\",\"coordinates\":[%.6f,%.6f]},"
    "\"alt\":%.2f,\"temp\":0.00,\"pres\":0,\"spd\":%.2f,"
    "\"mode\":\"%s\",\"sats\":%d,\"bat\":100}",
    ROBOT_ID,
    timestampISO8601().c_str(),
    lon,lat,
    alt,spd,
    MODE.c_str(),sats
  );

  LoRa.beginPacket();
  LoRa.print(buf);
  LoRa.endPacket();

  Serial.print("[TX LoRa] "); Serial.println(buf);

  appendTelemetryCSV(lat,lon,alt,spd,sats);
}

// ========= PROCESAR COMANDOS ENTRANTES =========
void processCommand(const String& cmd){
  Serial.print("[CMD RX] "); Serial.println(cmd);
  appendEventCSV("CMD_RX",cmd);

  // SET_MODE
  if(cmd.indexOf("\"cmd\":\"SET_MODE\"")>=0){
    if(cmd.indexOf("\"mode\":\"AUTO\"")>=0){
      MODE="AUTO";
      Serial.println("[ACT] MODO -> AUTO");
      appendEventCSV("MODE_SET","AUTO");
    } else if(cmd.indexOf("\"mode\":\"MANUAL\"")>=0){
      MODE="MANUAL";
      Serial.println("[ACT] MODO -> MANUAL");
      appendEventCSV("MODE_SET","MANUAL");
    }
  }

  // SET_RATE
  else if(cmd.indexOf("\"cmd\":\"SET_RATE\"")>=0){
    int i=cmd.indexOf("\"period_ms\":");
    if(i>=0){
      unsigned long ms=cmd.substring(i+12).toInt();
      if(ms>=500 && ms<=10000){
        PERIOD_MS=ms;
        Serial.printf("[ACT] NUEVO PERIODO=%lu ms\n",ms);
        appendEventCSV("SET_RATE",String(ms));
      }
    }
  }

  // PING
  else if(cmd.indexOf("\"cmd\":\"PING\"")>=0){
    delay(100);
    LoRa.beginPacket();
    LoRa.print("{\"ack\":\"PONG\"}");
    LoRa.endPacket();
    Serial.println("[ACT] PONG enviado");
    appendEventCSV("PING_ACK","PONG");
  }

  // DRIVE (mando de movimiento)
  else if(cmd.indexOf("\"cmd\":\"DRIVE\"")>=0){
    if(cmd.indexOf("\"dir\":\"FWD\"")>=0){
      driveFWD();
    } else if(cmd.indexOf("\"dir\":\"BACK\"")>=0){
      driveBACK();
    } else if(cmd.indexOf("\"dir\":\"LEFT\"")>=0){
      driveLEFT();
    } else if(cmd.indexOf("\"dir\":\"RIGHT\"")>=0){
      driveRIGHT();
    } else if(cmd.indexOf("\"dir\":\"STOP\"")>=0){
      driveSTOP();
    }
  }
}

// ========= SETUP =========
void setup(){
  Serial.begin(115200);
  delay(500);
  Serial.println("===== ROBOT ESP32 (LoRa+GPS+SD+DRV8833) =====");

  // Motores: configurar PWM por canal
  ledcSetup(PWM_CH_AIN1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_AIN2, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_BIN1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_BIN2, PWM_FREQ, PWM_RES);

  ledcAttachPin(AIN1, PWM_CH_AIN1);
  ledcAttachPin(AIN2, PWM_CH_AIN2);
  ledcAttachPin(BIN1, PWM_CH_BIN1);
  ledcAttachPin(BIN2, PWM_CH_BIN2);

  driveSTOP(); // asegurar que arranca frenado

  // GPS
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("[GPS] OK");

  // LoRa
  SPI.begin(18,19,23,LORA_SS);
  LoRa.setPins(LORA_SS,LORA_RST,LORA_DIO0);
  if(!LoRa.begin(LORA_BAND)){
    Serial.println("[LoRa] FALLO begin() revisa antena/pines/frecuencia");
    while(true) delay(1000);
  }
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(LORA_SYNC);
  Serial.println("[LoRa] OK @ 433 MHz");

  // microSD
  if(SD.begin(SD_CS,SPI,25000000)){
    sdOK=true;
    Serial.println("[SD] Inicializada ✅");
    ensureLogFiles(); // crea/abre CSVs del día (o UNKWDATE)
  }else{
    sdOK=false;
    Serial.println("[SD] No inicializó ❌ revisa cableado/CS/3.3V");
  }
}

// ========= LOOP PRINCIPAL =========
void loop(){
  // Siempre ir leyendo NMEA del GPS
  while (SerialGPS.available()) gps.encode(SerialGPS.read());

  // Telemetría periódica
  if (millis() - lastSend > PERIOD_MS){
    lastSend = millis();

    float lat = gps.location.isValid()? gps.location.lat() : 0;
    float lon = gps.location.isValid()? gps.location.lng() : 0;
    float alt = gps.altitude.isValid()? gps.altitude.meters() : 0;
    float spd = gps.speed.isValid()?    gps.speed.mps()    : 0;
    int sats  = gps.satellites.value();

    sendTelemetry(lat,lon,alt,spd,sats);
    Serial.printf("[GPS] sats=%d hdop=%.2f\n", sats, gps.hdop.hdop());

    // Ventana de escucha ~700ms para recibir comandos downlink
    unsigned long t0 = millis();
    while (millis() - t0 < 700){
      int p = LoRa.parsePacket();
      if(p){
        String incoming;
        while(LoRa.available()) incoming += (char)LoRa.read();
        processCommand(incoming);
      }
      while (SerialGPS.available()) gps.encode(SerialGPS.read());
    }
  }

  // Escucha adicional fuera de ventana
  int p = LoRa.parsePacket();
  if(p){
    String incoming;
    while(LoRa.available()) incoming += (char)LoRa.read();
    processCommand(incoming);
  }
}
