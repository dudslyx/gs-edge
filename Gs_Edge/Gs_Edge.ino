/*
  SmartWorkAssistant.ino
  ESP32-based Smart Work Assistant for Wokwi (DHT22 + LDR + PIR + LED + BUZZER)
  Sends telemetry via MQTT and/or HTTP.
  Autores: (Eduardo Francisco Mauro Gonçalves RM561969, Gabriel Luchetta dos Santos RM561861)
  Date: 2025-11-11

  Livrarias utilizadas:
    - DHT sensor library (Adafruit)
    - PubSubClient
    - WiFiClient
    - HTTPClient
    - Adafruit_SSD1306 / Adafruit_GFX para OLED display

 Mapeamento de pinos (padrão - altere conforme necessário):
    DHT_PIN = 4
    LDR_PIN = 34   // ADC1 channel
    PIR_PIN = 32
    BUZZER_PIN = 27
    LED_PIN = 26
    SDA = 21, SCL = 22 (OLED opicional)
*/

#include <WiFi.h>
#include "DHT.h"
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ========== CONFIGURAÇÃO DE HARDWARE ==========
#define DHTPIN 4
#define DHTTYPE DHT22

#define LDRPIN 34   // ADC1_CH6 on many ESP32 boards
#define PIRPIN 32
#define BUZZER 27
#define LEDPIN 26

DHT dht(DHTPIN, DHTTYPE);

// ========== CONFIGURAÇÃO REDE & MQTT/HTTP ==========
const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASSWORD";

// MQTT (ex.: broker público ou interno)
const bool USE_MQTT = true;
const char* MQTT_BROKER = "test.mosquitto.org"; // altere se desejar
const uint16_t MQTT_PORT = 1883;
const char* MQTT_TOPIC_TELEMETRY = "smartworkassistant/telemetry";
const char* MQTT_TOPIC_ALERT = "smartworkassistant/alerts";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// HTTP (ex.: endpoint para POST)
const bool USE_HTTP = false;
const char* HTTP_ENDPOINT = "https://example.com/api/telemetry"; // altere

// ========== PARÂMETROS DO SISTEMA ==========
const unsigned long READ_INTERVAL_MS = 5000; // leitura a cada 5s
const unsigned long ALERT_REPEAT_MS = 300000; // 5 min entre alertas
const unsigned long PRESENCE_TIMEOUT_MS = 5 * 60 * 1000UL; // após 5 min sem movimento, considera pausa
const unsigned long MIN_ACTIVE_FOR_ALERT_MS = 45 * 60 * 1000UL; // 45 min sem pausa -> alerta

// Critérios de conforto (ajuste conforme necessário)
const float TEMP_THRESHOLD = 28.0; // °C
const float HUM_THRESHOLD = 70.0;  // %
const int LDR_THRESHOLD_ADC = 800; // ADC value lower -> dark (valor de referência; ajustar)

unsigned long lastRead = 0;
unsigned long lastAlertSent = 0;

unsigned long lastPresenceSeen = 0;
unsigned long sessionStart = 0;
bool currentlyPresent = false;

// Função auxiliar: obter a hora como um timestamp baseado em milissegundos (no RTC)
unsigned long nowMillis() {
  return millis();
}

void setupWiFi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi conectado!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("");
    Serial.println("Falha ao conectar WiFi (continuando sem rede).");
  }
}

void connectMQTT() {
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  unsigned long lastTry = 0;
  if (WiFi.status() != WL_CONNECTED) return;
  while (!mqttClient.connected()) {
    Serial.print("Conectando MQTT...");
    // ID do cliente pode ser random
    String clientId = "ESP32-" + String((uint32_t)esp_random(), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("conectado");
      // subscribe se precisar
    } else {
      Serial.print("falhou, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" tentando novamente em 2s");
      delay(2000);
    }
    // Evita loop infinito bloqueante se sem rede
    if (millis() - lastTry > 30000) break;
  }
}

void sendTelemetryMQTT(JsonDocument &doc) {
  if (!USE_MQTT) return;
  if (!mqttClient.connected()) {
    connectMQTT();
  }
  if (mqttClient.connected()) {
    char buffer[512];
    size_t n = serializeJson(doc, buffer);
    bool ok = mqttClient.publish(MQTT_TOPIC_TELEMETRY, buffer, n);
    Serial.print("MQTT publish telemetry -> ");
    Serial.println(ok ? "ok" : "falha");
  } else {
    Serial.println("MQTT não conectado, pulando publish.");
  }
}

void sendAlertMQTT(const char* alertMsg, JsonDocument &doc) {
  if (!USE_MQTT) return;
  if (!mqttClient.connected()) {
    connectMQTT();
  }
  if (mqttClient.connected()) {
    // inclui campo alert na doc
    doc["alert"] = alertMsg;
    char buffer[512];
    size_t n = serializeJson(doc, buffer);
    mqttClient.publish(MQTT_TOPIC_ALERT, buffer, n);
    Serial.println("MQTT alert enviado.");
  }
}

void sendTelemetryHTTP(JsonDocument &doc) {
  if (!USE_HTTP) return;
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Sem WiFi - não pode enviar HTTP.");
    return;
  }
  HTTPClient http;
  http.begin(HTTP_ENDPOINT);
  http.addHeader("Content-Type", "application/json");
  String payload;
  serializeJson(doc, payload);
  int code = http.POST(payload);
  Serial.print("HTTP POST code: ");
  Serial.println(code);
  http.end();
}

void alertLocal(const char* reason) {
  Serial.print("ALERTA: ");
  Serial.println(reason);
  // Liga LED e buzzer por 1.5s
  digitalWrite(LEDPIN, HIGH);
  digitalWrite(BUZZER, HIGH);
  delay(1500);
  digitalWrite(BUZZER, LOW);
  digitalWrite(LEDPIN, LOW);
}

void setup() {
  Serial.begin(115200);
  pinMode(LEDPIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(PIRPIN, INPUT);
  analogSetPinAttenuation(LDRPIN, ADC_11db); // ajustar para alcance maior
  dht.begin();

  setupWiFi();
  if (USE_MQTT) connectMQTT();

  lastRead = nowMillis();
  lastPresenceSeen = 0;
  sessionStart = 0;
}

String isoTimestamp() {
  // sem RTC -> usa millis para gerar um pseudo-timestamp em segundos desde boot
  unsigned long s = millis() / 1000;
  char buf[32];
  snprintf(buf, sizeof(buf), "boot_secs_%lu", s);
  return String(buf);
}

void loop() {
  if (USE_MQTT) mqttClient.loop();

  unsigned long t = nowMillis();
  if (t - lastRead < READ_INTERVAL_MS) return;
  lastRead = t;

  // ========== LEITURAS ==========
  float temp = NAN, hum = NAN;
  temp = dht.readTemperature();
  hum = dht.readHumidity();
  int ldrRaw = analogRead(LDRPIN); // 0..4095
  int pirState = digitalRead(PIRPIN);

  // presença lógica
  if (pirState == HIGH) {
    lastPresenceSeen = t;
    if (!currentlyPresent) {
      // novo retorno / início de sessão
      currentlyPresent = true;
      sessionStart = t;
      Serial.println("Presença detectada - início de sessão.");
    }
  } else {
    // se passou mais do que PRESENCE_TIMEOUT_MS desde a última presença, considera ausente
    if (currentlyPresent && (t - lastPresenceSeen >= PRESENCE_TIMEOUT_MS)) {
      currentlyPresent = false;
      Serial.println("Presença perdida - pausa detectada, zerando sessão.");
      sessionStart = 0;
    }
  }

  unsigned long timeActiveMs = 0;
  if (currentlyPresent && sessionStart > 0) {
    timeActiveMs = t - sessionStart;
  }

  // ========== AVALIAÇÃO DOS CRITÉRIOS ==========
  String alertReason = "";
  if (!isnan(temp) && temp > TEMP_THRESHOLD) {
    alertReason += "ALTA_TEMPERATURA; ";
  }
  if (!isnan(hum) && hum > HUM_THRESHOLD) {
    alertReason += "ALTA_UMIDADE; ";
  }
  if (ldrRaw > LDR_THRESHOLD_ADC) {
    // dependendo do divisor a sensibilidade se inverte. Ajuste no README.
    alertReason += "BAIXA_LUMINOSIDADE; ";
  }
  if (timeActiveMs >= MIN_ACTIVE_FOR_ALERT_MS) {
    alertReason += "SEM_PAUSAS_PROLONGADAS; ";
  }
  bool hasAlert = alertReason.length() > 0;

  // ========== MONTAR JSON ==========
  StaticJsonDocument<512> doc;
  doc["device_id"] = "esp32_swa_001";
  doc["timestamp"] = isoTimestamp();
  if (!isnan(temp)) doc["temp_c"] = temp;
  if (!isnan(hum)) doc["hum_pct"] = hum;
  doc["ldr_raw"] = ldrRaw;
  doc["presence"] = currentlyPresent ? 1 : 0;
  doc["timeActiveSec"] = (unsigned long)(timeActiveMs / 1000);
  if (hasAlert) doc["alert"] = alertReason;

  // envia telemetria
  sendTelemetryMQTT(doc);
  sendTelemetryHTTP(doc);

  // se houver alerta e passou tempo suficiente desde último alerta, envia e aciona local
  if (hasAlert && (t - lastAlertSent >= ALERT_REPEAT_MS)) {
    lastAlertSent = t;
    sendAlertMQTT(alertReason.c_str(), doc); // também publica no tópico de alertas
    alertLocal(alertReason.c_str());
  }

  // Logs
  Serial.print("Temp: ");
  Serial.print(temp);
  Serial.print(" °C  Hum: ");
  Serial.print(hum);
  Serial.print(" %  LDR: ");
  Serial.print(ldrRaw);
  Serial.print("  Presença: ");
  Serial.print(currentlyPresent ? "SIM" : "NÃO");
  Serial.print("  timeActiveMin: ");
  Serial.println(timeActiveMs / 60000);
}
