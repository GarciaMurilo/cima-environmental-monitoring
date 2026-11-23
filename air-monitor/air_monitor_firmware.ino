/**
 * Estacao de qualidade do ar - ESP32 + PMS5003 + BME280 + microSD
 * Versao 4.1.0 - 2026-08-11
 *
 * Principais decisoes:
 * - PMS5003 em modo ativo e ligado continuamente.
 * - Parser proprio do frame de 32 bytes, com validacao de cabecalho,
 *   tamanho e checksum conforme o manual Plantower V2.3.
 * - Agregacao em cadencia fixa de 5 s para nao ponderar excessivamente
 *   periodos em que o PMS5003 transmite mais frames.
 * - BME280 em modo FORCED, adequado a uma estacao ambiental de baixa taxa.
 * - CSV no SD e fila duravel NDJSON para envio posterior a nuvem.
 * - Portal de configuracao Wi-Fi sem SSID/senha gravados no codigo.
 * - Dashboard local e API JSON em tempo real.
 * - Horario UTC por NTP e coordenadas configuraveis por dispositivo.
 *
 * Alvo: ESP32 Dev Module / ESP32 DevKit V1
 * Arduino-ESP32: validado com a versao 3.3.10
 * Bibliotecas externas:
 *   - Adafruit BME280 Library
 *   - Adafruit Unified Sensor
 * Bibliotecas nativas do core ESP32:
 *   WiFi, WebServer, DNSServer, Preferences, HTTPClient,
 *   WiFiClientSecure, ESPmDNS, SD, SPI, Wire
 *
 * Pinagem padrao:
 *   PMS5003 TX -> GPIO16 (RX2 do ESP32)
 *   PMS5003 RX -> GPIO17 (TX2 do ESP32; reservado para comandos futuros)
 *   PMS5003 SET -> GPIO27
 *   PMS5003 RESET -> GPIO26
 *   BME280 SDA/SCL -> GPIO21/GPIO22
 *   microSD CS/SCK/MISO/MOSI -> GPIO5/GPIO18/GPIO19/GPIO23
 *
 * IMPORTANTE PARA HTTPS:
 * Cole em CLOUD_ROOT_CA o certificado raiz da autoridade que assina o
 * endpoint da futura plataforma. HTTPS sem validacao de certificado nao e
 * aceito neste firmware. Para testes locais, endpoint http:// e permitido.
 */

#include <Adafruit_BME280.h>
#include <Arduino.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <SD.h>
#include <SPI.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <esp_arduino_version.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <time.h>

// -----------------------------------------------------------------------------
// Hardware e tempos
// -----------------------------------------------------------------------------

constexpr uint8_t PMS_RX_PIN = 16;
constexpr uint8_t PMS_TX_PIN = 17;
constexpr uint8_t PMS_SET_PIN = 27;
constexpr uint8_t PMS_RESET_PIN = 26;
constexpr uint8_t SD_CS_PIN = 5;
constexpr uint8_t SD_SCK_PIN = 18;
constexpr uint8_t SD_MISO_PIN = 19;
constexpr uint8_t SD_MOSI_PIN = 23;
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint8_t CONFIG_BUTTON_PIN = 0; // Botao BOOT da maioria dos DevKit V1

constexpr uint32_t PMS_BAUD = 9600;
constexpr size_t PMS_RX_BUFFER_BYTES = 2048;
constexpr uint32_t PMS_WARMUP_MS = 30000;
constexpr uint32_t PMS_FRAME_MAX_AGE_MS = 5000;
constexpr uint32_t PMS_RESET_AFTER_MS = 12000;
constexpr uint32_t PMS_RESET_COOLDOWN_MS = 60000;
constexpr uint32_t FIXED_SAMPLE_TICK_MS = 5000;
constexpr uint32_t WIFI_RETRY_MS = 30000;
constexpr uint32_t WIFI_PORTAL_AFTER_MS = 180000;
constexpr uint32_t CONFIG_BUTTON_HOLD_MS = 3000;
constexpr uint32_t WATCHDOG_TIMEOUT_SEC = 30;
constexpr uint32_t OUTBOX_MAX_BYTES = 4UL * 1024UL * 1024UL;
constexpr size_t OUTBOX_MAX_LINE_BYTES = 4096;
constexpr uint32_t CLOUD_TASK_STACK_BYTES = 8192;

constexpr char OUTBOX_FILE[] = "/cloud_outbox.ndjson";
constexpr char OUTBOX_ACK_FILE[] = "/cloud_outbox.ack";
constexpr char OUTBOX_ACK_TMP_FILE[] = "/cloud_outbox.ack.tmp";

constexpr char NTP_SERVER_1[] = "pool.ntp.org";
constexpr char NTP_SERVER_2[] = "time.google.com";

// Preencher somente quando o endpoint HTTPS definitivo estiver definido.
const char CLOUD_ROOT_CA[] PROGMEM = R"EOF(
)EOF";

// -----------------------------------------------------------------------------
// Estruturas
// -----------------------------------------------------------------------------

struct DeviceConfig {
  String ssid;
  String wifiPassword;
  String deviceId;
  String cloudUrl;
  String apiKey;
  double latitude = 0.0;
  double longitude = 0.0;
  bool locationValid = false;
  float temperatureOffsetC = 0.0F;
  uint32_t recordIntervalSec = 60;
};

struct PmsFrame {
  uint16_t pm1Sp = 0;
  uint16_t pm25Sp = 0;
  uint16_t pm10Sp = 0;
  uint16_t pm1Ae = 0;
  uint16_t pm25Ae = 0;
  uint16_t pm10Ae = 0;
  uint16_t particles03 = 0;
  uint16_t particles05 = 0;
  uint16_t particles10 = 0;
  uint16_t particles25 = 0;
  uint16_t particles50 = 0;
  uint16_t particles100 = 0;
};

struct EnvironmentData {
  float temperatureC = NAN;
  float humidityPct = NAN;
  float pressureHpa = NAN;
  bool valid = false;
  uint32_t sampledAtMs = 0;
};

struct IntervalAccumulator {
  double pm1Sum = 0;
  double pm25Sum = 0;
  double pm10Sum = 0;
  double p03Sum = 0;
  double p05Sum = 0;
  double p10Sum = 0;
  double p25Sum = 0;
  double p50Sum = 0;
  double p100Sum = 0;
  uint16_t pm1Min = UINT16_MAX;
  uint16_t pm25Min = UINT16_MAX;
  uint16_t pm10Min = UINT16_MAX;
  uint16_t pm1Max = 0;
  uint16_t pm25Max = 0;
  uint16_t pm10Max = 0;
  uint16_t pmsSamples = 0;

  double temperatureSum = 0;
  double humiditySum = 0;
  double pressureSum = 0;
  uint16_t environmentSamples = 0;
};

struct MeasurementRecord {
  char measurementId[72] = {};
  char timestampUtc[25] = {};
  bool timeValid = false;
  uint64_t uptimeMs = 0;
  uint32_t intervalSec = 0;

  bool pmsValid = false;
  float pm1Mean = NAN;
  float pm25Mean = NAN;
  float pm10Mean = NAN;
  uint16_t pm1Min = 0;
  uint16_t pm25Min = 0;
  uint16_t pm10Min = 0;
  uint16_t pm1Max = 0;
  uint16_t pm25Max = 0;
  uint16_t pm10Max = 0;
  float particles03Mean = NAN;
  float particles05Mean = NAN;
  float particles10Mean = NAN;
  float particles25Mean = NAN;
  float particles50Mean = NAN;
  float particles100Mean = NAN;
  uint16_t pmsSamples = 0;

  bool bmeValid = false;
  float temperatureMean = NAN;
  float humidityMean = NAN;
  float pressureMean = NAN;
  uint16_t environmentSamples = 0;
  int wifiRssi = 0;
};

struct CloudUploadJob {
  String payload;
  String url;
  String apiKey;
  String status;
  uint32_t nextOutboxOffset = 0;
  bool queueBacked = false;
  bool success = false;
};

// -----------------------------------------------------------------------------
// Objetos e estado global
// -----------------------------------------------------------------------------

HardwareSerial pmsSerial(2);
Adafruit_BME280 bme;
WebServer server(80);
DNSServer dnsServer;
Preferences preferences;

DeviceConfig config;
PmsFrame latestPms;
EnvironmentData latestEnvironment;
IntervalAccumulator accumulator;
MeasurementRecord latestRecord;

uint8_t pmsBuffer[32] = {};
uint8_t pmsBufferIndex = 0;
uint32_t pmsReadyAtMs = 0;
uint32_t lastValidPmsFrameMs = 0;
uint32_t lastPmsResetMs = 0;
uint32_t validPmsFrames = 0;
uint32_t pmsChecksumErrors = 0;
uint32_t pmsFormatErrors = 0;

bool bmeAvailable = false;
uint8_t bmeAddress = 0;
bool sdAvailable = false;
bool portalActive = false;
bool wifiWasConnected = false;
bool ntpConfigured = false;
bool mdnsStarted = false;
bool watchdogEnabled = false;
bool webServerStarted = false;

uint32_t bootId = 0;
uint32_t measurementSequence = 0;
uint32_t lastFixedSampleMs = 0;
uint32_t lastRecordMs = 0;
uint32_t lastWifiAttemptMs = 0;
uint32_t wifiDisconnectedSinceMs = 0;
uint32_t configButtonPressedAtMs = 0;
uint32_t restartAtMs = 0;
uint32_t lastMillisObserved = 0;
uint64_t millisWrapBase = 0;

uint32_t outboxAckOffset = 0;
uint32_t cloudNextAttemptMs = 0;
uint32_t cloudBackoffMs = 2000;
uint32_t cloudSentCount = 0;
uint32_t cloudFailureCount = 0;
uint32_t cloudQueueDropCount = 0;
String lastCloudStatus = "desativado";
QueueHandle_t cloudResultQueue = nullptr;
bool cloudUploadActive = false;

// -----------------------------------------------------------------------------
// Pagina local (armazenada em flash)
// -----------------------------------------------------------------------------

const char DASHBOARD_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="pt-BR"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Estacao de qualidade do ar</title>
<style>
:root{color-scheme:dark;--bg:#0b1114;--card:#152027;--line:#29404c;--text:#edf7f5;--muted:#91aaa6;--accent:#56d6b2;--warn:#ffc857;--bad:#ff6b6b}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:15px system-ui,sans-serif}.wrap{max-width:980px;margin:auto;padding:24px}h1{margin:0;font-size:1.55rem}.sub{color:var(--muted);margin:6px 0 22px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:12px}.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:16px}.card h2{color:var(--muted);font-size:.78rem;letter-spacing:.08em;text-transform:uppercase;margin:0 0 12px}.big{font-size:2rem;font-weight:700}.unit{font-size:.85rem;color:var(--muted)}.row{display:flex;justify-content:space-between;gap:12px;padding:4px 0}.row span:last-child{text-align:right}.status{margin-top:12px}.ok{color:var(--accent)}.warn{color:var(--warn)}.bad{color:var(--bad)}footer{color:var(--muted);font-size:.82rem;margin-top:18px}a{color:var(--accent)}
</style></head><body><main class="wrap">
<h1 id="device">Estacao de qualidade do ar</h1><div class="sub" id="location">Carregando...</div>
<section class="grid">
<article class="card"><h2>PM1.0 agora</h2><div class="big"><span id="pm1">--</span> <span class="unit">ug/m3</span></div><div class="row"><span>Media do intervalo</span><span id="pm1m">--</span></div></article>
<article class="card"><h2>PM2.5 agora</h2><div class="big"><span id="pm25">--</span> <span class="unit">ug/m3</span></div><div class="row"><span>Media do intervalo</span><span id="pm25m">--</span></div></article>
<article class="card"><h2>PM10 agora</h2><div class="big"><span id="pm10">--</span> <span class="unit">ug/m3</span></div><div class="row"><span>Media do intervalo</span><span id="pm10m">--</span></div></article>
<article class="card"><h2>Ambiente</h2><div class="row"><span>Temperatura</span><span id="temp">--</span></div><div class="row"><span>Umidade</span><span id="hum">--</span></div><div class="row"><span>Pressao</span><span id="press">--</span></div></article>
<article class="card status"><h2>Estado</h2><div class="row"><span>PMS5003</span><span id="pmsState">--</span></div><div class="row"><span>Wi-Fi</span><span id="wifi">--</span></div><div class="row"><span>Cartao SD</span><span id="sd">--</span></div><div class="row"><span>Relogio</span><span id="clock">--</span></div></article>
<article class="card status"><h2>Nuvem</h2><div class="row"><span>Estado</span><span id="cloud">--</span></div><div class="row"><span>Fila pendente</span><span id="queue">--</span></div><div class="row"><span>Ultimo registro</span><span id="timestamp">--</span></div></article>
</section>
<footer>API local: <a href="/api/v1/live">/api/v1/live</a> e <a href="/api/v1/status">/api/v1/status</a>. Para reconfigurar, mantenha o botao BOOT pressionado por 3 segundos.</footer>
</main><script>
const text=(id,v,s='')=>document.getElementById(id).textContent=(v===null||v===undefined?'--':v+s);
const state=(id,label,kind)=>{const e=document.getElementById(id);e.textContent=label;e.className=kind};
async function update(){try{const r=await fetch('/api/v1/live',{cache:'no-store'});const d=await r.json();
text('device',d.device_id);text('location',d.location_valid?(d.latitude.toFixed(6)+', '+d.longitude.toFixed(6)):'localizacao ainda nao configurada');
text('pm1',d.live.pm1_0);text('pm25',d.live.pm2_5);text('pm10',d.live.pm10);
text('pm1m',d.record.pm1_0_mean,' ug/m3');text('pm25m',d.record.pm2_5_mean,' ug/m3');text('pm10m',d.record.pm10_mean,' ug/m3');
text('temp',d.environment.temperature_c,' C');text('hum',d.environment.humidity_pct,' %');text('press',d.environment.pressure_hpa,' hPa');
state('pmsState',d.pms_warming?'aquecendo':(d.pms_fresh?'online':'sem dados'),d.pms_fresh?'ok':'warn');
state('wifi',d.wifi.connected?d.wifi.rssi+' dBm':'offline',d.wifi.connected?'ok':'bad');state('sd',d.sd_ok?'gravando':'erro',d.sd_ok?'ok':'bad');state('clock',d.time_valid?'UTC sincronizado':'sem NTP',d.time_valid?'ok':'warn');
text('cloud',d.cloud.status);text('queue',d.cloud.pending_bytes,' bytes');text('timestamp',d.record.timestamp_utc);
}catch(e){state('wifi','sem resposta','bad')}}update();setInterval(update,2000);
</script></body></html>
)HTML";

// -----------------------------------------------------------------------------
// Utilitarios
// -----------------------------------------------------------------------------

uint16_t be16(const uint8_t *data) {
  return (static_cast<uint16_t>(data[0]) << 8) | data[1];
}

bool elapsed(uint32_t now, uint32_t since, uint32_t interval) {
  return static_cast<uint32_t>(now - since) >= interval;
}

uint64_t monotonicUptimeMs() {
  const uint32_t now = millis();
  if (now < lastMillisObserved)
    millisWrapBase += (1ULL << 32);
  lastMillisObserved = now;
  return millisWrapBase + now;
}

void feedWatchdog() {
  if (watchdogEnabled)
    esp_task_wdt_reset();
}

String htmlEscape(const String &input) {
  String output;
  output.reserve(input.length() + 16);
  for (size_t i = 0; i < input.length(); ++i) {
    switch (input[i]) {
    case '&':
      output += F("&amp;");
      break;
    case '<':
      output += F("&lt;");
      break;
    case '>':
      output += F("&gt;");
      break;
    case '"':
      output += F("&quot;");
      break;
    case '\'':
      output += F("&#39;");
      break;
    default:
      output += input[i];
    }
  }
  return output;
}

String sanitizeIdentifier(const String &input) {
  String output;
  output.reserve(31);
  for (size_t i = 0; i < input.length() && output.length() < 31; ++i) {
    const char c = input[i];
    if (isalnum(static_cast<unsigned char>(c)) || c == '-' || c == '_')
      output += c;
  }
  return output;
}

String defaultDeviceId() {
  const uint64_t mac = ESP.getEfuseMac();
  char id[24];
  snprintf(id, sizeof(id), "NEVOA-%06llX", mac & 0xFFFFFFULL);
  return String(id);
}

String portalSsid() {
  String prefix = config.deviceId;
  if (prefix.length() > 24)
    prefix.remove(24);
  return prefix + F("-CONFIG");
}

String portalPassword() {
  return "CIMA2026!";
}

String mdnsHostname() {
  String host = config.deviceId;
  host.toLowerCase();
  host.replace('_', '-');
  return host;
}

bool clockIsValid() { return time(nullptr) > 1700000000; }

void formatUtc(char *output, size_t outputSize) {
  if (!clockIsValid()) {
    output[0] = '\0';
    return;
  }
  const time_t now = time(nullptr);
  struct tm utc = {};
  gmtime_r(&now, &utc);
  strftime(output, outputSize, "%Y-%m-%dT%H:%M:%SZ", &utc);
}

String jsonNumberOrNull(float value, unsigned int decimals) {
  if (!isfinite(value)) {
    return String(F("null"));
  }

  return String(value, decimals);
}

uint32_t outboxPendingBytes() {
  if (!sdAvailable || !SD.exists(OUTBOX_FILE))
    return 0;
  File file = SD.open(OUTBOX_FILE, FILE_READ);
  if (!file)
    return 0;
  const uint32_t size = file.size();
  file.close();
  return size > outboxAckOffset ? size - outboxAckOffset : 0;
}

// -----------------------------------------------------------------------------
// Configuracao persistente
// -----------------------------------------------------------------------------

void loadConfig() {
  preferences.begin("airmon", false);
  config.ssid = preferences.getString("ssid", "");
  config.wifiPassword = preferences.getString("pass", "");
  config.deviceId = preferences.getString("device", defaultDeviceId());
  config.cloudUrl = preferences.getString("cloud_url", "");
  config.apiKey = preferences.getString("api_key", "");
  config.latitude = preferences.getDouble("lat", 0.0);
  config.longitude = preferences.getDouble("lon", 0.0);
  config.locationValid = preferences.getBool("loc_set", false);
  config.temperatureOffsetC = preferences.getFloat("temp_off", 0.0F);
  config.recordIntervalSec = preferences.getUInt("interval", 60);

  config.deviceId = sanitizeIdentifier(config.deviceId);
  if (config.deviceId.isEmpty())
    config.deviceId = defaultDeviceId();
  config.recordIntervalSec = constrain(config.recordIntervalSec, 30UL, 3600UL);
}

void saveConfig() {
  preferences.putString("ssid", config.ssid);
  preferences.putString("pass", config.wifiPassword);
  preferences.putString("device", config.deviceId);
  preferences.putString("cloud_url", config.cloudUrl);
  preferences.putString("api_key", config.apiKey);
  preferences.putDouble("lat", config.latitude);
  preferences.putDouble("lon", config.longitude);
  preferences.putBool("loc_set", config.locationValid);
  preferences.putFloat("temp_off", config.temperatureOffsetC);
  preferences.putUInt("interval", config.recordIntervalSec);
}

// -----------------------------------------------------------------------------
// PMS5003 - parser nao bloqueante do frame ativo de 32 bytes
// -----------------------------------------------------------------------------

bool decodePmsFrame(const uint8_t *frame, PmsFrame &output) {
  if (frame[0] != 0x42 || frame[1] != 0x4D || be16(frame + 2) != 28) {
    ++pmsFormatErrors;
    return false;
  }

  uint16_t checksum = 0;
  for (uint8_t i = 0; i < 30; ++i)
    checksum += frame[i];
  if (checksum != be16(frame + 30)) {
    ++pmsChecksumErrors;
    return false;
  }

  output.pm1Sp = be16(frame + 4);
  output.pm25Sp = be16(frame + 6);
  output.pm10Sp = be16(frame + 8);
  output.pm1Ae = be16(frame + 10);
  output.pm25Ae = be16(frame + 12);
  output.pm10Ae = be16(frame + 14);
  output.particles03 = be16(frame + 16);
  output.particles05 = be16(frame + 18);
  output.particles10 = be16(frame + 20);
  output.particles25 = be16(frame + 22);
  output.particles50 = be16(frame + 24);
  output.particles100 = be16(frame + 26);
  return true;
}

void consumePmsByte(uint8_t value) {
  if (pmsBufferIndex == 0) {
    if (value == 0x42)
      pmsBuffer[pmsBufferIndex++] = value;
    return;
  }
  if (pmsBufferIndex == 1 && value != 0x4D) {
    pmsBufferIndex = (value == 0x42) ? 1 : 0;
    pmsBuffer[0] = 0x42;
    return;
  }

  pmsBuffer[pmsBufferIndex++] = value;
  if (pmsBufferIndex == 4 && be16(pmsBuffer + 2) != 28) {
    ++pmsFormatErrors;
    pmsBufferIndex = 0;
    return;
  }
  if (pmsBufferIndex < sizeof(pmsBuffer))
    return;

  PmsFrame decoded;
  if (decodePmsFrame(pmsBuffer, decoded)) {
    latestPms = decoded;
    lastValidPmsFrameMs = millis();
    ++validPmsFrames;
  }
  pmsBufferIndex = 0;
}

void pollPmsSerial() {
  while (pmsSerial.available() > 0)
    consumePmsByte(static_cast<uint8_t>(pmsSerial.read()));
}

void resetPmsHardware() {
  const uint32_t now = millis();
  if (!elapsed(now, lastPmsResetMs, PMS_RESET_COOLDOWN_MS))
    return;
  Serial.println(F("[PMS] Sem frames validos; reset de hardware."));
  digitalWrite(PMS_RESET_PIN, LOW);
  delay(100);
  digitalWrite(PMS_RESET_PIN, HIGH);
  pmsBufferIndex = 0;
  lastPmsResetMs = now;
  lastValidPmsFrameMs = 0;
  pmsReadyAtMs = millis() + PMS_WARMUP_MS;
}

void maintainPms() {
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - pmsReadyAtMs) < 0)
    return;
  if (lastValidPmsFrameMs == 0 ||
      elapsed(now, lastValidPmsFrameMs, PMS_RESET_AFTER_MS)) {
    resetPmsHardware();
  }
}

// -----------------------------------------------------------------------------
// BME280 e agregacao em cadencia fixa
// -----------------------------------------------------------------------------

bool initBme() {
  if (bme.begin(0x76, &Wire))
    bmeAddress = 0x76;
  else if (bme.begin(0x77, &Wire))
    bmeAddress = 0x77;
  else
    return false;

  bme.setSampling(Adafruit_BME280::MODE_FORCED, Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::SAMPLING_X1, Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::FILTER_OFF);
  Serial.printf("[BME] Detectado em 0x%02X.\n", bmeAddress);
  return true;
}

void sampleBme() {
  if (!bmeAvailable) {
    bmeAvailable = initBme();
    if (!bmeAvailable) {
      latestEnvironment.valid = false;
      return;
    }
  }

  if (!bme.takeForcedMeasurement()) {
    latestEnvironment.valid = false;
    bmeAvailable = false;
    return;
  }

  const float temperature = bme.readTemperature() + config.temperatureOffsetC;
  const float humidity = bme.readHumidity();
  const float pressure = bme.readPressure() / 100.0F;
  const bool sane =
      isfinite(temperature) && isfinite(humidity) && isfinite(pressure) &&
      temperature >= -40.0F && temperature <= 85.0F && humidity >= 0.0F &&
      humidity <= 100.0F && pressure >= 300.0F && pressure <= 1100.0F;
  if (!sane) {
    latestEnvironment.valid = false;
    return;
  }

  latestEnvironment.temperatureC = temperature;
  latestEnvironment.humidityPct = humidity;
  latestEnvironment.pressureHpa = pressure;
  latestEnvironment.valid = true;
  latestEnvironment.sampledAtMs = millis();
}

void resetAccumulator() { accumulator = IntervalAccumulator(); }

void takeFixedSample() {
  sampleBme();
  const uint32_t now = millis();

  if (static_cast<int32_t>(now - pmsReadyAtMs) >= 0 &&
      lastValidPmsFrameMs != 0 &&
      !elapsed(now, lastValidPmsFrameMs, PMS_FRAME_MAX_AGE_MS)) {
    accumulator.pm1Sum += latestPms.pm1Ae;
    accumulator.pm25Sum += latestPms.pm25Ae;
    accumulator.pm10Sum += latestPms.pm10Ae;
    accumulator.p03Sum += latestPms.particles03;
    accumulator.p05Sum += latestPms.particles05;
    accumulator.p10Sum += latestPms.particles10;
    accumulator.p25Sum += latestPms.particles25;
    accumulator.p50Sum += latestPms.particles50;
    accumulator.p100Sum += latestPms.particles100;
    accumulator.pm1Min = min(accumulator.pm1Min, latestPms.pm1Ae);
    accumulator.pm25Min = min(accumulator.pm25Min, latestPms.pm25Ae);
    accumulator.pm10Min = min(accumulator.pm10Min, latestPms.pm10Ae);
    accumulator.pm1Max = max(accumulator.pm1Max, latestPms.pm1Ae);
    accumulator.pm25Max = max(accumulator.pm25Max, latestPms.pm25Ae);
    accumulator.pm10Max = max(accumulator.pm10Max, latestPms.pm10Ae);
    ++accumulator.pmsSamples;
  }

  if (latestEnvironment.valid) {
    accumulator.temperatureSum += latestEnvironment.temperatureC;
    accumulator.humiditySum += latestEnvironment.humidityPct;
    accumulator.pressureSum += latestEnvironment.pressureHpa;
    ++accumulator.environmentSamples;
  }
}

MeasurementRecord finishRecord() {
  MeasurementRecord record;
  record.uptimeMs = monotonicUptimeMs();
  record.intervalSec = config.recordIntervalSec;
  record.timeValid = clockIsValid();
  formatUtc(record.timestampUtc, sizeof(record.timestampUtc));
  record.wifiRssi = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;

  ++measurementSequence;
  if (record.timeValid) {
    snprintf(record.measurementId, sizeof(record.measurementId), "%s-%lu-%lu",
             config.deviceId.c_str(), static_cast<unsigned long>(time(nullptr)),
             static_cast<unsigned long>(measurementSequence));
  } else {
    snprintf(record.measurementId, sizeof(record.measurementId), "%s-%08lX-%lu",
             config.deviceId.c_str(), static_cast<unsigned long>(bootId),
             static_cast<unsigned long>(measurementSequence));
  }

  if (accumulator.pmsSamples > 0) {
    const float n = accumulator.pmsSamples;
    record.pmsValid = true;
    record.pm1Mean = accumulator.pm1Sum / n;
    record.pm25Mean = accumulator.pm25Sum / n;
    record.pm10Mean = accumulator.pm10Sum / n;
    record.pm1Min = accumulator.pm1Min;
    record.pm25Min = accumulator.pm25Min;
    record.pm10Min = accumulator.pm10Min;
    record.pm1Max = accumulator.pm1Max;
    record.pm25Max = accumulator.pm25Max;
    record.pm10Max = accumulator.pm10Max;
    record.particles03Mean = accumulator.p03Sum / n;
    record.particles05Mean = accumulator.p05Sum / n;
    record.particles10Mean = accumulator.p10Sum / n;
    record.particles25Mean = accumulator.p25Sum / n;
    record.particles50Mean = accumulator.p50Sum / n;
    record.particles100Mean = accumulator.p100Sum / n;
    record.pmsSamples = accumulator.pmsSamples;
  }

  if (accumulator.environmentSamples > 0) {
    const float n = accumulator.environmentSamples;
    record.bmeValid = true;
    record.temperatureMean = accumulator.temperatureSum / n;
    record.humidityMean = accumulator.humiditySum / n;
    record.pressureMean = accumulator.pressureSum / n;
    record.environmentSamples = accumulator.environmentSamples;
  }
  return record;
}

// -----------------------------------------------------------------------------
// Cartao SD
// -----------------------------------------------------------------------------

bool initSd() {
  SD.end();
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN, SPI, 4000000U)) {
    Serial.println(F("[SD] Falha ao montar o cartao."));
    return false;
  }
  if (SD.cardType() == CARD_NONE) {
    Serial.println(F("[SD] Nenhum cartao detectado."));
    SD.end();
    return false;
  }
  Serial.printf("[SD] OK: %llu MB.\n", SD.cardSize() / (1024ULL * 1024ULL));
  return true;
}

String logFilename(const MeasurementRecord &record) {
  if (!record.timeValid)
    return String(F("/air_quality_unsynced.csv"));
  char filename[28];
  snprintf(filename, sizeof(filename), "/air_quality_%.4s%.2s%.2s.csv",
           record.timestampUtc, record.timestampUtc + 5,
           record.timestampUtc + 8);
  return String(filename);
}

bool ensureCsvHeader(const String &filename) {
  if (SD.exists(filename))
    return true;
  File file = SD.open(filename, FILE_WRITE);
  if (!file)
    return false;
  file.println(
      F("schema_version,measurement_id,timestamp_utc,time_valid,uptime_ms,"
        "device_id,location_valid,latitude,longitude,interval_s,pm1_0_mean,pm2_"
        "5_mean,pm10_"
        "mean,pm1_0_min,pm2_5_min,pm10_min,pm1_0_max,pm2_5_max,pm10_max,"
        "particles_gt_0_3_mean,particles_gt_0_5_mean,particles_gt_1_0_mean,"
        "particles_gt_2_5_mean,particles_gt_5_0_mean,particles_gt_10_mean,"
        "temperature_mean_c,humidity_mean_pct,pressure_mean_hpa,pms_samples,"
        "bme_samples,wifi_rssi_dbm,pms_valid,bme_valid"));
  file.flush();
  const bool written = file.getWriteError() == 0;
  file.close();
  return written;
}

void printCsvFloat(File &file, float value, uint8_t decimals = 2) {
  if (isfinite(value))
    file.print(value, decimals);
}

bool appendCsv(const MeasurementRecord &record) {
  if (!sdAvailable)
    sdAvailable = initSd();
  if (!sdAvailable)
    return false;

  const String filename = logFilename(record);
  if (!ensureCsvHeader(filename)) {
    sdAvailable = false;
    return false;
  }
  File file = SD.open(filename, FILE_APPEND);
  if (!file) {
    sdAvailable = false;
    return false;
  }

  file.print(F("1,"));
  file.print(record.measurementId);
  file.print(',');
  file.print(record.timestampUtc);
  file.print(',');
  file.print(record.timeValid ? 1 : 0);
  file.print(',');
  file.print(record.uptimeMs);
  file.print(',');
  file.print(config.deviceId);
  file.print(',');
  file.print(config.locationValid ? 1 : 0);
  file.print(',');
  file.print(config.latitude, 6);
  file.print(',');
  file.print(config.longitude, 6);
  file.print(',');
  file.print(record.intervalSec);
  file.print(',');
  printCsvFloat(file, record.pm1Mean);
  file.print(',');
  printCsvFloat(file, record.pm25Mean);
  file.print(',');
  printCsvFloat(file, record.pm10Mean);
  file.print(',');
  if (record.pmsValid)
    file.print(record.pm1Min);
  file.print(',');
  if (record.pmsValid)
    file.print(record.pm25Min);
  file.print(',');
  if (record.pmsValid)
    file.print(record.pm10Min);
  file.print(',');
  if (record.pmsValid)
    file.print(record.pm1Max);
  file.print(',');
  if (record.pmsValid)
    file.print(record.pm25Max);
  file.print(',');
  if (record.pmsValid)
    file.print(record.pm10Max);
  file.print(',');
  printCsvFloat(file, record.particles03Mean);
  file.print(',');
  printCsvFloat(file, record.particles05Mean);
  file.print(',');
  printCsvFloat(file, record.particles10Mean);
  file.print(',');
  printCsvFloat(file, record.particles25Mean);
  file.print(',');
  printCsvFloat(file, record.particles50Mean);
  file.print(',');
  printCsvFloat(file, record.particles100Mean);
  file.print(',');
  printCsvFloat(file, record.temperatureMean);
  file.print(',');
  printCsvFloat(file, record.humidityMean);
  file.print(',');
  printCsvFloat(file, record.pressureMean);
  file.print(',');
  file.print(record.pmsSamples);
  file.print(',');
  file.print(record.environmentSamples);
  file.print(',');
  file.print(record.wifiRssi);
  file.print(',');
  file.print(record.pmsValid ? 1 : 0);
  file.print(',');
  file.println(record.bmeValid ? 1 : 0);
  file.flush();
  const bool written = file.getWriteError() == 0;
  file.close();
  if (!written)
    sdAvailable = false;
  return written;
}

bool writeOutboxAck(uint32_t offset) {
  if (SD.exists(OUTBOX_ACK_TMP_FILE))
    SD.remove(OUTBOX_ACK_TMP_FILE);
  File file = SD.open(OUTBOX_ACK_TMP_FILE, FILE_WRITE);
  if (!file)
    return false;
  const size_t written = file.print(offset);
  file.flush();
  const bool writeOk = written > 0 && file.getWriteError() == 0;
  file.close();
  if (!writeOk) {
    SD.remove(OUTBOX_ACK_TMP_FILE);
    return false;
  }
  if (SD.exists(OUTBOX_ACK_FILE) && !SD.remove(OUTBOX_ACK_FILE))
    return false;
  return SD.rename(OUTBOX_ACK_TMP_FILE, OUTBOX_ACK_FILE);
}

void loadOutboxAck() {
  outboxAckOffset = 0;
  if (!sdAvailable)
    return;
  if (!SD.exists(OUTBOX_FILE)) {
    if (SD.exists(OUTBOX_ACK_FILE))
      SD.remove(OUTBOX_ACK_FILE);
    if (SD.exists(OUTBOX_ACK_TMP_FILE))
      SD.remove(OUTBOX_ACK_TMP_FILE);
    return;
  }
  if (!SD.exists(OUTBOX_ACK_FILE) && SD.exists(OUTBOX_ACK_TMP_FILE))
    SD.rename(OUTBOX_ACK_TMP_FILE, OUTBOX_ACK_FILE);
  else if (SD.exists(OUTBOX_ACK_TMP_FILE))
    SD.remove(OUTBOX_ACK_TMP_FILE);
  if (!SD.exists(OUTBOX_ACK_FILE))
    return;
  File file = SD.open(OUTBOX_ACK_FILE, FILE_READ);
  if (!file)
    return;
  outboxAckOffset = static_cast<uint32_t>(file.parseInt());
  file.close();

  File outbox = SD.open(OUTBOX_FILE, FILE_READ);
  if (!outbox) {
    outboxAckOffset = 0;
    return;
  }
  const uint32_t size = outbox.size();
  bool boundaryValid = outboxAckOffset <= size;
  if (boundaryValid && outboxAckOffset > 0) {
    boundaryValid = outbox.seek(outboxAckOffset - 1) && outbox.read() == '\n';
  }
  outbox.close();
  if (!boundaryValid) {
    Serial.println(F("[Nuvem] ACK da fila invalido; reenvio desde o inicio."));
    outboxAckOffset = 0;
    writeOutboxAck(0);
  }
}

bool enqueueCloudJson(const String &json) {
  if (config.cloudUrl.isEmpty())
    return true;
  if (!sdAvailable)
    return false;
  if (!SD.exists(OUTBOX_FILE)) {
    outboxAckOffset = 0;
    if (SD.exists(OUTBOX_ACK_FILE))
      SD.remove(OUTBOX_ACK_FILE);
    if (SD.exists(OUTBOX_ACK_TMP_FILE))
      SD.remove(OUTBOX_ACK_TMP_FILE);
  }
  const uint32_t pending = outboxPendingBytes();
  if (pending + json.length() + 1 > OUTBOX_MAX_BYTES) {
    ++cloudQueueDropCount;
    lastCloudStatus = "fila cheia; CSV preservado";
    return false;
  }
  File file = SD.open(OUTBOX_FILE, FILE_APPEND);
  if (!file)
    return false;
  const size_t written = file.println(json);
  file.flush();
  const bool writeOk = file.getWriteError() == 0 &&
                       (written == json.length() + 2 ||
                        written == json.length() + 1);
  file.close();
  if (!writeOk)
    sdAvailable = false;
  return writeOk;
}

// -----------------------------------------------------------------------------
// JSON e nuvem
// -----------------------------------------------------------------------------

String recordToJson(const MeasurementRecord &record) {
  String json;
  json.reserve(1200);
  json += F("{\"schema_version\":1,\"measurement_id\":\"");
  json += record.measurementId;
  json += F("\",\"device_id\":\"");
  json += config.deviceId;
  json += F("\",\"timestamp_utc\":");
  if (record.timeValid) {
    json += '"';
    json += record.timestampUtc;
    json += '"';
  } else
    json += F("null");
  json += F(",\"time_valid\":");
  json += record.timeValid ? F("true") : F("false");
  json += F(",\"uptime_ms\":");
  json += record.uptimeMs;
  json += F(",\"location\":{\"valid\":");
  json += config.locationValid ? F("true") : F("false");
  json += F(",\"latitude\":");
  json += String(config.latitude, 6);
  json += F(",\"longitude\":");
  json += String(config.longitude, 6);
  json += '}';
  json += F(",\"interval_s\":");
  json += record.intervalSec;
  json += F(",\"pm\":{\"valid\":");
  json += record.pmsValid ? F("true") : F("false");
  json += F(",\"unit\":\"ug/m3\",\"fixed_tick_s\":5,\"samples\":");
  json += record.pmsSamples;
  json += F(",\"pm1_0_mean\":");
  json += jsonNumberOrNull(record.pm1Mean, 2);
  json += F(",\"pm2_5_mean\":");
  json += jsonNumberOrNull(record.pm25Mean, 2);
  json += F(",\"pm10_mean\":");
  json += jsonNumberOrNull(record.pm10Mean, 2);
  json += F(",\"pm1_0_min\":");
  json += record.pmsValid ? String(record.pm1Min) : String(F("null"));
  json += F(",\"pm2_5_min\":");
  json += record.pmsValid ? String(record.pm25Min) : String(F("null"));
  json += F(",\"pm10_min\":");
  json += record.pmsValid ? String(record.pm10Min) : String(F("null"));
  json += F(",\"pm1_0_max\":");
  json += record.pmsValid ? String(record.pm1Max) : String(F("null"));
  json += F(",\"pm2_5_max\":");
  json += record.pmsValid ? String(record.pm25Max) : String(F("null"));
  json += F(",\"pm10_max\":");
  json += record.pmsValid ? String(record.pm10Max) : String(F("null"));
  json += '}';
  json += F(",\"particles\":{\"unit\":\"count/0.1L\",\"gt_0_3_mean\":");
  json += jsonNumberOrNull(record.particles03Mean, 2);
  json += F(",\"gt_0_5_mean\":");
  json += jsonNumberOrNull(record.particles05Mean, 2);
  json += F(",\"gt_1_0_mean\":");
  json += jsonNumberOrNull(record.particles10Mean, 2);
  json += F(",\"gt_2_5_mean\":");
  json += jsonNumberOrNull(record.particles25Mean, 2);
  json += F(",\"gt_5_0_mean\":");
  json += jsonNumberOrNull(record.particles50Mean, 2);
  json += F(",\"gt_10_mean\":");
  json += jsonNumberOrNull(record.particles100Mean, 2);
  json += '}';
  json += F(",\"environment\":{\"valid\":");
  json += record.bmeValid ? F("true") : F("false");
  json += F(",\"samples\":");
  json += record.environmentSamples;
  json += F(",\"temperature_mean_c\":");
  json += jsonNumberOrNull(record.temperatureMean, 2);
  json += F(",\"humidity_mean_pct\":");
  json += jsonNumberOrNull(record.humidityMean, 2);
  json += F(",\"pressure_mean_hpa\":");
  json += jsonNumberOrNull(record.pressureMean, 2);
  json += '}';
  json += F(",\"diagnostics\":{\"wifi_rssi_dbm\":");
  json += record.wifiRssi;
  json += F(",\"pms_checksum_errors\":");
  json += pmsChecksumErrors;
  json += F(",\"pms_format_errors\":");
  json += pmsFormatErrors;
  json += F(",\"sd_ok\":");
  json += sdAvailable ? F("true") : F("false");
  json += F("}}");
  return json;
}

String measurementIdFromJson(const String &json) {
  const String marker = F("\"measurement_id\":\"");
  const int start = json.indexOf(marker);
  if (start < 0)
    return String();
  const int valueStart = start + marker.length();
  const int end = json.indexOf('"', valueStart);
  return end > valueStart ? json.substring(valueStart, end) : String();
}

bool performCloudPost(const CloudUploadJob &job, String &statusText) {
  HTTPClient http;
  WiFiClient plainClient;
  WiFiClientSecure secureClient;
  bool begun = false;

  if (job.url.startsWith("https://")) {
    if (strlen_P(CLOUD_ROOT_CA) < 32) {
      statusText = "certificado CA ausente";
      return false;
    }
    secureClient.setCACert(CLOUD_ROOT_CA);
    begun = http.begin(secureClient, job.url);
  } else if (job.url.startsWith("http://")) {
    begun = http.begin(plainClient, job.url);
  } else {
    statusText = "URL invalida";
    return false;
  }
  if (!begun) {
    statusText = "falha ao iniciar HTTP";
    return false;
  }

  http.setConnectTimeout(2000);
  http.setTimeout(4000);
  http.addHeader(F("Content-Type"), F("application/json"));
  const String measurementId = measurementIdFromJson(job.payload);
  if (!measurementId.isEmpty())
    http.addHeader(F("Idempotency-Key"), measurementId);
  if (!job.apiKey.isEmpty())
    http.addHeader(F("X-API-Key"), job.apiKey);
  const int httpStatus = http.POST(job.payload);
  http.end();

  if (httpStatus >= 200 && httpStatus < 300) {
    statusText = "online";
    return true;
  }
  statusText = httpStatus > 0 ? String(F("HTTP ")) + httpStatus
                              : String(F("erro ")) + httpStatus;
  return false;
}

void cloudUploadTask(void *parameter) {
  CloudUploadJob *job = static_cast<CloudUploadJob *>(parameter);
  if (WiFi.status() != WL_CONNECTED) {
    job->status = "Wi-Fi desconectado";
    job->success = false;
  } else {
    job->success = performCloudPost(*job, job->status);
  }
  xQueueSend(cloudResultQueue, &job, portMAX_DELAY);
  vTaskDelete(nullptr);
}

bool startCloudUpload(const String &payload, bool queueBacked,
                      uint32_t nextOutboxOffset) {
  if (cloudUploadActive || cloudResultQueue == nullptr ||
      WiFi.status() != WL_CONNECTED || config.cloudUrl.isEmpty())
    return false;
  if (config.cloudUrl.startsWith("https://") && !clockIsValid()) {
    lastCloudStatus = "aguardando horario NTP para TLS";
    return false;
  }

  CloudUploadJob *job = new CloudUploadJob();
  if (job == nullptr) {
    lastCloudStatus = "memoria insuficiente para envio";
    return false;
  }
  job->payload = payload;
  job->url = config.cloudUrl;
  job->apiKey = config.apiKey;
  job->queueBacked = queueBacked;
  job->nextOutboxOffset = nextOutboxOffset;

  cloudUploadActive = true;
  const BaseType_t created =
      xTaskCreate(cloudUploadTask, "cloudUpload", CLOUD_TASK_STACK_BYTES, job, 1,
                  nullptr);
  if (created != pdPASS) {
    cloudUploadActive = false;
    lastCloudStatus = "falha ao criar tarefa HTTP";
    delete job;
    return false;
  }
  lastCloudStatus = "enviando";
  return true;
}

void handleCloudUploadResult() {
  if (cloudResultQueue == nullptr)
    return;
  CloudUploadJob *job = nullptr;
  if (xQueueReceive(cloudResultQueue, &job, 0) != pdTRUE || job == nullptr)
    return;

  cloudUploadActive = false;
  lastCloudStatus = job->status;
  if (job->success) {
    ++cloudSentCount;
    cloudBackoffMs = 2000;
    cloudNextAttemptMs = millis() + cloudBackoffMs;
    if (job->queueBacked) {
      outboxAckOffset = job->nextOutboxOffset;
      if (!writeOutboxAck(outboxAckOffset))
        Serial.println(F("[Nuvem] Aviso: falha ao atualizar o ACK da fila."));

      File file = SD.open(OUTBOX_FILE, FILE_READ);
      const uint32_t currentSize = file ? file.size() : 0;
      if (file)
        file.close();
      if (currentSize > 0 && outboxAckOffset >= currentSize)
        clearCompletedOutbox();
    }
  } else {
    ++cloudFailureCount;
    cloudBackoffMs = min(cloudBackoffMs * 2UL, 300000UL);
    cloudNextAttemptMs = millis() + cloudBackoffMs;
  }
  delete job;
}

void clearCompletedOutbox() {
  if (SD.exists(OUTBOX_FILE))
    SD.remove(OUTBOX_FILE);
  if (SD.exists(OUTBOX_ACK_FILE))
    SD.remove(OUTBOX_ACK_FILE);
  if (SD.exists(OUTBOX_ACK_TMP_FILE))
    SD.remove(OUTBOX_ACK_TMP_FILE);
  outboxAckOffset = 0;
}

bool readBoundedOutboxLine(File &file, String &payload, bool &tooLong,
                           bool &lineComplete) {
  payload = String();
  payload.reserve(1600);
  tooLong = false;
  lineComplete = false;
  while (file.available()) {
    const int raw = file.read();
    if (raw < 0)
      break;
    const char value = static_cast<char>(raw);
    if (value == '\n') {
      lineComplete = true;
      break;
    }
    if (value == '\r')
      continue;
    if (payload.length() < OUTBOX_MAX_LINE_BYTES)
      payload += value;
    else
      tooLong = true;
  }
  payload.trim();
  return lineComplete || !payload.isEmpty() || tooLong;
}

void processOutbox() {
  handleCloudUploadResult();
  if (cloudUploadActive)
    return;
  if (!sdAvailable || config.cloudUrl.isEmpty() ||
      WiFi.status() != WL_CONNECTED)
    return;
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - cloudNextAttemptMs) < 0)
    return;
  if (config.cloudUrl.startsWith("https://") && !clockIsValid()) {
    lastCloudStatus = "aguardando horario NTP para TLS";
    cloudNextAttemptMs = now + 5000UL;
    return;
  }
  if (!SD.exists(OUTBOX_FILE)) {
    lastCloudStatus = "fila vazia";
    return;
  }

  File file = SD.open(OUTBOX_FILE, FILE_READ);
  if (!file)
    return;
  const uint32_t size = file.size();
  if (outboxAckOffset >= size) {
    file.close();
    clearCompletedOutbox();
    return;
  }
  if (!file.seek(outboxAckOffset)) {
    file.close();
    outboxAckOffset = 0;
    writeOutboxAck(0);
    return;
  }
  String payload;
  bool tooLong = false;
  bool lineComplete = false;
  const bool hasLine =
      readBoundedOutboxLine(file, payload, tooLong, lineComplete);
  const uint32_t nextOffset = file.position();
  file.close();
  if (!hasLine || payload.isEmpty()) {
    outboxAckOffset = nextOffset;
    writeOutboxAck(outboxAckOffset);
    return;
  }
  const bool structurallyValid = !tooLong && payload.startsWith("{") &&
                                 payload.endsWith("}") &&
                                 !measurementIdFromJson(payload).isEmpty();
  if (!structurallyValid) {
    Serial.println(F("[Nuvem] Linha corrompida na fila; CSV foi preservado."));
    ++cloudQueueDropCount;
    outboxAckOffset = nextOffset;
    writeOutboxAck(outboxAckOffset);
    if (outboxAckOffset >= size)
      clearCompletedOutbox();
    return;
  }
  startCloudUpload(payload, true, nextOffset);
}

// -----------------------------------------------------------------------------
// Wi-Fi, NTP, mDNS e portal de configuracao
// -----------------------------------------------------------------------------

void onWifiConnected() {
  wifiWasConnected = true;
  wifiDisconnectedSinceMs = 0;
  Serial.printf("[WiFi] Conectado a %s, IP %s, RSSI %d dBm.\n",
                WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(),
                WiFi.RSSI());
  if (!ntpConfigured) {
    configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2);
    ntpConfigured = true;
  }
  if (mdnsStarted)
    MDNS.end();
  mdnsStarted = MDNS.begin(mdnsHostname().c_str());
  if (mdnsStarted)
    MDNS.addService("http", "tcp", 80);
}

void startPortal() {
  if (portalActive)
    return;
  WiFi.mode(WIFI_AP_STA);
  const String apName = portalSsid();
  const String apPassword = portalPassword();
  if (!WiFi.softAP(apName.c_str(), apPassword.c_str())) {
    Serial.println(F("[Portal] Falha ao criar ponto de acesso."));
    return;
  }
  dnsServer.start(53, "*", WiFi.softAPIP());
  portalActive = true;
  Serial.printf(
      "[Portal] Rede: %s | senha: %s | endereco: http://192.168.4.1\n",
      apName.c_str(), apPassword.c_str());
}

void beginStaConnection() {
  if (config.ssid.isEmpty())
    return;
  WiFi.mode(portalActive ? WIFI_AP_STA : WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.begin(config.ssid.c_str(), config.wifiPassword.c_str());
  lastWifiAttemptMs = millis();
  if (wifiDisconnectedSinceMs == 0)
    wifiDisconnectedSinceMs = millis();
}

void connectWifiAtBoot() {
  if (config.ssid.isEmpty()) {
    startPortal();
    return;
  }
  beginStaConnection();
  const uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && !elapsed(millis(), started, 15000)) {
    pollPmsSerial();
    feedWatchdog();
    delay(20);
  }
  if (WiFi.status() == WL_CONNECTED)
    onWifiConnected();
  else
    startPortal();
}

void maintainWifi() {
  const bool connected = WiFi.status() == WL_CONNECTED;
  if (connected && !wifiWasConnected)
    onWifiConnected();
  if (connected)
    return;

  if (wifiWasConnected) {
    wifiWasConnected = false;
    wifiDisconnectedSinceMs = millis();
    Serial.println(F("[WiFi] Conexao perdida; dados continuam no SD."));
  }
  if (wifiDisconnectedSinceMs == 0)
    wifiDisconnectedSinceMs = millis();
  if (!config.ssid.isEmpty() &&
      elapsed(millis(), lastWifiAttemptMs, WIFI_RETRY_MS))
    beginStaConnection();
  if (!portalActive &&
      elapsed(millis(), wifiDisconnectedSinceMs, WIFI_PORTAL_AFTER_MS))
    startPortal();
}

void handleConfigButton() {
  if (digitalRead(CONFIG_BUTTON_PIN) == LOW) {
    if (configButtonPressedAtMs == 0)
      configButtonPressedAtMs = millis();
    else if (!portalActive &&
             elapsed(millis(), configButtonPressedAtMs, CONFIG_BUTTON_HOLD_MS))
      startPortal();
  } else
    configButtonPressedAtMs = 0;
}

String configPage(const String &message = String()) {
  String page;
  page.reserve(5000);
  page += F(
      "<!doctype html><html lang='pt-BR'><head><meta charset='utf-8'><meta "
      "name='viewport' "
      "content='width=device-width,initial-scale=1'><title>Configurar "
      "estacao</title><style>body{max-width:680px;margin:30px auto;padding:0 "
      "18px;font:16px "
      "system-ui;background:#0b1114;color:#edf7f5}form{display:grid;gap:12px;"
      "background:#152027;padding:20px;border-radius:14px}label{display:grid;"
      "gap:5px}input{font:inherit;padding:10px;border:1px solid "
      "#48606b;border-radius:8px;background:#0b1114;color:#fff}button{padding:"
      "12px;border:0;border-radius:8px;background:#56d6b2;font-weight:700}.msg{"
      "padding:10px;background:#29404c;border-radius:8px}.hint{color:#91aaa6;"
      "font-size:.88rem}</style></head><body><h1>Configurar estacao</h1>");
  if (!message.isEmpty()) {
    page += F("<p class='msg'>");
    page += htmlEscape(message);
    page += F("</p>");
  }
  page += F("<form method='post' action='/save'><label>Nome da rede "
            "Wi-Fi<input name='ssid' maxlength='32' required value='");
  page += htmlEscape(config.ssid);
  page += F("'></label>");
  page += F(
      "<label>Senha do Wi-Fi<input type='password' name='wifi_password' "
      "maxlength='63' placeholder='Deixe vazio para manter a atual'></label>");
  page += F("<label>ID do dispositivo<input name='device_id' maxlength='31' "
            "required value='");
  page += htmlEscape(config.deviceId);
  page += F("'></label>");
  page += F("<label>Latitude<input name='latitude' type='number' min='-90' "
            "max='90' step='0.000001' required value='");
  page += String(config.latitude, 6);
  page += F("'></label>");
  page += F("<label>Longitude<input name='longitude' type='number' min='-180' "
            "max='180' step='0.000001' required value='");
  page += String(config.longitude, 6);
  page += F("'></label>");
  page += F("<label>Intervalo do registro (s)<input name='interval' "
            "type='number' min='30' max='3600' required value='");
  page += config.recordIntervalSec;
  page += F("'></label>");
  page += F("<label>Correcao de temperatura (C)<input name='temp_offset' "
            "type='number' min='-10' max='10' step='0.1' value='");
  page += String(config.temperatureOffsetC, 1);
  page += F("'></label>");
  page += F("<label>Endpoint da nuvem (opcional)<input name='cloud_url' "
            "maxlength='191' placeholder='https://.../measurements' value='");
  page += htmlEscape(config.cloudUrl);
  page += F("'></label>");
  page += F(
      "<label>Chave da API (opcional)<input type='password' name='api_key' "
      "maxlength='95' placeholder='Deixe vazio para manter a atual'></label>");
  page += F("<button type='submit'>Salvar e reiniciar</button><div "
            "class='hint'>A senha Wi-Fi e a chave da API nao sao exibidas. "
            "HTTPS so envia dados depois que o certificado raiz do endpoint "
            "for inserido no firmware.</div></form></body></html>");
  return page;
}

void handleSaveConfig() {
  if (!portalActive) {
    server.send(403, "text/plain", "Portal de configuracao inativo.");
    return;
  }
  const String newSsid = server.arg("ssid");
  const String newDevice = sanitizeIdentifier(server.arg("device_id"));
  const double newLatitude = server.arg("latitude").toDouble();
  const double newLongitude = server.arg("longitude").toDouble();
  const uint32_t newInterval =
      static_cast<uint32_t>(server.arg("interval").toInt());
  const float newOffset = server.arg("temp_offset").toFloat();
  const String newCloudUrl = server.arg("cloud_url");

  if (newSsid.isEmpty() || newDevice.isEmpty() || newLatitude < -90 ||
      newLatitude > 90 || newLongitude < -180 || newLongitude > 180 ||
      newInterval < 30 || newInterval > 3600 || newOffset < -10 ||
      newOffset > 10 ||
      (!newCloudUrl.isEmpty() && !newCloudUrl.startsWith("http://") &&
       !newCloudUrl.startsWith("https://"))) {
    server.send(400, "text/html; charset=utf-8",
                configPage("Revise os campos: ha um valor invalido."));
    return;
  }

  const String submittedWifiPassword = server.arg("wifi_password");
  const String submittedApiKey = server.arg("api_key");
  if (newSsid != config.ssid || !submittedWifiPassword.isEmpty())
    config.wifiPassword = submittedWifiPassword;
  if (!submittedApiKey.isEmpty())
    config.apiKey = submittedApiKey;
  config.ssid = newSsid;
  config.deviceId = newDevice;
  config.latitude = newLatitude;
  config.longitude = newLongitude;
  config.locationValid = true;
  config.recordIntervalSec = newInterval;
  config.temperatureOffsetC = newOffset;
  config.cloudUrl = newCloudUrl;
  saveConfig();

  server.send(
      200, "text/html; charset=utf-8",
      configPage("Configuracao salva. A estacao reiniciara em instantes."));
  restartAtMs = millis() + 1500;
}

// -----------------------------------------------------------------------------
// API e servidor web local
// -----------------------------------------------------------------------------

String liveJson() {
  const uint32_t now = millis();
  const bool warming = static_cast<int32_t>(now - pmsReadyAtMs) < 0;
  const bool pmsFresh =
      !warming && lastValidPmsFrameMs != 0 &&
      !elapsed(now, lastValidPmsFrameMs, PMS_FRAME_MAX_AGE_MS);
  String json;
  json.reserve(1500);
  json += F("{\"device_id\":\"");
  json += config.deviceId;
  json += F("\",\"latitude\":");
  json += String(config.latitude, 6);
  json += F(",\"longitude\":");
  json += String(config.longitude, 6);
  json += F(",\"location_valid\":");
  json += config.locationValid ? F("true") : F("false");
  json += F(",\"pms_warming\":");
  json += warming ? F("true") : F("false");
  json += F(",\"pms_fresh\":");
  json += pmsFresh ? F("true") : F("false");
  json += F(",\"live\":{\"pm1_0\":");
  json += pmsFresh ? String(latestPms.pm1Ae) : String(F("null"));
  json += F(",\"pm2_5\":");
  json += pmsFresh ? String(latestPms.pm25Ae) : String(F("null"));
  json += F(",\"pm10\":");
  json += pmsFresh ? String(latestPms.pm10Ae) : String(F("null"));
  json += F(",\"frame_age_ms\":");
  json += lastValidPmsFrameMs ? String(now - lastValidPmsFrameMs)
                              : String(F("null"));
  json += '}';
  json += F(",\"environment\":{\"temperature_c\":");
  json += jsonNumberOrNull(latestEnvironment.temperatureC, 1);
  json += F(",\"humidity_pct\":");
  json += jsonNumberOrNull(latestEnvironment.humidityPct, 1);
  json += F(",\"pressure_hpa\":");
  json += jsonNumberOrNull(latestEnvironment.pressureHpa, 1);
  json += '}';
  json += F(",\"record\":{\"measurement_id\":\"");
  json += latestRecord.measurementId;
  json += F("\",\"timestamp_utc\":");
  if (latestRecord.timeValid) {
    json += '"';
    json += latestRecord.timestampUtc;
    json += '"';
  } else
    json += F("null");
  json += F(",\"pm1_0_mean\":");
  json += jsonNumberOrNull(latestRecord.pm1Mean, 1);
  json += F(",\"pm2_5_mean\":");
  json += jsonNumberOrNull(latestRecord.pm25Mean, 1);
  json += F(",\"pm10_mean\":");
  json += jsonNumberOrNull(latestRecord.pm10Mean, 1);
  json += '}';
  json += F(",\"wifi\":{\"connected\":");
  json += WiFi.status() == WL_CONNECTED ? F("true") : F("false");
  json += F(",\"rssi\":");
  json +=
      WiFi.status() == WL_CONNECTED ? String(WiFi.RSSI()) : String(F("null"));
  json += F(",\"ip\":\"");
  json += WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString()
                                        : WiFi.softAPIP().toString();
  json += F("\"}");
  json += F(",\"sd_ok\":");
  json += sdAvailable ? F("true") : F("false");
  json += F(",\"time_valid\":");
  json += clockIsValid() ? F("true") : F("false");
  json += F(",\"cloud\":{\"status\":\"");
  json += lastCloudStatus;
  json += F("\",\"pending_bytes\":");
  json += outboxPendingBytes();
  json += F("}}");
  return json;
}

String statusJson() {
  String json;
  json.reserve(900);
  json += F("{\"firmware_version\":\"4.1.0\",\"uptime_ms\":");
  json += monotonicUptimeMs();
  json += F(",\"free_heap_bytes\":");
  json += ESP.getFreeHeap();
  json += F(",\"valid_pms_frames\":");
  json += validPmsFrames;
  json += F(",\"pms_checksum_errors\":");
  json += pmsChecksumErrors;
  json += F(",\"pms_format_errors\":");
  json += pmsFormatErrors;
  json += F(",\"bme_address\":");
  json += bmeAvailable ? String(bmeAddress) : String(F("null"));
  json += F(",\"cloud_sent\":");
  json += cloudSentCount;
  json += F(",\"cloud_failures\":");
  json += cloudFailureCount;
  json += F(",\"cloud_queue_drops\":");
  json += cloudQueueDropCount;
  json += F(",\"portal_active\":");
  json += portalActive ? F("true") : F("false");
  json += F(",\"watchdog_enabled\":");
  json += watchdogEnabled ? F("true") : F("false");
  json += F(",\"cloud_upload_active\":");
  json += cloudUploadActive ? F("true") : F("false");
  json += '}';
  return json;
}

void setupWebServer() {
  if (webServerStarted)
    return;
  server.on("/", HTTP_GET, []() {
    const bool requestCameThroughAp =
        portalActive && server.client().localIP() == WiFi.softAPIP();
    if (requestCameThroughAp)
      server.send(200, "text/html; charset=utf-8", configPage());
    else
      server.send_P(200, "text/html; charset=utf-8", DASHBOARD_HTML);
  });
  server.on("/config", HTTP_GET, []() {
    if (!portalActive)
      server.send(403, "text/plain",
                  "Mantenha BOOT pressionado por 3 segundos para ativar a "
                  "configuracao.");
    else
      server.send(200, "text/html; charset=utf-8", configPage());
  });
  server.on("/save", HTTP_POST, handleSaveConfig);
  server.on("/api/v1/live", HTTP_GET,
            []() { server.send(200, "application/json", liveJson()); });
  server.on("/api/v1/status", HTTP_GET,
            []() { server.send(200, "application/json", statusJson()); });
  server.on("/generate_204", HTTP_GET, []() {
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });
  server.on("/hotspot-detect.html", HTTP_GET, []() {
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });
  server.onNotFound([]() {
    if (portalActive) {
      server.sendHeader("Location", "/", true);
      server.send(302, "text/plain", "");
    } else
      server.send(404, "application/json", "{\"error\":\"not_found\"}");
  });
  server.begin();
  webServerStarted = true;
  Serial.println(F("[Web] Servidor local iniciado na porta 80."));
}

// -----------------------------------------------------------------------------
// Watchdog e ciclo de registro
// -----------------------------------------------------------------------------

void setupWatchdog() {
  esp_err_t configureResult = ESP_FAIL;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t watchdogConfig = {};
  watchdogConfig.timeout_ms = WATCHDOG_TIMEOUT_SEC * 1000UL;
  watchdogConfig.idle_core_mask = 0;
  watchdogConfig.trigger_panic = true;
  configureResult = esp_task_wdt_reconfigure(&watchdogConfig);
  if (configureResult == ESP_ERR_INVALID_STATE)
    configureResult = esp_task_wdt_init(&watchdogConfig);
#else
  configureResult = esp_task_wdt_init(WATCHDOG_TIMEOUT_SEC, true);
#endif
  if (configureResult != ESP_OK) {
    Serial.printf("[WDT] Falha ao configurar: %s.\n",
                  esp_err_to_name(configureResult));
    return;
  }

  esp_err_t subscriptionResult = esp_task_wdt_status(nullptr);
  if (subscriptionResult == ESP_ERR_NOT_FOUND)
    subscriptionResult = esp_task_wdt_add(nullptr);
  if (subscriptionResult != ESP_OK) {
    Serial.printf("[WDT] Falha ao registrar loopTask: %s.\n",
                  esp_err_to_name(subscriptionResult));
    return;
  }
  watchdogEnabled = true;
  Serial.printf("[WDT] loopTask protegida; timeout=%lu s.\n",
                static_cast<unsigned long>(WATCHDOG_TIMEOUT_SEC));
}

void publishRecord() {
  latestRecord = finishRecord();
  const bool saved = appendCsv(latestRecord);
  if (!saved)
    Serial.println(F("[Registro] ERRO: nao foi possivel gravar o CSV."));

  if (!config.cloudUrl.isEmpty()) {
    const String json = recordToJson(latestRecord);
    if (sdAvailable) {
      if (!enqueueCloudJson(json))
        Serial.println(F("[Nuvem] Registro ficou apenas no CSV."));
    } else if (!startCloudUpload(json, false, 0)) {
      Serial.println(F("[Nuvem] Sem SD e sem envio: registro existe apenas na "
                       "memoria atual."));
    }
  }

  Serial.printf(
      "[Registro] %s | PM2.5=%s | T=%s | PMS n=%u | BME n=%u | SD=%s\n",
      latestRecord.measurementId,
      latestRecord.pmsValid ? String(latestRecord.pm25Mean, 1).c_str() : "NA",
      latestRecord.bmeValid ? String(latestRecord.temperatureMean, 1).c_str()
                            : "NA",
      latestRecord.pmsSamples, latestRecord.environmentSamples,
      saved ? "OK" : "ERRO");
  resetAccumulator();
}

// -----------------------------------------------------------------------------
// setup e loop
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(300);
  bootId = esp_random();
  Serial.println(F("\n=== Estacao de qualidade do ar v4.1.0 ==="));

  pinMode(PMS_SET_PIN, OUTPUT);
  pinMode(PMS_RESET_PIN, OUTPUT);
  pinMode(CONFIG_BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(PMS_SET_PIN, HIGH);
  digitalWrite(PMS_RESET_PIN, HIGH);
  pmsSerial.setRxBufferSize(PMS_RX_BUFFER_BYTES);
  pmsSerial.begin(PMS_BAUD, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
  pmsReadyAtMs = millis() + PMS_WARMUP_MS;
  lastPmsResetMs = millis() - PMS_RESET_COOLDOWN_MS;
  Serial.printf("[PMS] UART2 em %lu bps; aquecimento=%lu s.\n",
                static_cast<unsigned long>(PMS_BAUD),
                static_cast<unsigned long>(PMS_WARMUP_MS / 1000UL));

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  loadConfig();
  sdAvailable = initSd();
  loadOutboxAck();
  bmeAvailable = initBme();
  if (!bmeAvailable)
    Serial.println(F("[BME] Nao detectado em 0x76 nem 0x77; novas tentativas "
                     "ocorrerao durante a operacao."));
  resetAccumulator();
  cloudResultQueue = xQueueCreate(1, sizeof(CloudUploadJob *));
  if (cloudResultQueue == nullptr)
    Serial.println(F("[Nuvem] Falha ao criar fila interna de resultados."));

  setupWatchdog();
  connectWifiAtBoot();
  // WebServer::begin() exige que a pilha TCP/IP ja tenha sido inicializada
  // por WiFi.begin() ou WiFi.softAP(). Iniciar antes causa assert no Core 3.3.10.
  setupWebServer();

  lastFixedSampleMs = millis() - FIXED_SAMPLE_TICK_MS;
  lastRecordMs = millis();
  lastMillisObserved = millis();
  Serial.printf(
      "[Sistema] ID=%s | registro=%lu s | dashboard=http://%s.local/\n",
      config.deviceId.c_str(), config.recordIntervalSec,
      mdnsHostname().c_str());
}

void loop() {
  feedWatchdog();
  pollPmsSerial();
  server.handleClient();
  if (portalActive)
    dnsServer.processNextRequest();

  maintainPms();
  maintainWifi();
  handleConfigButton();

  const uint32_t now = millis();
  if (elapsed(now, lastFixedSampleMs, FIXED_SAMPLE_TICK_MS)) {
    lastFixedSampleMs += FIXED_SAMPLE_TICK_MS;
    if (elapsed(now, lastFixedSampleMs, FIXED_SAMPLE_TICK_MS))
      lastFixedSampleMs = now;
    takeFixedSample();
  }
  if (elapsed(now, lastRecordMs, config.recordIntervalSec * 1000UL)) {
    lastRecordMs += config.recordIntervalSec * 1000UL;
    if (elapsed(now, lastRecordMs, config.recordIntervalSec * 1000UL))
      lastRecordMs = now;
    publishRecord();
  }

  processOutbox();
  if (restartAtMs != 0 && static_cast<int32_t>(millis() - restartAtMs) >= 0)
    ESP.restart();
  delay(2);
}
