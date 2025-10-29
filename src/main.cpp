#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <driver/i2s.h>

#include "SensorManager.h"
#include "TelemetryClient.h"

// ===== Wi-Fi credentials =====
// #define WIFI_SSID "Xiaomi_FC76"
// #define WIFI_PASSWORD "67bd7ce5ba683"
#define WIFI_SSID "TZH"
#define WIFI_PASSWORD "123456789hhh"

// ===== Python server (laptop) settings =====
#define MIC_SERVER_HOST "192.168.137.1"
#define MIC_SERVER_PORT 9000

// ===== Speaker (MAX98357A) configuration =====
#define I2S_SPK_PORT I2S_NUM_0
#define SPK_BCLK 14
#define SPK_LRC 12
#define SPK_DIN 23
#define SPK_SD 15
#define SPEAKER_SAMPLE_RATE 24000
#define SPEAKER_DMA_LEN 512

// ===== Microphone (INMP441) configuration =====
#define I2S_MIC_PORT I2S_NUM_1
#define MIC_BCLK 26
#define MIC_LRC 25
#define MIC_DOUT 34
#define MIC_SAMPLE_RATE 16000
#define MIC_FRAME_SAMPLES 256

struct PlayCommand
{
  char url[192];
};

static QueueHandle_t g_playQueue = nullptr;
static WiFiClient g_micClient;
static volatile bool g_cancelPlayback = false;
static volatile bool g_pauseMicUpload = false;
static TelemetryClient g_telemetryClient;

static void connectWiFi();
static void initSpeaker();
static void initMicrophone();
static void micStreamTask(void *param);
static bool ensureServerConnection();
static bool writeAll(WiFiClient &client, const uint8_t *data, size_t len);
static bool sendFrame(const uint8_t *data, size_t len);
static void enqueuePlay(const String &url);
static void downloadAndPlayWav(const char *url);
static void logRemote(const char *message);
static void logRemote(const String &message);

class MicUploadPauseGuard
{
public:
  explicit MicUploadPauseGuard(volatile bool &flag)
      : _flag(&flag), _prev(flag)
  {
    *_flag = true;
  }

  MicUploadPauseGuard(const MicUploadPauseGuard &) = delete;
  MicUploadPauseGuard &operator=(const MicUploadPauseGuard &) = delete;

  ~MicUploadPauseGuard()
  {
    resume();
  }

  void resume()
  {
    if (_flag)
    {
      *_flag = _prev;
      _flag = nullptr;
    }
  }

private:
  volatile bool *_flag;
  bool _prev;
};

#ifndef TELEMETRY_BASE_URL
#define TELEMETRY_BASE_URL "http://192.168.137.1:8000"
#endif
#ifndef TELEMETRY_DEVICE_ID
#define TELEMETRY_DEVICE_ID "esp32-01"
#endif

void setup()
{
  Serial.begin(115200);
  delay(200);
  Serial.println("Booting audio bridge firmware");

  g_playQueue = xQueueCreate(4, sizeof(PlayCommand));
  if (!g_playQueue)
  {
    Serial.println("Failed to create playback queue");
  }

  connectWiFi();
  initSpeaker();
  initMicrophone();

  g_telemetryClient.setBaseUrl(TELEMETRY_BASE_URL);
  g_telemetryClient.setDeviceId(TELEMETRY_DEVICE_ID);
  logRemote(String("Telemetry target: ") + TELEMETRY_BASE_URL);

  startSensorManagerTask(&g_telemetryClient);

  xTaskCreatePinnedToCore(micStreamTask, "micStream", 4096, nullptr, 1, nullptr, 0);
}

void loop()
{
  PlayCommand cmd;
  if (g_playQueue && xQueueReceive(g_playQueue, &cmd, pdMS_TO_TICKS(100)) == pdPASS)
  {
    if (cmd.url[0] != '\0')
    {
      downloadAndPlayWav(cmd.url);
    }
  }
}

static void connectWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  logRemote(String("WiFi Connected: ") + WiFi.localIP().toString());
}

static void initSpeaker()
{
  pinMode(SPK_SD, OUTPUT);
  digitalWrite(SPK_SD, LOW); // keep amplifier muted until playback starts

  i2s_config_t cfg = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = SPEAKER_SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_I2S,
      .intr_alloc_flags = 0,
      .dma_buf_count = 8,
      .dma_buf_len = SPEAKER_DMA_LEN,
      .use_apll = true,
      .tx_desc_auto_clear = true,
      .fixed_mclk = 0};

  i2s_pin_config_t pins = {
      .bck_io_num = SPK_BCLK,
      .ws_io_num = SPK_LRC,
      .data_out_num = SPK_DIN,
      .data_in_num = I2S_PIN_NO_CHANGE};

  i2s_driver_install(I2S_SPK_PORT, &cfg, 0, nullptr);
  i2s_set_pin(I2S_SPK_PORT, &pins);
}

static void initMicrophone()
{
  i2s_config_t cfg = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = MIC_SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_I2S,
      .intr_alloc_flags = 0,
      .dma_buf_count = 8,
      .dma_buf_len = MIC_FRAME_SAMPLES,
      .use_apll = false};

  i2s_pin_config_t pins = {
      .bck_io_num = MIC_BCLK,
      .ws_io_num = MIC_LRC,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = MIC_DOUT};

  i2s_driver_install(I2S_MIC_PORT, &cfg, 0, nullptr);
  i2s_set_pin(I2S_MIC_PORT, &pins);
  i2s_start(I2S_MIC_PORT);
}

static void micStreamTask(void *param)
{
  String commandBuffer;
  int32_t rawBuffer[MIC_FRAME_SAMPLES];
  int16_t pcmBuffer[MIC_FRAME_SAMPLES];

  while (true)
  {
    if (!ensureServerConnection())
    {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    size_t bytesRead = 0;
    esp_err_t err = i2s_read(I2S_MIC_PORT, rawBuffer, sizeof(rawBuffer), &bytesRead, portMAX_DELAY);
    if (err != ESP_OK || bytesRead == 0)
    {
      continue;
    }

    int samples = bytesRead / sizeof(int32_t);
    for (int i = 0; i < samples; ++i)
    {
      int32_t raw = (rawBuffer[i] >> 8) & 0x00FFFFFF;
      if (raw & 0x00800000)
      {
        raw |= 0xFF000000;
      }
      float sample = (float)raw / 8388608.0f;
      pcmBuffer[i] = (int16_t)(sample * 32767.0f);
    }

    size_t payloadBytes = samples * sizeof(int16_t);
    if (g_pauseMicUpload)
    {
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }
    if (!sendFrame((uint8_t *)pcmBuffer, payloadBytes))
    {
      Serial.println("Audio server disconnected, will retry");
      g_micClient.stop();
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    while (g_micClient.connected() && g_micClient.available())
    {
      char ch = (char)g_micClient.read();
      if (ch == '\r')
      {
        continue;
      }

      if (ch == '\n')
      {
        String line = commandBuffer;
        commandBuffer = "";
        line.trim();
        if (line.length() == 0)
        {
          continue;
        }

        if (line.startsWith("PLAY "))
        {
          String url = line.substring(5);
          url.trim();
          enqueuePlay(url);
        }
        else if (line == "STOP")
        {
          g_cancelPlayback = true;
          digitalWrite(SPK_SD, LOW);
        }
        else
        {
          Serial.print("Unknown command: ");
          Serial.println(line);
        }
      }
      else
      {
        commandBuffer += ch;
        if (commandBuffer.length() > 190)
        {
          commandBuffer = "";
        }
      }
    }
  }
}

static bool ensureServerConnection()
{
  if (g_micClient.connected())
  {
    return true;
  }

  Serial.printf("Connecting to audio bridge at %s:%d...\n", MIC_SERVER_HOST, MIC_SERVER_PORT);
  logRemote(String("Connecting to audio bridge at ") + MIC_SERVER_HOST + ":" + String(MIC_SERVER_PORT));
  if (!g_micClient.connect(MIC_SERVER_HOST, MIC_SERVER_PORT))
  {
    Serial.println("Connection failed");
    logRemote("Audio bridge connection failed");
    return false;
  }

  g_micClient.setNoDelay(true);
  char header[48];
  snprintf(header, sizeof(header), "PCM %d 16 1\n", MIC_SAMPLE_RATE);
  g_micClient.print(header);
  Serial.println("Audio server connected");
  logRemote("Audio server connected");
  return true;
}

static bool writeAll(WiFiClient &client, const uint8_t *data, size_t len)
{
  size_t offset = 0;
  while (offset < len)
  {
    size_t written = client.write(data + offset, len - offset);
    if (written == 0)
    {
      if (!client.connected())
      {
        return false;
      }
      delay(1);
      continue;
    }
    offset += written;
  }
  return true;
}

static bool sendFrame(const uint8_t *data, size_t len)
{
  if (!g_micClient.connected())
  {
    return false;
  }

  uint8_t header[4];
  header[0] = (uint8_t)((len >> 24) & 0xFF);
  header[1] = (uint8_t)((len >> 16) & 0xFF);
  header[2] = (uint8_t)((len >> 8) & 0xFF);
  header[3] = (uint8_t)(len & 0xFF);

  if (!writeAll(g_micClient, header, sizeof(header)))
  {
    logRemote("Failed to send frame header, disconnecting");
    return false;
  }
  return writeAll(g_micClient, data, len);
}

static void enqueuePlay(const String &url)
{
  if (!g_playQueue)
  {
    return;
  }

  PlayCommand cmd = {};
  url.toCharArray(cmd.url, sizeof(cmd.url));
  if (xQueueSend(g_playQueue, &cmd, 0) != pdPASS)
  {
    Serial.println("Playback queue full, dropping PLAY command");
  }
}

static void downloadAndPlayWav(const char *url)
{
  if (!url || url[0] == '\0')
  {
    return;
  }

  Serial.printf("Downloading WAV: %s\n", url);
  logRemote(String("Downloading WAV: ") + url);
  HTTPClient http;
  if (!http.begin(url))
  {
    Serial.println("HTTP begin failed");
    logRemote("HTTP begin failed for playback");
    return;
  }

  int code = http.GET();
  if (code != HTTP_CODE_OK)
  {
    Serial.printf("HTTP error: %d\n", code);
    logRemote(String("HTTP playback error: ") + code);
    http.end();
    return;
  }

  WiFiClient *stream = http.getStreamPtr();
  uint8_t header[44];
  if (stream->readBytes(header, sizeof(header)) != sizeof(header))
  {
    Serial.println("Failed to read WAV header");
   logRemote("Failed to read WAV header");
    http.end();
    return;
  }

  g_cancelPlayback = false;
  MicUploadPauseGuard micPauseGuard(g_pauseMicUpload);
  bool suppressSpeechLed = false;
  bool appliedSpeechLed = false;

  String urlStr(url);
  urlStr.toLowerCase();
  suppressSpeechLed = urlStr.endsWith("agent-greeting.wav") || urlStr.endsWith("alert_fall_detected.wav");

  if (!suppressSpeechLed)
  {
    sensorManagerTriggerSpeechSpeaking();
    appliedSpeechLed = true;
  }
  digitalWrite(SPK_SD, HIGH);

  uint8_t buffer[SPEAKER_DMA_LEN];
  size_t bytesWritten;

  while (stream->connected())
  {
    int len = stream->readBytes(buffer, sizeof(buffer));
    if (len <= 0)
    {
      logRemote("Playback stream ended");
      break;
    }
    if (g_cancelPlayback)
    {
      Serial.println("Playback cancelled");
      break;
    }
    i2s_write(I2S_SPK_PORT, buffer, len, &bytesWritten, portMAX_DELAY);
  }

  digitalWrite(SPK_SD, LOW);
  g_cancelPlayback = false;
  micPauseGuard.resume();
  if (appliedSpeechLed)
  {
    sensorManagerClearSpeechOverride();
  }
  if (!g_telemetryClient.notifyPlaybackFinished())
  {
    Serial.println("[AUDIO][WARN] Failed to notify playback completion");
  }
  http.end();
  Serial.println("Playback finished");
  logRemote("Playback finished");
}

static void logRemote(const char *message)
{
  if (!message)
  {
    return;
  }
  g_telemetryClient.postLog(message);
}

static void logRemote(const String &message)
{
  g_telemetryClient.postLog(message);
}
