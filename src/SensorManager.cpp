#include "SensorManager.h"

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Wire.h>

#ifdef I2C_BUFFER_LENGTH
#undef I2C_BUFFER_LENGTH
#define I2C_BUFFER_LENGTH 128
#endif

#include "MAX30105.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#include "spo2_algorithm.h"
#pragma GCC diagnostic pop

#include "TelemetryClient.h"
#include "TelemetryPayloadBuilder.h"

namespace
{
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;

// LED ring hardware configuration (FireBeetle's on-board NeoPixel ring).
constexpr uint8_t LED_RING_PIN = 4;
constexpr uint16_t LED_RING_COUNT = 16;

constexpr uint32_t SENSOR_LOOP_DELAY_MS = 20;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 2000;
constexpr uint32_t STATUS_PRINT_INTERVAL_MS = 5000;

constexpr float FALL_IMPACT_THRESHOLD_G = 2.0f;
constexpr float FALL_TILT_ENTER_DEG = 60.0f;
constexpr float FALL_TILT_EXIT_DEG = 45.0f;
constexpr float FALL_UPRIGHT_DEG = 25.0f;
constexpr unsigned long FALL_CALIB_DURATION_MS = 15000;
constexpr unsigned long FALL_TILT_HOLD_MS = 500;
constexpr unsigned long FALL_EXIT_STABLE_MS = 400;
constexpr unsigned long FALL_POST_WINDOW_MS = 2000;
constexpr unsigned long FALL_MIN_LOCK_MS = 2000;
constexpr unsigned long FALL_UPRIGHT_HOLD_MS = 1000;
constexpr unsigned long FALL_IMPACT_COOLDOWN_MS = 1500;

constexpr const char *FALL_ALERT_CLIP = "alert_fall_detected.wav";
constexpr const char *FALL_CLEAR_CLIP = "status_all_clear.wav";

// Temperature calibration (reuse definitions from prior sketches)
#ifndef TEMP_CALIB_SCALE
#define TEMP_CALIB_SCALE 1.000f
#endif
#ifndef TEMP_CALIB_OFFSET
#define TEMP_CALIB_OFFSET 3.80f
#endif
#ifndef TEMP_EMA_ALPHA
#define TEMP_EMA_ALPHA 0.30f
#endif

enum class FallState
{
  Calibrating,
  Monitoring,
  PostImpact,
  FallDetected
};

struct SensorTaskArgs
{
  TelemetryClient *client;
};

struct FallContext
{
  bool present = false;
  uint8_t mpuAddr = 0x68;
  FallState state = FallState::Calibrating;
  float base_gx = 0.0f;
  float base_gy = 0.0f;
  float base_gz = 0.0f;
  uint32_t baseSamples = 0;
  unsigned long startCalibMs = 0;
  unsigned long monitoringSinceMs = 0;
  unsigned long impactDetectedMs = 0;
  unsigned long tiltStartMs = 0;
  unsigned long exitStableStartMs = 0;
  unsigned long fellTimeMs = 0;
  unsigned long uprightStartMs = 0;
  float filteredX = 0.0f;
  float filteredY = 0.0f;
  float filteredZ = 1.0f;
  bool alertIssued = false;
};

struct FallReading
{
  bool available = false;
  float accelMag = NAN;
  float tiltDeg = NAN;
  FallState state = FallState::Calibrating;
  bool fallTriggered = false;
  bool fallRecovered = false;
};

enum class TemperatureSensorType
{
  Unknown,
  MAX30205,
  LM75,
  TMP102
};

struct TemperatureContext
{
  TemperatureSensorType type = TemperatureSensorType::Unknown;
  bool present = false;
  float emaValueC = NAN;
  unsigned long lastReadMs = 0;
};

struct TemperatureReading
{
  bool available = false;
  float valueC = NAN;
  String status;
};

struct VitalsContext
{
  bool present = false;
  uint32_t irBuffer[100] = {};
  uint32_t redBuffer[100] = {};
  uint8_t index = 0;
  bool ready = false;
  unsigned long lastCalcMs = 0;
  int32_t spo2 = -1;
  int8_t spo2Valid = 0;
  int32_t heartRate = -1;
  int8_t heartRateValid = 0;
};

struct VitalsReading
{
  bool available = false;
  int heartRate = 0;
  bool heartRateValid = false;
  int spo2 = 0;
  bool spo2Valid = false;
};

// Central LED ring instance used to convey sensor and fall-detection states.
Adafruit_NeoPixel g_ledRing(LED_RING_COUNT, LED_RING_PIN, NEO_GRB + NEO_KHZ800);
MAX30105 g_maxSensor;
TelemetryClient *g_client = nullptr;

enum class SpeechLedMode
{
  Idle,
  Listening,
  Speaking
};

portMUX_TYPE g_speechLedMux = portMUX_INITIALIZER_UNLOCKED;
SpeechLedMode g_speechLedMode = SpeechLedMode::Idle;

// Forward declarations
bool scanMpuAddress(FallContext &ctx);
bool writeMpuReg(uint8_t addr, uint8_t reg, uint8_t value);
bool readAccelG(const FallContext &ctx, float &ax_g, float &ay_g, float &az_g);
float angleDeg(float ax, float ay, float az, float bx, float by, float bz);
void normalize(float &x, float &y, float &z);

TemperatureSensorType detectTempSensor();
float readTemperatureC(TemperatureSensorType type);

const char *fallStateToString(FallState state);
void renderLed(FallState state, unsigned long nowMs, float tiltDeg, bool fallAlertActive);
void renderSpeechListeningEffect(unsigned long nowMs);
void renderSpeechSpeakingEffect(unsigned long nowMs);

void setSpeechLedMode(SpeechLedMode mode);
SpeechLedMode currentSpeechLedMode();

FallReading updateFallDetector(FallContext &ctx, unsigned long nowMs);
TemperatureReading updateTemperature(TemperatureContext &ctx, unsigned long nowMs);
VitalsReading updateVitals(VitalsContext &ctx, unsigned long nowMs);

void sensorTask(void *param);

void logMessage(TelemetryClient *client, const String &message)
{
  if (!client || !client->isReady())
  {
    return;
  }
  client->postLog(message);
}

} // namespace

void sensorManagerTriggerSpeechListening()
{
  setSpeechLedMode(SpeechLedMode::Listening);
}

void sensorManagerTriggerSpeechSpeaking()
{
  setSpeechLedMode(SpeechLedMode::Speaking);
}

void sensorManagerClearSpeechOverride()
{
  setSpeechLedMode(SpeechLedMode::Idle);
}

void startSensorManagerTask(TelemetryClient *client)
{
  static SensorTaskArgs args{};
  args.client = client;
  g_client = client;
  xTaskCreatePinnedToCore(sensorTask, "sensorTask", 8192, &args, 1, nullptr, 1);
}


namespace
{

bool scanMpuAddress(FallContext &ctx)
{
  for (uint8_t candidate : {0x68, 0x69})
  {
    Wire.beginTransmission(candidate);
    if (Wire.endTransmission() == 0)
    {
      ctx.mpuAddr = candidate;
      return true;
    }
  }
  return false;
}

bool writeMpuReg(uint8_t addr, uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readAccelG(const FallContext &ctx, float &ax_g, float &ay_g, float &az_g)
{
  Wire.beginTransmission(ctx.mpuAddr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }
  if (Wire.requestFrom(static_cast<int>(ctx.mpuAddr), 6) < 6)
  {
    return false;
  }

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();

  ax_g = static_cast<float>(ax) / 16384.0f;
  ay_g = static_cast<float>(ay) / 16384.0f;
  az_g = static_cast<float>(az) / 16384.0f;
  return true;
}

void normalize(float &x, float &y, float &z)
{
  float mag = sqrtf(x * x + y * y + z * z);
  if (mag > 1e-6f)
  {
    x /= mag;
    y /= mag;
    z /= mag;
  }
}

float angleDeg(float ax, float ay, float az, float bx, float by, float bz)
{
  float dot = ax * bx + ay * by + az * bz;
  dot = fmaxf(-1.0f, fminf(1.0f, dot));
  return acosf(dot) * 180.0f / PI;
}

TemperatureSensorType detectTempSensor()
{
  constexpr uint8_t TEMP_I2C_ADDR = 0x48;
  Wire.beginTransmission(TEMP_I2C_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0)
  {
    return TemperatureSensorType::Unknown;
  }
  if (Wire.requestFrom(TEMP_I2C_ADDR, static_cast<uint8_t>(2)) < 2)
  {
    return TemperatureSensorType::Unknown;
  }
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  uint16_t raw = (static_cast<uint16_t>(msb) << 8) | lsb;
  if ((raw & 0x00FF) == 0)
  {
    return TemperatureSensorType::LM75;
  }
  if ((raw & 0x000F) == 0)
  {
    return TemperatureSensorType::TMP102;
  }
  return TemperatureSensorType::MAX30205;
}

float decodeTemperature(TemperatureSensorType type, uint16_t raw)
{
  switch (type)
  {
  case TemperatureSensorType::MAX30205:
    return static_cast<int16_t>(raw) / 256.0f;
  case TemperatureSensorType::LM75:
  {
    int16_t val = static_cast<int16_t>(raw);
    val >>= 7;
    return val * 0.5f;
  }
  case TemperatureSensorType::TMP102:
  {
    int16_t val = static_cast<int16_t>(raw);
    val >>= 4;
    return val * 0.0625f;
  }
  default:
    return NAN;
  }
}

float readTemperatureC(TemperatureSensorType type)
{
  if (type == TemperatureSensorType::Unknown)
  {
    return NAN;
  }
  constexpr uint8_t TEMP_I2C_ADDR = 0x48;
  Wire.beginTransmission(TEMP_I2C_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0)
  {
    return NAN;
  }
  if (Wire.requestFrom(TEMP_I2C_ADDR, static_cast<uint8_t>(2)) < 2)
  {
    return NAN;
  }
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  uint16_t raw = (static_cast<uint16_t>(msb) << 8) | lsb;
  return decodeTemperature(type, raw);
}

const char *fallStateToString(FallState state)
{
  switch (state)
  {
  case FallState::Calibrating:
    return "CALIBRATING";
  case FallState::Monitoring:
    return "MONITORING";
  case FallState::PostImpact:
    return "POST_IMPACT";
  case FallState::FallDetected:
    return "FALL_DETECTED";
  default:
    return "UNKNOWN";
  }
}

// Shared LED helpers so speech-aware components can render consistent animations.
void renderSpeechListeningEffect(unsigned long nowMs)
{
  // Dim white ring with a single bright pixel rotating clockwise.
  uint32_t baseColor = g_ledRing.Color(8, 8, 8);
  g_ledRing.fill(baseColor, 0, LED_RING_COUNT);
  int chaseIndex = (nowMs / 120) % LED_RING_COUNT;
  g_ledRing.setPixelColor(chaseIndex, g_ledRing.Color(180, 180, 180));
}

void renderSpeechSpeakingEffect(unsigned long nowMs)
{
  // Soft white breathing effect during active speech playback/recording.
  float phase = (nowMs % 3000) / 3000.0f;
  float level = 0.15f + 0.75f * (0.5f + 0.5f * sinf(phase * 2.0f * PI));
  uint8_t intensity = static_cast<uint8_t>(fminf(level, 1.0f) * 255.0f);
  uint32_t color = g_ledRing.Color(intensity, intensity, intensity);
  g_ledRing.fill(color, 0, LED_RING_COUNT);
}

void setSpeechLedMode(SpeechLedMode mode)
{
  portENTER_CRITICAL(&g_speechLedMux);
  g_speechLedMode = mode;
  portEXIT_CRITICAL(&g_speechLedMux);
}

SpeechLedMode currentSpeechLedMode()
{
  portENTER_CRITICAL(&g_speechLedMux);
  SpeechLedMode mode = g_speechLedMode;
  portEXIT_CRITICAL(&g_speechLedMux);
  return mode;
}

// Render the LED ring according to the current fall-detection state.
// Each state maps to a distinctive animation so operators can gauge status at a glance.
void renderLed(FallState state, unsigned long nowMs, float tiltDeg, bool fallAlertActive)
{
  SpeechLedMode speechOverride = currentSpeechLedMode();
  if (speechOverride == SpeechLedMode::Listening)
  {
    renderSpeechListeningEffect(nowMs);
    g_ledRing.show();
    return;
  }
  if (speechOverride == SpeechLedMode::Speaking)
  {
    renderSpeechSpeakingEffect(nowMs);
    g_ledRing.show();
    return;
  }

  switch (state)
  {
  case FallState::Calibrating:
  {
    // Breathing blue pulse while the IMU baseline is being established.
    float phase = (nowMs % 6000) / 6000.0f;
    float level = 0.15f + 0.65f * (0.5f + 0.5f * sinf(phase * 2.0f * PI));
    uint8_t b = static_cast<uint8_t>(fminf(level, 1.0f) * 255.0f);
    uint32_t color = g_ledRing.Color(0, 0, b);
    g_ledRing.fill(color, 0, LED_RING_COUNT);
    break;
  }
  case FallState::Monitoring:
  {
    // White chase indicates the system is passively listening for speech input.
    renderSpeechListeningEffect(nowMs);
    break;
  }
  case FallState::PostImpact:
  {
    // Amber blinking warns the system is in the post-impact cool-down window.
    bool flash = ((nowMs / 200) % 2) == 0;
    uint32_t color = flash ? g_ledRing.Color(255, 200, 0) : g_ledRing.Color(10, 10, 0);
    g_ledRing.fill(color, 0, LED_RING_COUNT);
    break;
  }
  case FallState::FallDetected:
  {
    // Bold red strobe draws immediate attention to an active fall alert.
    bool flash = ((nowMs / 120) % 2) == 0;
    uint32_t color = flash ? g_ledRing.Color(255, 0, 0) : g_ledRing.Color(0, 0, 0);
    g_ledRing.fill(color, 0, LED_RING_COUNT);
    break;
  }
  }
  g_ledRing.show();
}

FallReading updateFallDetector(FallContext &ctx, unsigned long nowMs)
{
  FallReading reading{};
  if (!ctx.present)
  {
    return reading;
  }

  float ax, ay, az;
  if (!readAccelG(ctx, ax, ay, az))
  {
    return reading;
  }

  float gx = ax;
  float gy = ay;
  float gz = az;
  normalize(gx, gy, gz);

  constexpr float alpha = 0.2f;
  ctx.filteredX = (1.0f - alpha) * ctx.filteredX + alpha * gx;
  ctx.filteredY = (1.0f - alpha) * ctx.filteredY + alpha * gy;
  ctx.filteredZ = (1.0f - alpha) * ctx.filteredZ + alpha * gz;

  float accelMag = sqrtf(ax * ax + ay * ay + az * az);
  float tiltDeg = angleDeg(ctx.filteredX, ctx.filteredY, ctx.filteredZ, ctx.base_gx, ctx.base_gy, ctx.base_gz);

  reading.available = true;
  reading.accelMag = accelMag;
  reading.tiltDeg = tiltDeg;
  reading.state = ctx.state;

  switch (ctx.state)
  {
  case FallState::Calibrating:
    if (nowMs - ctx.startCalibMs <= FALL_CALIB_DURATION_MS)
    {
      ctx.base_gx = (ctx.base_gx * ctx.baseSamples + gx) / (ctx.baseSamples + 1);
      ctx.base_gy = (ctx.base_gy * ctx.baseSamples + gy) / (ctx.baseSamples + 1);
      ctx.base_gz = (ctx.base_gz * ctx.baseSamples + gz) / (ctx.baseSamples + 1);
      ctx.baseSamples++;
    }
    else
    {
      normalize(ctx.base_gx, ctx.base_gy, ctx.base_gz);
      ctx.state = FallState::Monitoring;
      ctx.monitoringSinceMs = nowMs;
    }
    break;

  case FallState::Monitoring:
    if (nowMs - ctx.monitoringSinceMs >= FALL_IMPACT_COOLDOWN_MS && accelMag >= FALL_IMPACT_THRESHOLD_G)
    {
      ctx.impactDetectedMs = nowMs;
      ctx.tiltStartMs = 0;
      ctx.exitStableStartMs = 0;
      ctx.state = FallState::PostImpact;
      Serial.printf("[FALL] Impact detected |a|=%.2fg\n", accelMag);
    }
    break;

  case FallState::PostImpact:
  {
    if (tiltDeg >= FALL_TILT_ENTER_DEG)
    {
      if (ctx.tiltStartMs == 0)
      {
        ctx.tiltStartMs = nowMs;
      }
      if (nowMs - ctx.tiltStartMs >= FALL_TILT_HOLD_MS)
      {
        ctx.state = FallState::FallDetected;
        ctx.fellTimeMs = nowMs;
        ctx.uprightStartMs = 0;
        reading.fallTriggered = !ctx.alertIssued;
        ctx.alertIssued = true;
        Serial.printf("[FALL] Tilt %.1f deg -> FALL DETECTED\n", tiltDeg);
      }
    }
    else
    {
      ctx.tiltStartMs = 0;
    }

    if (tiltDeg < FALL_TILT_EXIT_DEG)
    {
      if (ctx.exitStableStartMs == 0)
      {
        ctx.exitStableStartMs = nowMs;
      }
      if (nowMs - ctx.exitStableStartMs > FALL_EXIT_STABLE_MS)
      {
        ctx.state = FallState::Monitoring;
        ctx.monitoringSinceMs = nowMs;
        ctx.tiltStartMs = 0;
        ctx.exitStableStartMs = 0;
      }
    }
    else
    {
      ctx.exitStableStartMs = 0;
    }

    if (nowMs - ctx.impactDetectedMs > FALL_POST_WINDOW_MS && ctx.state != FallState::FallDetected)
    {
      ctx.state = FallState::Monitoring;
      ctx.monitoringSinceMs = nowMs;
      ctx.tiltStartMs = 0;
    }

    break;
  }

  case FallState::FallDetected:
  {
    bool minLockSatisfied = (nowMs - ctx.fellTimeMs) > FALL_MIN_LOCK_MS;
    if (tiltDeg < FALL_UPRIGHT_DEG)
    {
      if (ctx.uprightStartMs == 0)
      {
        ctx.uprightStartMs = nowMs;
      }
      if (minLockSatisfied && (nowMs - ctx.uprightStartMs) > FALL_UPRIGHT_HOLD_MS)
      {
        ctx.state = FallState::Monitoring;
        ctx.monitoringSinceMs = nowMs;
        ctx.fellTimeMs = 0;
        ctx.uprightStartMs = 0;
        ctx.alertIssued = false;
        reading.fallRecovered = true;
        Serial.println("[FALL] Upright posture restored -> monitoring");
      }
    }
    else
    {
      ctx.uprightStartMs = 0;
    }
    break;
  }
  }

  reading.state = ctx.state;
  return reading;
}

TemperatureReading updateTemperature(TemperatureContext &ctx, unsigned long nowMs)
{
  TemperatureReading reading{};
  if (!ctx.present)
  {
    return reading;
  }
  if (nowMs - ctx.lastReadMs < 1000)
  {
    return reading;
  }

  ctx.lastReadMs = nowMs;
  float raw = readTemperatureC(ctx.type);
  if (isnan(raw))
  {
    return reading;
  }

  float calibrated = TEMP_CALIB_SCALE * raw + TEMP_CALIB_OFFSET;
  if (isnan(ctx.emaValueC))
  {
    ctx.emaValueC = calibrated;
  }
  else
  {
    ctx.emaValueC = TEMP_EMA_ALPHA * calibrated + (1.0f - TEMP_EMA_ALPHA) * ctx.emaValueC;
  }

  reading.available = true;
  reading.valueC = ctx.emaValueC;
  if (ctx.emaValueC >= 38.0f)
  {
    reading.status = "high";
  }
  else if (ctx.emaValueC <= 35.0f)
  {
    reading.status = "low";
  }
  else
  {
    reading.status = "normal";
  }
  return reading;
}

VitalsReading updateVitals(VitalsContext &ctx, unsigned long nowMs)
{
  VitalsReading reading{};
  if (!ctx.present)
  {
    return reading;
  }

  g_maxSensor.check();
  while (g_maxSensor.available())
  {
    uint32_t red = g_maxSensor.getFIFORed();
    uint32_t ir = g_maxSensor.getFIFOIR();
    g_maxSensor.nextSample();

    if (ctx.index < 100)
    {
      ctx.redBuffer[ctx.index] = red;
      ctx.irBuffer[ctx.index] = ir;
      ctx.index++;
      if (ctx.index == 100)
      {
        ctx.ready = true;
      }
    }
    else
    {
      for (uint8_t i = 25; i < 100; ++i)
      {
        ctx.redBuffer[i - 25] = ctx.redBuffer[i];
        ctx.irBuffer[i - 25] = ctx.irBuffer[i];
      }
      ctx.redBuffer[75] = red;
      ctx.irBuffer[75] = ir;
      ctx.index = 76;
    }
  }

  if (ctx.ready && (nowMs - ctx.lastCalcMs >= 250))
  {
    ctx.lastCalcMs = nowMs;
    maxim_heart_rate_and_oxygen_saturation(
        ctx.irBuffer, 100, ctx.redBuffer,
        &ctx.spo2, &ctx.spo2Valid, &ctx.heartRate, &ctx.heartRateValid);
  }

  if (ctx.spo2Valid || ctx.heartRateValid)
  {
    reading.available = true;
    reading.heartRate = ctx.heartRate;
    reading.heartRateValid = ctx.heartRateValid;
    reading.spo2 = ctx.spo2;
    reading.spo2Valid = ctx.spo2Valid;
  }
  return reading;
}

void sensorTask(void *param)
{
  auto *args = static_cast<SensorTaskArgs *>(param);
  TelemetryClient *client = args ? args->client : nullptr;

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000); // fast mode if hardware supports it

  g_ledRing.begin();        // Prepare the NeoPixel driver before any animations run.
  g_ledRing.setBrightness(80);
  g_ledRing.show();         // Clear residual pixels at boot.

  FallContext fall{};
  TemperatureContext temp{};
  VitalsContext vitals{};

  fall.present = scanMpuAddress(fall);
  if (fall.present)
  {
    writeMpuReg(fall.mpuAddr, 0x6B, 0x01); // wake up
    writeMpuReg(fall.mpuAddr, 0x1C, 0x00); // ±2g
    writeMpuReg(fall.mpuAddr, 0x1B, 0x08); // ±500 dps
    writeMpuReg(fall.mpuAddr, 0x1A, 0x03); // DLPF
    writeMpuReg(fall.mpuAddr, 0x19, 9);    // sample rate divider
    fall.startCalibMs = millis();
    fall.filteredX = 0.0f;
    fall.filteredY = 0.0f;
    fall.filteredZ = 1.0f;
    Serial.print("[FALL] MPU6050 detected at 0x");
    Serial.println(fall.mpuAddr, HEX);
    logMessage(client, "[FALL] MPU6050 detected");
  }
  else
  {
    Serial.println("[FALL][WARN] MPU6050 not found.");
    logMessage(client, "[FALL][WARN] MPU6050 not found");
  }

  temp.type = detectTempSensor();
  temp.present = (temp.type != TemperatureSensorType::Unknown);
  if (temp.present)
  {
    switch (temp.type)
    {
    case TemperatureSensorType::MAX30205:
      Serial.println("[TEMP] Detected MAX30205");
      logMessage(client, "[TEMP] Detected MAX30205");
      break;
    case TemperatureSensorType::LM75:
      Serial.println("[TEMP] Detected LM75");
      logMessage(client, "[TEMP] Detected LM75");
      break;
    case TemperatureSensorType::TMP102:
      Serial.println("[TEMP] Detected TMP102");
      logMessage(client, "[TEMP] Detected TMP102");
      break;
    default:
      break;
    }
  }
  else
  {
    Serial.println("[TEMP][WARN] Sensor not found at 0x48");
    logMessage(client, "[TEMP][WARN] Sensor not found");
  }

  vitals.present = g_maxSensor.begin(Wire, I2C_SPEED_FAST);
  if (vitals.present)
  {
    byte ledBrightness = 0x7F;
    byte sampleAverage = 4;
    byte ledMode = 2;
    byte sampleRate = 100;
    int pulseWidth = 411;
    int adcRange = 16384;
    g_maxSensor.setup(ledBrightness, sampleAverage, ledMode,
                      sampleRate, pulseWidth, adcRange);
    g_maxSensor.setPulseAmplitudeGreen(0);
    Serial.println("[HR] MAX30102 ready");
    logMessage(client, "[HR] MAX30102 ready");
  }
  else
  {
    Serial.println("[HR][WARN] MAX30102 not responding at 0x57");
    logMessage(client, "[HR][WARN] MAX30102 not responding");
  }

  unsigned long lastTelemetryMs = 0;
  unsigned long lastStatusMs = 0;

  for (;;)
  {
    unsigned long nowMs = millis();
    FallReading fallReading = updateFallDetector(fall, nowMs);
    TemperatureReading tempReading = updateTemperature(temp, nowMs);
    VitalsReading vitalsReading = updateVitals(vitals, nowMs);

    renderLed(fall.state, nowMs, fallReading.tiltDeg, fall.alertIssued);

    if (fallReading.fallTriggered && client)
    {
      const String message = "[AUDIO] Fall detected";
      Serial.println(message);
      logMessage(client, message);
      if (!client->queueAudioClip(FALL_ALERT_CLIP))
      {
        Serial.println("[AUDIO] Failed to queue fall alert clip");
        logMessage(client, "[AUDIO][ERR] Failed to queue fall alert clip");
      }
    }
    if (fallReading.fallRecovered && client)
    {
      const String message = "[AUDIO] Fall recovery detected";
      Serial.println(message);
      logMessage(client, message);
      if (!client->queueAudioClip(FALL_CLEAR_CLIP))
      {
        Serial.println("[AUDIO] Failed to queue all-clear clip");
        logMessage(client, "[AUDIO][ERR] Failed to queue all-clear clip");
      }
    }

    if (nowMs - lastTelemetryMs >= TELEMETRY_INTERVAL_MS)
    {
      lastTelemetryMs = nowMs;
      TelemetryPayloadBuilder builder;
      builder.reset();

      if (fallReading.available)
      {
        float tiltValue = isnan(fallReading.tiltDeg) ? 0.0f : fallReading.tiltDeg;
        float impactValue = isnan(fallReading.accelMag) ? 0.0f : fallReading.accelMag;
        builder.setFall(fallStateToString(fallReading.state), tiltValue, impactValue);
        if (fallReading.fallTriggered)
        {
          String detail = "tilt=" + String(tiltValue, 1) + "deg";
          builder.addAlert("FALL_DETECTED", detail);
        }
      }

      if (vitalsReading.available)
      {
        builder.setVitals(
            vitalsReading.heartRate,
            vitalsReading.heartRateValid,
            vitalsReading.spo2,
            vitalsReading.spo2Valid);
        if (vitalsReading.spo2Valid && vitalsReading.spo2 < 92)
        {
          builder.addAlert("SPO2_LOW", "SpO2=" + String(vitalsReading.spo2));
        }
      }

      if (tempReading.available)
      {
        builder.setTemperature(tempReading.valueC, tempReading.status);
        if (tempReading.status == "high")
        {
          builder.addAlert("TEMPERATURE_HIGH", "temp=" + String(tempReading.valueC, 2) + "C");
        }
      }

      if (client && client->isReady())
      {
        String payload = builder.build(nowMs);
        if (!client->postTelemetry(payload))
        {
          Serial.println("[TEL] Failed to post telemetry");
        }
      }
    }

    if (nowMs - lastStatusMs >= STATUS_PRINT_INTERVAL_MS)
    {
      lastStatusMs = nowMs;
      Serial.print("[DATA] fall=");
      Serial.print(fallStateToString(fall.state));
      Serial.print(" tilt=");
      if (fallReading.available && !isnan(fallReading.tiltDeg))
      {
        Serial.print(fallReading.tiltDeg, 1);
        Serial.print("deg");
      }
      else
      {
        Serial.print("--");
      }
      Serial.print(" | HR=");
      if (vitalsReading.available && vitalsReading.heartRateValid)
      {
        Serial.print(vitalsReading.heartRate);
      }
      else
      {
        Serial.print("--");
      }
      Serial.print(" bpm SpO2=");
      if (vitalsReading.available && vitalsReading.spo2Valid)
      {
        Serial.print(vitalsReading.spo2);
        Serial.print("%");
      }
      else
      {
        Serial.print("--");
      }
      Serial.print(" | Temp=");
      if (tempReading.available && !isnan(tempReading.valueC))
      {
        Serial.print(tempReading.valueC, 2);
        Serial.print("C");
      }
      else
      {
        Serial.print("--");
      }
      Serial.println();
    }

    vTaskDelay(pdMS_TO_TICKS(SENSOR_LOOP_DELAY_MS));
  }
}

} // namespace
