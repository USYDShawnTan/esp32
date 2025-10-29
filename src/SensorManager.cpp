#include "SensorManager.h"

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Wire.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace
{
  constexpr uint8_t I2C_SDA_PIN = 21;
  constexpr uint8_t I2C_SCL_PIN = 22;

  constexpr uint8_t LED_RING_PIN = 4;
  constexpr uint16_t LED_RING_COUNT = 16;
  constexpr uint32_t LED_BRIGHTNESS = 96;

  // Motion processing parameters
  constexpr uint32_t MOTION_LOOP_DELAY_MS = 20;
  constexpr uint32_t BASELINE_SAMPLE_COUNT = 200;
  constexpr uint32_t BASELINE_SETTLE_MS = 100;

  constexpr bool ENABLE_SENSOR_DEBUG = true;
  constexpr uint32_t SENSOR_DEBUG_INTERVAL_MS = 250;

  constexpr float MOTION_IDLE_THRESHOLD = 0.05f;
  constexpr float MOTION_ACTIVE_THRESHOLD = 0.18f;
  constexpr float MOTION_WAVE_THRESHOLD = 0.42f;
  constexpr float MOTION_OVERLOAD_THRESHOLD = 0.75f;

  constexpr uint32_t ENERGY_DECAY_MS = 600;
  constexpr uint32_t OVERLOAD_DECAY_MS = 900;

  constexpr float ANGLE_PICKUP_ENTER_DEG = 10.0f;
  constexpr float ANGLE_PICKUP_EXIT_DEG = 4.0f;
  constexpr float ANGLE_PICKUP_CONFIRM_DEG = 14.0f;
  constexpr uint32_t PICKUP_MIN_DURATION_MS = 700;
  constexpr uint32_t PUTDOWN_MIN_DURATION_MS = 900;
  constexpr float PICKUP_MOTION_THRESHOLD = 0.16f;
  constexpr uint32_t PICKUP_MOTION_MIN_MS = 350;
  constexpr float PICKUP_ALONG_THRESHOLD = 0.14f;
  constexpr uint32_t PICKUP_ALONG_MIN_MS = 350;
  constexpr float PUTDOWN_ALONG_EXIT = 0.08f;

  enum class SystemState
  {
    Initializing,
    Idle,
    Ready,
    EnergyWave,
    Overload,
    ModeCycling
  };

  enum class SpeechLedMode
  {
    None,
    Listening,
    Speaking
  };

  struct MotionContext
  {
    bool present = false;
    uint8_t mpuAddr = 0x68;

    bool baselineCalibrated = false;
    float baseX = 0.0f;
    float baseY = 0.0f;
    float baseZ = 1.0f;
    float baseSumX = 0.0f;
    float baseSumY = 0.0f;
    float baseSumZ = 0.0f;
    uint32_t baseSamples = 0;
    unsigned long baselineStartMs = 0;

    float filteredMotion = 0.0f;
    float filteredAngle = 0.0f;

    bool energyActive = false;
    bool overloadActive = false;
    unsigned long lastEnergyMs = 0;
    unsigned long lastOverloadMs = 0;

    SystemState state = SystemState::Initializing;

    bool inHand = false;
    bool putdownCandidate = false;
    unsigned long putdownCandidateStartMs = 0;
    float netAlongBaseline = 0.0f;
    float filteredLateral = 0.0f;
    bool pickupCandidate = false;
    unsigned long pickupCandidateStartMs = 0;
    unsigned long pickupMotionStartMs = 0;
    unsigned long pickupAlongStartMs = 0;
    bool pickupMotionSatisfied = false;
    bool pickupAlongSatisfied = false;
    unsigned long lastDebugPrintMs = 0;
  };

  Adafruit_NeoPixel g_ledRing(LED_RING_COUNT, LED_RING_PIN, NEO_GRB + NEO_KHZ800);

  portMUX_TYPE g_speechMux = portMUX_INITIALIZER_UNLOCKED;
  SpeechLedMode g_speechMode = SpeechLedMode::None;
  bool g_modeCycleRequest = false;

  // Forward declarations
  bool scanMpuAddress(MotionContext &ctx);
  bool writeMpuReg(uint8_t addr, uint8_t reg, uint8_t value);
  bool readAccelG(const MotionContext &ctx, float &ax, float &ay, float &az);
  void normalize(float &x, float &y, float &z);
  float angleBetween(float ax, float ay, float az, float bx, float by, float bz);

  void updateMotion(MotionContext &ctx, unsigned long nowMs);
  void updateState(MotionContext &ctx, unsigned long nowMs);

  void renderState(const MotionContext &ctx, unsigned long nowMs);
  void renderInitializing(unsigned long nowMs);
  void renderIdle(unsigned long nowMs);
  void renderReady(unsigned long nowMs);
  void renderEnergyWave(unsigned long nowMs);
  void renderOverload(unsigned long nowMs);
  void renderModeCycling(unsigned long nowMs);
  void renderSpeechListening(unsigned long nowMs);
  void renderSpeechSpeaking(unsigned long nowMs);

  void sensorTask(void *param);
  SpeechLedMode currentSpeechLedMode();
  void setSpeechLedMode(SpeechLedMode mode);
} // namespace

void startSensorManagerTask(TelemetryClient *client)
{
  (void)client;
  xTaskCreatePinnedToCore(sensorTask, "sensorTask", 4096, nullptr, 1, nullptr, 1);
}

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
  setSpeechLedMode(SpeechLedMode::None);
}

void sensorManagerUnlockMonitoring()
{
  // No-op in the new state model; kept for backwards compatibility.
}

namespace
{
  bool scanMpuAddress(MotionContext &ctx)
  {
    for (uint8_t addr : {uint8_t(0x68), uint8_t(0x69)})
    {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0)
      {
        ctx.mpuAddr = addr;
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

  bool readAccelG(const MotionContext &ctx, float &ax, float &ay, float &az)
  {
    Wire.beginTransmission(ctx.mpuAddr);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0)
    {
      return false;
    }
    constexpr size_t bytes = 6;
    if (Wire.requestFrom(ctx.mpuAddr, static_cast<uint8_t>(bytes)) != bytes)
    {
      return false;
    }
    int16_t rawAx = (Wire.read() << 8) | Wire.read();
    int16_t rawAy = (Wire.read() << 8) | Wire.read();
    int16_t rawAz = (Wire.read() << 8) | Wire.read();

    constexpr float scale = 16384.0f; // +/-2g
    ax = static_cast<float>(rawAx) / scale;
    ay = static_cast<float>(rawAy) / scale;
    az = static_cast<float>(rawAz) / scale;
    return true;
  }

  void normalize(float &x, float &y, float &z)
  {
    const float mag = sqrtf(x * x + y * y + z * z);
    if (mag < 1e-6f)
    {
      x = y = 0.0f;
      z = 1.0f;
      return;
    }
    x /= mag;
    y /= mag;
    z /= mag;
  }

  float angleBetween(float ax, float ay, float az, float bx, float by, float bz)
  {
    const float dot = ax * bx + ay * by + az * bz;
    const float magA = sqrtf(ax * ax + ay * ay + az * az);
    const float magB = sqrtf(bx * bx + by * by + bz * bz);
    if (magA < 1e-6f || magB < 1e-6f)
    {
      return 0.0f;
    }
    float cosTheta = dot / (magA * magB);
    cosTheta = fminf(fmaxf(cosTheta, -1.0f), 1.0f);
    return acosf(cosTheta) * 180.0f / static_cast<float>(M_PI);
  }

  void updateMotion(MotionContext &ctx, unsigned long nowMs)
  {
    float ax, ay, az;
    if (!ctx.present || !readAccelG(ctx, ax, ay, az))
    {
      ctx.filteredMotion *= 0.95f;
      ctx.filteredAngle *= 0.95f;
      return;
    }

    if (!ctx.baselineCalibrated)
    {
      if (ctx.baseSamples == 0)
      {
        ctx.baselineStartMs = nowMs;
        ctx.baseSumX = ctx.baseSumY = ctx.baseSumZ = 0.0f;
      }

      ctx.baseSumX += ax;
      ctx.baseSumY += ay;
      ctx.baseSumZ += az;
      ++ctx.baseSamples;

      if (nowMs - ctx.baselineStartMs >= BASELINE_SETTLE_MS && ctx.baseSamples >= BASELINE_SAMPLE_COUNT)
      {
        ctx.baseX = ctx.baseSumX / static_cast<float>(ctx.baseSamples);
        ctx.baseY = ctx.baseSumY / static_cast<float>(ctx.baseSamples);
        ctx.baseZ = ctx.baseSumZ / static_cast<float>(ctx.baseSamples);
        normalize(ctx.baseX, ctx.baseY, ctx.baseZ);
        ctx.baselineCalibrated = true;
        Serial.print("[SENS] Baseline calibrated. Vector=");
        Serial.print(ctx.baseX, 3);
        Serial.print(",");
        Serial.print(ctx.baseY, 3);
        Serial.print(",");
        Serial.println(ctx.baseZ, 3);
      }
      return;
    }

    const float accelMag = sqrtf(ax * ax + ay * ay + az * az);
    const float motion = fabsf(accelMag - 1.0f);
    ctx.filteredMotion = ctx.filteredMotion * 0.90f + motion * 0.10f;

    const float angle = angleBetween(ax, ay, az, ctx.baseX, ctx.baseY, ctx.baseZ);
    ctx.filteredAngle = ctx.filteredAngle * 0.85f + angle * 0.15f;

    const float projection = (ax * ctx.baseX + ay * ctx.baseY + az * ctx.baseZ);
    const float along = projection - 1.0f;
    ctx.netAlongBaseline = along;

    float lateralSq = accelMag * accelMag - projection * projection;
    if (lateralSq < 0.0f)
    {
      lateralSq = 0.0f;
    }
    const float lateral = sqrtf(lateralSq);
    ctx.filteredLateral = ctx.filteredLateral * 0.80f + lateral * 0.20f;

    const bool motionIdle = ctx.filteredMotion < MOTION_IDLE_THRESHOLD;
    const bool angleActive = ctx.filteredAngle > ANGLE_PICKUP_ENTER_DEG;
    const bool angleConfirm = ctx.filteredAngle > ANGLE_PICKUP_CONFIRM_DEG;
    const bool angleIdle = ctx.filteredAngle < ANGLE_PICKUP_EXIT_DEG;
    const bool motionHigh = ctx.filteredMotion > PICKUP_MOTION_THRESHOLD;
    const bool motionCalm = ctx.filteredMotion < (PICKUP_MOTION_THRESHOLD * 0.5f);
    const float absAlong = fabsf(along);
    const bool alongHigh = absAlong > PICKUP_ALONG_THRESHOLD;
    const bool alongCalm = absAlong < PUTDOWN_ALONG_EXIT;
    const bool pickupSignal = angleActive;
    const bool pickupConfirmed = angleConfirm;
    const bool pickupReset = !angleActive && motionCalm && alongCalm;
    const bool putdownSignal = angleIdle && motionIdle && alongCalm;

    if (!ctx.inHand)
    {
      if (pickupSignal)
      {
        if (!ctx.pickupCandidate)
        {
          ctx.pickupCandidate = true;
          ctx.pickupCandidateStartMs = nowMs;
          ctx.pickupMotionStartMs = motionHigh ? nowMs : 0;
          ctx.pickupAlongStartMs = alongHigh ? nowMs : 0;
          ctx.pickupMotionSatisfied = false;
          ctx.pickupAlongSatisfied = false;
        }
        else
        {
          if (motionHigh)
          {
            if (ctx.pickupMotionStartMs == 0)
            {
              ctx.pickupMotionStartMs = nowMs;
            }
            if (!ctx.pickupMotionSatisfied &&
                (nowMs - ctx.pickupMotionStartMs) >= PICKUP_MOTION_MIN_MS)
            {
              ctx.pickupMotionSatisfied = true;
              ctx.pickupMotionStartMs = 0;
            }
          }
          else
          {
            if (!ctx.pickupMotionSatisfied)
            {
              ctx.pickupMotionStartMs = 0;
            }
          }

          if (alongHigh)
          {
            if (ctx.pickupAlongStartMs == 0)
            {
              ctx.pickupAlongStartMs = nowMs;
            }
            if (!ctx.pickupAlongSatisfied &&
                (nowMs - ctx.pickupAlongStartMs) >= PICKUP_ALONG_MIN_MS)
            {
              ctx.pickupAlongSatisfied = true;
              ctx.pickupAlongStartMs = 0;
            }
          }
          else
          {
            if (!ctx.pickupAlongSatisfied)
            {
              ctx.pickupAlongStartMs = 0;
            }
          }

          const bool motionHoldSatisfied = ctx.pickupMotionSatisfied;
          const bool alongHoldSatisfied = ctx.pickupAlongSatisfied;
          const bool durationSatisfied = (nowMs - ctx.pickupCandidateStartMs) >= PICKUP_MIN_DURATION_MS;
          const bool accelerationSatisfied = motionHoldSatisfied || alongHoldSatisfied;

          if (pickupConfirmed && durationSatisfied && accelerationSatisfied)
          {
            ctx.inHand = true;
            ctx.pickupCandidate = false;
            ctx.pickupCandidateStartMs = 0;
            ctx.pickupMotionStartMs = 0;
            ctx.pickupAlongStartMs = 0;
            ctx.pickupMotionSatisfied = false;
            ctx.pickupAlongSatisfied = false;
            ctx.putdownCandidate = false;
            ctx.putdownCandidateStartMs = 0;
            Serial.println("[SENS] Pickup confirmed -> Ready");
          }
        }
      }
      else if (pickupReset)
      {
        ctx.pickupCandidate = false;
        ctx.pickupCandidateStartMs = 0;
        ctx.pickupMotionStartMs = 0;
        ctx.pickupAlongStartMs = 0;
        ctx.pickupMotionSatisfied = false;
        ctx.pickupAlongSatisfied = false;
      }
    }
    else
    {
      ctx.pickupCandidate = false;
      ctx.pickupCandidateStartMs = 0;
      ctx.pickupMotionStartMs = 0;
      ctx.pickupAlongStartMs = 0;
      ctx.pickupMotionSatisfied = false;
      ctx.pickupAlongSatisfied = false;

      if (putdownSignal)
      {
        if (!ctx.putdownCandidate)
        {
          ctx.putdownCandidate = true;
          ctx.putdownCandidateStartMs = nowMs;
        }
        else if ((nowMs - ctx.putdownCandidateStartMs) >= PUTDOWN_MIN_DURATION_MS)
        {
          ctx.inHand = false;
          ctx.putdownCandidate = false;
          ctx.putdownCandidateStartMs = 0;
          Serial.println("[SENS] Putdown confirmed -> Idle");
        }
      }
      else
      {
        ctx.putdownCandidate = false;
        ctx.putdownCandidateStartMs = 0;
      }
    }

    if (ctx.filteredMotion > MOTION_WAVE_THRESHOLD)
    {
      ctx.energyActive = true;
      ctx.lastEnergyMs = nowMs;
    }
    else if (ctx.energyActive && nowMs - ctx.lastEnergyMs > ENERGY_DECAY_MS && ctx.filteredMotion < MOTION_ACTIVE_THRESHOLD)
    {
      ctx.energyActive = false;
    }

    if (ctx.filteredMotion > MOTION_OVERLOAD_THRESHOLD)
    {
      ctx.overloadActive = true;
      ctx.lastOverloadMs = nowMs;
    }
    else if (ctx.overloadActive && nowMs - ctx.lastOverloadMs > OVERLOAD_DECAY_MS && ctx.filteredMotion < MOTION_ACTIVE_THRESHOLD)
    {
      ctx.overloadActive = false;
    }

    if (ENABLE_SENSOR_DEBUG && ctx.baselineCalibrated)
    {
      if (nowMs - ctx.lastDebugPrintMs >= SENSOR_DEBUG_INTERVAL_MS)
      {
        ctx.lastDebugPrintMs = nowMs;
        const unsigned long pickMs = ctx.pickupCandidate ? (nowMs - ctx.pickupCandidateStartMs) : 0;
        const unsigned long pickMotionMs = ctx.pickupMotionStartMs ? (nowMs - ctx.pickupMotionStartMs) : 0;
        const unsigned long pickAlongMs = ctx.pickupAlongStartMs ? (nowMs - ctx.pickupAlongStartMs) : 0;
        const bool motionSatisfied = ctx.pickupMotionSatisfied;
        const bool alongSatisfied = ctx.pickupAlongSatisfied;
        const unsigned long putMs = ctx.putdownCandidate ? (nowMs - ctx.putdownCandidateStartMs) : 0;
        Serial.printf(
            "[SENS][DBG] ax=%.3f ay=%.3f az=%.3f | |a|=%.3f motion=%.3f angle=%.1f | lat=%.3f along=%.3f inHand=%d pick=%d(%lu/%lu/%lu|%d%d) put=%d(%lu)\n",
            ax,
            ay,
            az,
            accelMag,
            ctx.filteredMotion,
            ctx.filteredAngle,
            ctx.filteredLateral,
            ctx.netAlongBaseline,
            ctx.inHand ? 1 : 0,
            ctx.pickupCandidate ? 1 : 0,
            pickMs,
            pickMotionMs,
            pickAlongMs,
            motionSatisfied ? 1 : 0,
            alongSatisfied ? 1 : 0,
            ctx.putdownCandidate ? 1 : 0,
            putMs);
      }
    }
  }

  void updateState(MotionContext &ctx, unsigned long nowMs)
  {
    if (!ctx.present)
    {
      ctx.state = SystemState::Initializing;
      return;
    }
    if (!ctx.baselineCalibrated)
    {
      ctx.state = SystemState::Initializing;
      return;
    }

    if (g_modeCycleRequest)
    {
      ctx.state = SystemState::ModeCycling;
      if (nowMs - ctx.lastEnergyMs > 1200)
      {
        g_modeCycleRequest = false;
      }
      return;
    }

    if (ctx.overloadActive)
    {
      ctx.state = SystemState::Overload;
      return;
    }

    if (ctx.energyActive && ctx.filteredMotion > MOTION_ACTIVE_THRESHOLD)
    {
      ctx.state = SystemState::EnergyWave;
      return;
    }

    if (ctx.inHand)
    {
      ctx.state = SystemState::Ready;
      return;
    }

    ctx.state = SystemState::Idle;
  }

  uint32_t scaleColor(uint32_t color, float scale)
  {
    scale = fmaxf(0.0f, fminf(scale, 1.0f));
    uint8_t r = static_cast<uint8_t>((color >> 16) & 0xFF);
    uint8_t g = static_cast<uint8_t>((color >> 8) & 0xFF);
    uint8_t b = static_cast<uint8_t>(color & 0xFF);
    r = static_cast<uint8_t>(r * scale);
    g = static_cast<uint8_t>(g * scale);
    b = static_cast<uint8_t>(b * scale);
    return (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) | b;
  }

  void clearRing()
  {
    for (uint16_t i = 0; i < LED_RING_COUNT; ++i)
    {
      g_ledRing.setPixelColor(i, 0);
    }
  }

  void renderState(const MotionContext &ctx, unsigned long nowMs)
  {
    SpeechLedMode speechMode = currentSpeechLedMode();
    if (speechMode == SpeechLedMode::Speaking)
    {
      renderSpeechSpeaking(nowMs);
      return;
    }
    if (speechMode == SpeechLedMode::Listening)
    {
      renderSpeechListening(nowMs);
      return;
    }

    switch (ctx.state)
    {
    case SystemState::Initializing:
      renderInitializing(nowMs);
      break;
    case SystemState::Idle:
      renderIdle(nowMs);
      break;
    case SystemState::Ready:
      renderReady(nowMs);
      break;
    case SystemState::EnergyWave:
      renderEnergyWave(nowMs);
      break;
    case SystemState::Overload:
      renderOverload(nowMs);
      break;
    case SystemState::ModeCycling:
      renderModeCycling(nowMs);
      break;
    }
  }

  void renderInitializing(unsigned long nowMs)
  {
    clearRing();
    const float progress = fmodf(nowMs / 10.0f, static_cast<float>(LED_RING_COUNT));
    const uint16_t lead = static_cast<uint16_t>(progress);
    for (uint16_t i = 0; i <= lead && i < LED_RING_COUNT; ++i)
    {
      float scale = 0.2f + (static_cast<float>(i) / LED_RING_COUNT) * 0.8f;
      g_ledRing.setPixelColor(i, scaleColor(g_ledRing.Color(0, 0, 80), scale));
    }
    g_ledRing.show();
  }

  void renderIdle(unsigned long nowMs)
  {
    clearRing();
    const float phase = (nowMs % 4000) / 4000.0f;
    const float wave = 0.5f + 0.5f * sinf(phase * 2.0f * static_cast<float>(M_PI));
    uint32_t color = g_ledRing.Color(0, 0, 180);
    color = scaleColor(color, 0.25f + 0.75f * wave);
    for (uint16_t i = 0; i < LED_RING_COUNT; ++i)
    {
      g_ledRing.setPixelColor(i, color);
    }
    g_ledRing.show();
  }

  void renderReady(unsigned long nowMs)
  {
    clearRing();
    const uint16_t index = static_cast<uint16_t>((nowMs / 120) % LED_RING_COUNT);
    for (uint16_t i = 0; i < LED_RING_COUNT; ++i)
    {
      float scale = 0.05f;
      int16_t diff = static_cast<int16_t>((i + LED_RING_COUNT) - index) % LED_RING_COUNT;
      if (diff == 0)
      {
        scale = 1.0f;
      }
      else if (diff == 1 || diff == -1)
      {
        scale = 0.25f;
      }
      g_ledRing.setPixelColor(i, scaleColor(g_ledRing.Color(120, 120, 120), scale));
    }
    g_ledRing.show();
  }

  void renderEnergyWave(unsigned long nowMs)
  {
    clearRing();
    const float phase = (nowMs % 1600) / 1600.0f;
    const float pulse = 0.4f + 0.6f * sinf(phase * 2.0f * static_cast<float>(M_PI));
    const uint32_t color = scaleColor(g_ledRing.Color(255, 120, 0), pulse);
    for (uint16_t i = 0; i < LED_RING_COUNT; ++i)
    {
      g_ledRing.setPixelColor(i, color);
    }
    g_ledRing.show();
  }

  void renderOverload(unsigned long nowMs)
  {
    clearRing();
    const bool on = (nowMs / 120) % 2 == 0;
    const uint32_t color = on ? g_ledRing.Color(255, 0, 0) : 0;
    for (uint16_t i = 0; i < LED_RING_COUNT; ++i)
    {
      g_ledRing.setPixelColor(i, color);
    }
    g_ledRing.show();
  }

  uint32_t colorWheel(uint16_t pos)
  {
    pos = 255 - pos;
    if (pos < 85)
    {
      return g_ledRing.Color(255 - pos * 3, 0, pos * 3);
    }
    if (pos < 170)
    {
      pos -= 85;
      return g_ledRing.Color(0, pos * 3, 255 - pos * 3);
    }
    pos -= 170;
    return g_ledRing.Color(pos * 3, 255 - pos * 3, 0);
  }

  void renderModeCycling(unsigned long nowMs)
  {
    clearRing();
    for (uint16_t i = 0; i < LED_RING_COUNT; ++i)
    {
      uint16_t wheelPos = static_cast<uint16_t>((i * 256 / LED_RING_COUNT) + (nowMs / 8));
      g_ledRing.setPixelColor(i, colorWheel(wheelPos & 0xFF));
    }
    g_ledRing.show();
  }

  void renderSpeechListening(unsigned long nowMs)
  {
    clearRing();
    const uint16_t head = static_cast<uint16_t>((nowMs / 140) % LED_RING_COUNT);
    for (uint16_t i = 0; i < LED_RING_COUNT; ++i)
    {
      float scale = 0.05f;
      uint16_t dist = (head + LED_RING_COUNT - i) % LED_RING_COUNT;
      if (dist == 0)
      {
        scale = 1.0f;
      }
      else if (dist == 1 || dist == LED_RING_COUNT - 1)
      {
        scale = 0.35f;
      }
      g_ledRing.setPixelColor(i, scaleColor(g_ledRing.Color(40, 120, 255), scale));
    }
    g_ledRing.show();
  }

  void renderSpeechSpeaking(unsigned long nowMs)
  {
    clearRing();
    const float phase = (nowMs % 1200) / 1200.0f;
    const float pulse = 0.6f + 0.4f * sinf(phase * 2.0f * static_cast<float>(M_PI));
    const uint32_t color = scaleColor(g_ledRing.Color(255, 255, 255), pulse);
    for (uint16_t i = 0; i < LED_RING_COUNT; ++i)
    {
      g_ledRing.setPixelColor(i, color);
    }
    g_ledRing.show();
  }

  void sensorTask(void *param)
  {
    (void)param;

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

    g_ledRing.begin();
    g_ledRing.setBrightness(LED_BRIGHTNESS);
    g_ledRing.show();

    MotionContext ctx{};
    ctx.present = scanMpuAddress(ctx);

    if (ctx.present)
    {
      writeMpuReg(ctx.mpuAddr, 0x6B, 0x01); // wake
      writeMpuReg(ctx.mpuAddr, 0x1C, 0x00); // +/-2g accel
      writeMpuReg(ctx.mpuAddr, 0x1B, 0x08); // gyro 500 dps
      writeMpuReg(ctx.mpuAddr, 0x1A, 0x03); // DLPF
      writeMpuReg(ctx.mpuAddr, 0x19, 9);    // sample divider
      Serial.printf("[SENS] MPU6050 detected at 0x%02X\n", ctx.mpuAddr);
    }
    else
    {
      Serial.println("[SENS][WARN] MPU6050 not detected. LED animations will fallback.");
    }

    for (;;)
    {
      const unsigned long nowMs = millis();
      if (ctx.present)
      {
        updateMotion(ctx, nowMs);
      }
      updateState(ctx, nowMs);
      renderState(ctx, nowMs);
      vTaskDelay(pdMS_TO_TICKS(MOTION_LOOP_DELAY_MS));
    }
  }

  SpeechLedMode currentSpeechLedMode()
  {
    portENTER_CRITICAL(&g_speechMux);
    SpeechLedMode mode = g_speechMode;
    portEXIT_CRITICAL(&g_speechMux);
    return mode;
  }

  void setSpeechLedMode(SpeechLedMode mode)
  {
    portENTER_CRITICAL(&g_speechMux);
    g_speechMode = mode;
    portEXIT_CRITICAL(&g_speechMux);
  }

} // namespace
