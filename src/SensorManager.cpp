#include "SensorManager.h"

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Wire.h>
#include <math.h>

#include "LedController.h"
#include "TelemetryClient.h"

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
  constexpr float MOTION_OVERLOAD_THRESHOLD = 0.75f;
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
    Overload
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

    bool overloadActive = false;
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
  void renderOverload(unsigned long nowMs);
  void renderSpeechListening(unsigned long nowMs);
  void renderSpeechSpeaking(unsigned long nowMs);

  void sensorTask(void *param);

  TelemetryClient *g_telemetryClient = nullptr;
  bool g_lastReportedListening = false;

  void notifyListeningState(bool active)
  {
    if (!g_telemetryClient || !g_telemetryClient->isReady())
    {
      return;
    }
    if (g_lastReportedListening == active)
    {
      return;
    }
    if (g_telemetryClient->setListeningActive(active))
    {
      g_lastReportedListening = active;
    }
  }
} // namespace

void startSensorManagerTask(TelemetryClient *client)
{
  g_telemetryClient = client;
  g_lastReportedListening = false;
  xTaskCreatePinnedToCore(sensorTask, "sensorTask", 4096, nullptr, 1, nullptr, 1);
}

void sensorManagerTriggerSpeechListening()
{
  ledControllerSetSpeechMode(SpeechLedMode::Listening);
}

void sensorManagerTriggerSpeechSpeaking()
{
  ledControllerSetSpeechMode(SpeechLedMode::Speaking);
}

void sensorManagerClearSpeechOverride()
{
  ledControllerSetSpeechMode(SpeechLedMode::None);
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

    if (ctx.filteredMotion > MOTION_OVERLOAD_THRESHOLD)
    {
      ctx.overloadActive = true;
      ctx.lastOverloadMs = nowMs;
    }
    else if (ctx.overloadActive && nowMs - ctx.lastOverloadMs > OVERLOAD_DECAY_MS && ctx.filteredMotion < MOTION_IDLE_THRESHOLD)
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

    if (ctx.overloadActive)
    {
      ctx.state = SystemState::Overload;
      return;
    }

    if (ctx.inHand)
    {
      ctx.state = SystemState::Ready;
      return;
    }

    ctx.state = SystemState::Idle;
  }

  void renderState(const MotionContext &ctx, unsigned long nowMs)
  {
    if (ctx.state == SystemState::Overload)
    {
      ledControllerRenderScene(LedScene::Overload, nowMs);
      return;
    }

    SpeechLedMode speechMode = ledControllerCurrentSpeechMode();
    if (speechMode == SpeechLedMode::Speaking)
    {
      ledControllerRenderSpeechSpeaking(nowMs);
      return;
    }
    if (speechMode == SpeechLedMode::Listening)
    {
      ledControllerRenderSpeechListening(nowMs);
      return;
    }

    LedScene scene = LedScene::Idle;
    switch (ctx.state)
    {
    case SystemState::Initializing:
      scene = LedScene::Initializing;
      break;
    case SystemState::Idle:
      scene = LedScene::Idle;
      break;
    case SystemState::Ready:
      scene = LedScene::Ready;
      break;
    case SystemState::Overload:
      scene = LedScene::Overload;
      break;
    }
    ledControllerRenderScene(scene, nowMs);
  }

  void sensorTask(void *param)
  {
    (void)param;

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

    ledControllerInit(LED_RING_PIN, LED_RING_COUNT, LED_BRIGHTNESS);

    MotionContext ctx{};
    ctx.present = scanMpuAddress(ctx);

    notifyListeningState(false);

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
      const bool previouslyInHand = ctx.inHand;
      if (ctx.present)
      {
        updateMotion(ctx, nowMs);
      }
      updateState(ctx, nowMs);
      if (!previouslyInHand && ctx.inHand)
      {
        sensorManagerTriggerSpeechListening();
        notifyListeningState(true);
      }
      else if (previouslyInHand && !ctx.inHand)
      {
        sensorManagerClearSpeechOverride();
        notifyListeningState(false);
      }
      renderState(ctx, nowMs);
      vTaskDelay(pdMS_TO_TICKS(MOTION_LOOP_DELAY_MS));
    }
  }

} // namespace
