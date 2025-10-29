#include "LedController.h"

#include <Adafruit_NeoPixel.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace
{
  Adafruit_NeoPixel *g_led = nullptr;
  uint8_t g_pin = 0;
  uint16_t g_count = 0;

  portMUX_TYPE g_speechMux = portMUX_INITIALIZER_UNLOCKED;
  SpeechLedMode g_speechMode = SpeechLedMode::None;

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
    if (!g_led)
    {
      return;
    }
    for (uint16_t i = 0; i < g_led->numPixels(); ++i)
    {
      g_led->setPixelColor(i, 0);
    }
  }

  void renderInitializing(unsigned long nowMs)
  {
    if (!g_led)
    {
      return;
    }
    clearRing();
    const float progress = fmodf(nowMs / 10.0f, static_cast<float>(g_led->numPixels()));
    const uint16_t lead = static_cast<uint16_t>(progress);
    for (uint16_t i = 0; i <= lead && i < g_led->numPixels(); ++i)
    {
      float scale = 0.2f + (static_cast<float>(i) / g_led->numPixels()) * 0.8f;
      g_led->setPixelColor(i, scaleColor(g_led->Color(0, 0, 80), scale));
    }
    g_led->show();
  }

  void renderIdle(unsigned long nowMs)
  {
    if (!g_led)
    {
      return;
    }
    clearRing();
    const float phase = (nowMs % 4000) / 4000.0f;
    const float wave = 0.5f + 0.5f * sinf(phase * 2.0f * static_cast<float>(M_PI));
    uint32_t color = g_led->Color(0, 0, 180);
    color = scaleColor(color, 0.25f + 0.75f * wave);
    for (uint16_t i = 0; i < g_led->numPixels(); ++i)
    {
      g_led->setPixelColor(i, color);
    }
    g_led->show();
  }

  void renderReady(unsigned long nowMs)
  {
    if (!g_led)
    {
      return;
    }
    clearRing();
    const uint16_t index = static_cast<uint16_t>((nowMs / 120) % g_led->numPixels());
    for (uint16_t i = 0; i < g_led->numPixels(); ++i)
    {
      float scale = 0.05f;
      int16_t diff = static_cast<int16_t>((i + g_led->numPixels()) - index) % g_led->numPixels();
      if (diff == 0)
      {
        scale = 1.0f;
      }
      else if (diff == 1 || diff == -1)
      {
        scale = 0.25f;
      }
      g_led->setPixelColor(i, scaleColor(g_led->Color(120, 120, 120), scale));
    }
    g_led->show();
  }

  void renderOverload(unsigned long nowMs)
  {
    if (!g_led)
    {
      return;
    }
    clearRing();
    const bool on = (nowMs / 120) % 2 == 0;
    const uint32_t color = on ? g_led->Color(255, 0, 0) : 0;
    for (uint16_t i = 0; i < g_led->numPixels(); ++i)
    {
      g_led->setPixelColor(i, color);
    }
    g_led->show();
  }

  void renderSpeechListeningInternal(unsigned long nowMs)
  {
    if (!g_led)
    {
      return;
    }
    clearRing();
    const uint16_t head = static_cast<uint16_t>((nowMs / 140) % g_led->numPixels());
    for (uint16_t i = 0; i < g_led->numPixels(); ++i)
    {
      float scale = 0.05f;
      uint16_t dist = (head + g_led->numPixels() - i) % g_led->numPixels();
      if (dist == 0)
      {
        scale = 1.0f;
      }
      else if (dist == 1 || dist == g_led->numPixels() - 1)
      {
        scale = 0.35f;
      }
      g_led->setPixelColor(i, scaleColor(g_led->Color(40, 120, 255), scale));
    }
    g_led->show();
  }

  void renderSpeechSpeakingInternal(unsigned long nowMs)
  {
    if (!g_led)
    {
      return;
    }
    clearRing();
    const float phase = (nowMs % 1200) / 1200.0f;
    const float pulse = 0.6f + 0.4f * sinf(phase * 2.0f * static_cast<float>(M_PI));
    const uint32_t color = scaleColor(g_led->Color(255, 255, 255), pulse);
    for (uint16_t i = 0; i < g_led->numPixels(); ++i)
    {
      g_led->setPixelColor(i, color);
    }
    g_led->show();
  }

} // namespace

void ledControllerInit(uint8_t pin, uint16_t count, uint32_t brightness)
{
  if (g_led && (pin != g_pin || count != g_count))
  {
    delete g_led;
    g_led = nullptr;
  }
  if (!g_led)
  {
    g_led = new Adafruit_NeoPixel(count, pin, NEO_GRB + NEO_KHZ800);
    g_pin = pin;
    g_count = count;
  }
  g_led->begin();
  g_led->setBrightness(brightness);
  g_led->show();
}

void ledControllerRenderScene(LedScene scene, unsigned long nowMs)
{
  switch (scene)
  {
  case LedScene::Initializing:
    renderInitializing(nowMs);
    break;
  case LedScene::Idle:
    renderIdle(nowMs);
    break;
  case LedScene::Ready:
    renderReady(nowMs);
    break;
  case LedScene::Overload:
    renderOverload(nowMs);
    break;
  }
}

void ledControllerRenderSpeechListening(unsigned long nowMs)
{
  renderSpeechListeningInternal(nowMs);
}

void ledControllerRenderSpeechSpeaking(unsigned long nowMs)
{
  renderSpeechSpeakingInternal(nowMs);
}

void ledControllerSetSpeechMode(SpeechLedMode mode)
{
  portENTER_CRITICAL(&g_speechMux);
  g_speechMode = mode;
  portEXIT_CRITICAL(&g_speechMux);
}

SpeechLedMode ledControllerCurrentSpeechMode()
{
  portENTER_CRITICAL(&g_speechMux);
  SpeechLedMode mode = g_speechMode;
  portEXIT_CRITICAL(&g_speechMux);
  return mode;
}
