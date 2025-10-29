#pragma once

#include <Arduino.h>

enum class SpeechLedMode
{
  None,
  Listening,
  Speaking
};

enum class LedScene
{
  Initializing,
  Idle,
  Ready,
  Overload
};

void ledControllerInit(uint8_t pin, uint16_t count, uint32_t brightness);
void ledControllerRenderScene(LedScene scene, unsigned long nowMs);
void ledControllerRenderSpeechListening(unsigned long nowMs);
void ledControllerRenderSpeechSpeaking(unsigned long nowMs);

void ledControllerSetSpeechMode(SpeechLedMode mode);
SpeechLedMode ledControllerCurrentSpeechMode();
