#pragma once

#include <Arduino.h>

struct TelemetryAlertEntry
{
  String type;
  String detail;
};

class TelemetryPayloadBuilder
{
public:
  TelemetryPayloadBuilder();

  void reset();

  void setFall(const String &state, float tiltDeg, float impactG);
  void clearFall();

  bool addAlert(const String &type, const String &detail = String());

  String build(uint64_t timestampMs = millis()) const;

private:
  static String escapeJson(const String &input);
  static void appendKeyValue(String &json, const char *key, const String &value, bool &first);
  static void appendKeyValue(String &json, const char *key, float value, uint8_t decimals, bool &first);
  static void appendKeyValue(String &json, const char *key, int value, bool &first);
  static void appendKeyValue(String &json, const char *key, uint64_t value, bool &first);
  static void appendKeyValue(String &json, const char *key, bool value, bool &first);

  bool _fallSet;
  String _fallState;
  float _fallTiltDeg;
  float _fallImpact;

  TelemetryAlertEntry _alerts[4];
  size_t _alertCount;
};
