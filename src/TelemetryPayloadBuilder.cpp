#include "TelemetryPayloadBuilder.h"

#include <inttypes.h>

TelemetryPayloadBuilder::TelemetryPayloadBuilder()
{
  reset();
}

void TelemetryPayloadBuilder::reset()
{
  _fallSet = false;
  _fallState = "";
  _fallTiltDeg = 0.0f;
  _fallImpact = 0.0f;

  _vitalsSet = false;
  _heartRate = 0;
  _heartRateValid = false;
  _spo2 = 0;
  _spo2Valid = false;

  _temperatureSet = false;
  _temperatureC = 0.0f;
  _temperatureStatus = "";

  _alertCount = 0;
}

void TelemetryPayloadBuilder::setFall(const String &state, float tiltDeg, float impactG)
{
  _fallSet = true;
  _fallState = state;
  _fallTiltDeg = tiltDeg;
  _fallImpact = impactG;
}

void TelemetryPayloadBuilder::clearFall()
{
  _fallSet = false;
  _fallState = "";
  _fallTiltDeg = 0.0f;
  _fallImpact = 0.0f;
}

void TelemetryPayloadBuilder::setVitals(int heartRate, bool heartRateValid, int spo2, bool spo2Valid)
{
  _vitalsSet = true;
  _heartRate = heartRate;
  _heartRateValid = heartRateValid;
  _spo2 = spo2;
  _spo2Valid = spo2Valid;
}

void TelemetryPayloadBuilder::clearVitals()
{
  _vitalsSet = false;
  _heartRate = 0;
  _heartRateValid = false;
  _spo2 = 0;
  _spo2Valid = false;
}

void TelemetryPayloadBuilder::setTemperature(float valueC, const String &status)
{
  _temperatureSet = true;
  _temperatureC = valueC;
  _temperatureStatus = status;
}

void TelemetryPayloadBuilder::clearTemperature()
{
  _temperatureSet = false;
  _temperatureC = 0.0f;
  _temperatureStatus = "";
}

bool TelemetryPayloadBuilder::addAlert(const String &type, const String &detail)
{
  if (_alertCount >= (sizeof(_alerts) / sizeof(_alerts[0])))
  {
    return false;
  }
  _alerts[_alertCount].type = type;
  _alerts[_alertCount].detail = detail;
  ++_alertCount;
  return true;
}

String TelemetryPayloadBuilder::build(uint64_t timestampMs) const
{
  String json = "{";
  bool firstRootField = true;

  const float timestampSeconds = static_cast<float>(timestampMs) / 1000.0f;
  appendKeyValue(json, "timestamp", timestampSeconds, 3, firstRootField);

  bool hasSensors = _fallSet || _vitalsSet || _temperatureSet;
  if (hasSensors)
  {
    if (!firstRootField)
    {
      json += ",";
    }
    json += "\"sensors\":{";
    bool firstSensorField = true;

    if (_fallSet)
    {
      if (!firstSensorField)
      {
        json += ",";
      }
      json += "\"fall\":{";
      bool firstFallField = true;
      appendKeyValue(json, "state", escapeJson(_fallState), firstFallField);
      appendKeyValue(json, "tiltDeg", _fallTiltDeg, 2, firstFallField);
      appendKeyValue(json, "impactG", _fallImpact, 2, firstFallField);
      json += "}";
      firstSensorField = false;
    }

    if (_vitalsSet)
    {
      if (!firstSensorField)
      {
        json += ",";
      }
      json += "\"vitals\":{";
      bool firstVitalsField = true;
      appendKeyValue(json, "heartRate", _heartRate, firstVitalsField);
      appendKeyValue(json, "heartRateValid", _heartRateValid, firstVitalsField);
      appendKeyValue(json, "spo2", _spo2, firstVitalsField);
      appendKeyValue(json, "spo2Valid", _spo2Valid, firstVitalsField);
      json += "}";
      firstSensorField = false;
    }

    if (_temperatureSet)
    {
      if (!firstSensorField)
      {
        json += ",";
      }
      json += "\"temperature\":{";
      bool firstTempField = true;
      appendKeyValue(json, "valueC", _temperatureC, 2, firstTempField);
      if (_temperatureStatus.length() > 0)
      {
        appendKeyValue(json, "status", escapeJson(_temperatureStatus), firstTempField);
      }
      json += "}";
    }

    json += "}";
    firstRootField = false;
  }

  if (_alertCount > 0)
  {
    if (!firstRootField)
    {
      json += ",";
    }
    json += "\"alerts\":[";
    for (size_t i = 0; i < _alertCount; ++i)
    {
      if (i > 0)
      {
        json += ",";
      }
      json += "{";
      bool firstAlertField = true;
      appendKeyValue(json, "type", escapeJson(_alerts[i].type), firstAlertField);
      if (_alerts[i].detail.length() > 0)
      {
        appendKeyValue(json, "detail", escapeJson(_alerts[i].detail), firstAlertField);
      }
      json += "}";
    }
    json += "]";
    firstRootField = false;
  }

  json += "}";
  return json;
}

String TelemetryPayloadBuilder::escapeJson(const String &input)
{
  String output;
  output.reserve(input.length());
  for (size_t i = 0; i < input.length(); ++i)
  {
    const char c = input.charAt(i);
    switch (c)
    {
    case '\"':
      output += "\\\"";
      break;
    case '\\':
      output += "\\\\";
      break;
    case '\b':
      output += "\\b";
      break;
    case '\f':
      output += "\\f";
      break;
    case '\n':
      output += "\\n";
      break;
    case '\r':
      output += "\\r";
      break;
    case '\t':
      output += "\\t";
      break;
    default:
      if (static_cast<uint8_t>(c) < 0x20)
      {
        char buffer[7];
        snprintf(buffer, sizeof(buffer), "\\u%04x", static_cast<unsigned>(c));
        output += buffer;
      }
      else
      {
        output += c;
      }
      break;
    }
  }
  return output;
}

void TelemetryPayloadBuilder::appendKeyValue(String &json, const char *key, const String &value, bool &first)
{
  if (!first)
  {
    json += ",";
  }
  json += "\"";
  json += key;
  json += "\":\"";
  json += value;
  json += "\"";
  first = false;
}

void TelemetryPayloadBuilder::appendKeyValue(String &json, const char *key, float value, uint8_t decimals, bool &first)
{
  if (!first)
  {
    json += ",";
  }
  json += "\"";
  json += key;
  json += "\":";
  json += String(value, decimals);
  first = false;
}

void TelemetryPayloadBuilder::appendKeyValue(String &json, const char *key, int value, bool &first)
{
  if (!first)
  {
    json += ",";
  }
  json += "\"";
  json += key;
  json += "\":";
  json += String(value);
  first = false;
}

void TelemetryPayloadBuilder::appendKeyValue(String &json, const char *key, uint64_t value, bool &first)
{
  if (!first)
  {
    json += ",";
  }
  json += "\"";
  json += key;
  json += "\":";
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%llu", static_cast<unsigned long long>(value));
  json += buffer;
  first = false;
}

void TelemetryPayloadBuilder::appendKeyValue(String &json, const char *key, bool value, bool &first)
{
  if (!first)
  {
    json += ",";
  }
  json += "\"";
  json += key;
  json += "\":";
  json += value ? "true" : "false";
  first = false;
}
