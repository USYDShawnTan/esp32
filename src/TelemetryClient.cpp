#include "TelemetryClient.h"

#include "TelemetryClient.h"

#include <Arduino.h>
#include <HTTPClient.h>

namespace
{
String normalizeBaseUrl(const String &input)
{
  String normalized = input;
  normalized.trim();
  while (normalized.endsWith("/"))
  {
    normalized.remove(normalized.length() - 1);
  }
  return normalized;
}

String trimLeadingSlash(const String &value)
{
  size_t index = 0;
  while (index < value.length() && value.charAt(index) == '/')
  {
    ++index;
  }
  if (index == 0)
  {
    return value;
  }
  return value.substring(index);
}

String escapeJson(const String &input)
{
  String output;
  output.reserve(input.length());
  for (size_t i = 0; i < input.length(); ++i)
  {
    char c = input.charAt(i);
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
} // namespace

TelemetryClient::TelemetryClient() = default;

void TelemetryClient::setBaseUrl(const String &baseUrl)
{
  _baseUrl = normalizeBaseUrl(baseUrl);
}

void TelemetryClient::setDeviceId(const String &deviceId)
{
  _deviceId = deviceId;
}

bool TelemetryClient::isReady() const
{
  return _baseUrl.length() > 0 && _deviceId.length() > 0;
}

bool TelemetryClient::postTelemetry(const String &jsonPayload) const
{
  if (!isReady())
  {
    Serial.println("[TEL] client not ready, skip telemetry");
    return false;
  }
  String path = "/api/devices/" + _deviceId + "/telemetry";
  if (_baseUrl.endsWith("/"))
  {
    Serial.println("[TEL] base URL should not end with '/', adjusting automatically");
  }
  return postJson(composeUrl(path), jsonPayload);
}

bool TelemetryClient::postEvent(const String &jsonPayload) const
{
  if (!isReady())
  {
    Serial.println("[TEL] client not ready, skip event");
    return false;
  }
  String path = "/api/devices/" + _deviceId + "/events";
  return postJson(composeUrl(path), jsonPayload);
}

bool TelemetryClient::queueAudioClip(const String &clipName) const
{
  if (!isReady())
  {
    Serial.println("[TEL] base URL not set, cannot queue clip");
    return false;
  }
  String payload = "{\"clip\":\"";
  payload += clipName;
  payload += "\"}";
  return postJson(composeUrl("/api/audio/play"), payload);
}

bool TelemetryClient::queueAudioUrl(const String &url) const
{
  if (!isReady())
  {
    Serial.println("[TEL] base URL not set, cannot queue url");
    return false;
  }
  String payload = "{\"url\":\"";
  payload += url;
  payload += "\"}";
  return postJson(composeUrl("/api/audio/play"), payload);
}

bool TelemetryClient::queueRawCommand(const String &command) const
{
  if (!isReady())
  {
    Serial.println("[TEL] base URL not set, cannot queue command");
    return false;
  }
  String payload = "{\"command\":\"";
  payload += command;
  payload += "\"}";
  return postJson(composeUrl("/api/audio/play"), payload);
}

bool TelemetryClient::notifyPlaybackFinished() const
{
  if (!isReady())
  {
    Serial.println("[TEL] base URL not set, cannot notify playback finished");
    return false;
  }
  return postJson(composeUrl("/api/audio/playback_done"), "{}");
}

String TelemetryClient::composeUrl(const String &path) const
{
  if (_baseUrl.length() == 0)
  {
    return trimLeadingSlash(path);
  }
  String url = _baseUrl;
  if (url.endsWith("/"))
  {
    while (url.endsWith("/"))
    {
      url.remove(url.length() - 1);
    }
  }
  url += "/";
  url += trimLeadingSlash(path);
  return url;
}

bool TelemetryClient::postJson(const String &url, const String &jsonPayload) const
{
  HTTPClient http;
  if (!http.begin(url))
  {
    Serial.printf("[TEL] HTTP begin failed for %s\n", url.c_str());
    return false;
  }
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(jsonPayload);
  http.end();
  if (code < 200 || code >= 300)
  {
    Serial.printf("[TEL] HTTP POST %s returned %d\n", url.c_str(), code);
    return false;
  }
  return true;
}

bool TelemetryClient::postLog(const String &message) const
{
  if (!isReady())
  {
    Serial.println("[TEL] client not ready, skip log");
    return false;
  }
  String payload = "{\"timestamp\":";
  payload += String(millis());
  payload += ",\"type\":\"LOG\",\"message\":\"";
  payload += escapeJson(message);
  payload += "\"}";
  return postEvent(payload);
}
