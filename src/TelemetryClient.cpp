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
