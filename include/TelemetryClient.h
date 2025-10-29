#pragma once

#include <Arduino.h>

class TelemetryClient
{
public:
  TelemetryClient();

  void setBaseUrl(const String &baseUrl);
  void setDeviceId(const String &deviceId);

  const String &baseUrl() const { return _baseUrl; }
  const String &deviceId() const { return _deviceId; }

  bool isReady() const;

  bool queueAudioClip(const String &clipName) const;
  bool queueAudioUrl(const String &url) const;
  bool notifyPlaybackFinished() const;

private:
  String composeUrl(const String &path) const;
  bool postJson(const String &url, const String &jsonPayload) const;

  String _baseUrl;
  String _deviceId;
};
