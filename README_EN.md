# Audio Bridge & Telemetry System

[中文说明](README.md)

## Overview

This project pairs a FireBeetle 2 ESP32-E with a Python backend to deliver an “AI voice assistant + local sensing” experience. The ESP32 streams microphone audio to the Python server, which pipes it into Deepgram Agent for speech understanding and TTS. The server, in turn, pushes playback/control commands back to the ESP32. On-device sensors (MPU6050 + NeoPixel ring) provide pickup detection, fall alerts, and speech-state lighting.

## Highlights

- **Full-duplex audio path** – The ESP32 uploads 16 kHz PCM from an INMP441 microphone. Deepgram handles ASR + Agent logic, and sends back 24 kHz WAV responses that the ESP32 downloads and plays via MAX98357A.
- **Three-phase speech control** – A preset greeting plays on boot while the microphone stays muted; only a pickup event (`SPEECH LISTENING`) re-enables capture. During conversations the system automatically switches between Listening/Speaking, and returns to the correct state once playback ends.
- **Motion & fall sensing** – MPU6050 tracks pickup/putdown state and high-g overloads. Violent shaking forces the Overload LED scene (red flash) regardless of speech overrides.
- **Telemetry & control API** – The Python service exposes HTTP endpoints for enqueuing clips, acknowledging playback completion, and explicitly toggling microphone capture.

## Repository Layout

```
include/               C++ headers (TelemetryClient, SensorManager, ...)
python/
  server.py            Deepgram bridge + REST API
  make_clip.py         Utility for preparing WAV assets
src/
  main.cpp             ESP32 main loop: audio upload + command handling
  SensorManager.cpp    IMU state machine + LED rendering
platformio.ini         PlatformIO configuration
README.md / README_EN.md
```

## Hardware Checklist

- FireBeetle 2 ESP32-E (or compatible ESP32 module)
- INMP441 I2S microphone
- MAX98357A I2S DAC/amp
- MPU6050 IMU
- NeoPixel ring (default 16 LEDs on GPIO 4)
- Stable 5 V / 3.3 V power and wiring

## Firmware Deployment

1. Install [PlatformIO](https://platformio.org/) (VS Code extension recommended).
2. Edit `src/main.cpp` to configure Wi‑Fi credentials and the Python server IP/port.
3. Connect the board over USB and run:
   ```bash
   pio run --target upload
   ```
4. For serial logs, use `pio device monitor`.

## Python Backend

1. Install dependencies:
   ```bash
   cd python
   pip install -r requirements.txt
   ```
2. Provide environment variables (via shell or `python/.env`):
   - `DEEPGRAM_API_KEY` – Deepgram access token
   - `ESP_LISTEN_HOST` / `ESP_LISTEN_PORT` – inbound audio listener (default `0.0.0.0:9000`)
   - `BACKEND_PORT` – REST API port (default 8000)
   - Optionally override `HTTP_BASE_URL` if hosting WAV assets elsewhere
3. Launch:
   ```bash
   python server.py
   ```
4. When the ESP32 connects you should see handshake logs, Deepgram session events, playback commands, etc.

## Voice Interaction Flow

1. **Boot greeting** – Once the agent settings are applied, the server looks for `data/agent-greeting.wav`. If present it queues playback, hard-mutes the microphone, and prevents auto-unmute until a manual trigger arrives.
2. **Pickup unlock** – SensorManager detects `ctx.inHand` and calls `sensorManagerTriggerSpeechListening()`, which updates LEDs and invokes `/api/mic/listen` so the Python service re-opens the mic stream.
3. **Dialogue loop**  
   - User speech → Deepgram emits `AgentV1UserStartedSpeakingEvent`; if the mic is unlocked, audio continues streaming.  
   - Agent speech → `AgentV1AgentStartedSpeakingEvent` pauses upstream audio, sets the LEDs to Speaking, and saves the WAV response (`python/data/agent-output.wav`). The ESP32 downloads/plays the clip.  
   - Playback complete → The server either schedules `schedule_mic_unmute` or waits for another manual unlock depending on the current `manual_listen_required` flag.
4. **Putdown idle** – When the IMU detects the device being placed back down, SensorManager notifies the backend to disable listening and clear speech overrides; LEDs return to Idle.

## HTTP API

| Endpoint | Method | Description |
| -------- | ------ | ----------- |
| `/api/audio/play` | POST | Queue audio for playback. Accepts `{"clip": "...wav"}` or `{"url": "http://..."}`. |
| `/api/audio/playback_done` | POST | Firmware callback after playback finishes; used to synchronize microphone state. |
| `/api/mic/listen` | POST | Explicitly toggle microphone capture: `{"active": true/false}`. Also updates speech LED overrides. |

All responses are JSON; errors return appropriate HTTP codes with details.

## LED Scene Reference

- **Initializing** – Blue progress sweep while the IMU calibrates.
- **Idle** – Soft blue breathing.
- **Ready** – Silver chase animation indicating “in hand”.
- **Overload** – Red strobe when violent motion is detected (highest priority; overrides speech effects).
- **Speech Listening** – Blue chase effect while the assistant is listening.
- **Speech Speaking** – White pulse while audio is playing.

## Troubleshooting

- **Stuck muted after greeting** – Confirm the pickup detection fired (watch serial logs) or manually call `/api/mic/listen` with `{"active": true}`. If the IMU wiring is faulty, the device may never exit Idle.
- **Deepgram session drops** – Verify `DEEPGRAM_API_KEY`, network connectivity, and check server logs for socket errors. Restart `server.py` if necessary.
- **Playback lag or failures** – Inspect ESP32 serial output for HTTP GET timing / status codes. Ensure the host serving WAV files is reachable and increase the HTTPClient timeout if needed.

## Contributing

Issues and PRs are welcome. Before submitting code:

1. Ensure PlatformIO builds succeed.
2. Run `python -m compileall python/server.py`.
3. Update both README variants if documentation changes.

Thanks for trying and improving the project!
