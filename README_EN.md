# Audio Bridge & Remote Monitoring System
[中文版说明](README.md)

## Overview
This project combines an ESP32-based audio bridge with a multi-sensor telemetry stack and a Python backend that integrates with the Deepgram Agent API. The firmware streams microphone audio over Wi-Fi, plays remote WAV clips through an I2S amplifier, and gathers fall detection, heart-rate/SpO₂, and temperature data. The Python server relays audio to Deepgram, injects fresh telemetry into the dialogue, and exposes HTTP endpoints for logs, events, and audio commands.

## Key Features
- **Full-duplex audio bridge**: The ESP32 pushes PCM frames from an INMP441 microphone while automatically pausing uploads during local WAV playback to avoid feedback.
- **Download-and-play queue**: The Python server can enqueue clips (local or remote URLs); the ESP32 downloads, plays, and unmutes the microphone once playback ends.
- **Sensor suite**: Built-in support for MPU6050 fall detection, MAX30102 heart-rate/SpO2, and auto-detected temperature sensors (MAX30205/LM75/TMP102).
- **NeoPixel status ring**: Visual states for calibration, monitoring, fall alerts, and speech listening/speaking overrides.
- **Telemetry & logging**: The firmware periodically posts JSON telemetry and log events to the Python backend; the backend keeps the latest snapshot and shares it with the voice agent on demand.

## Repository Layout
```
├─include/            Shared C++ headers (TelemetryClient, SensorManager, ...)
├─lib/                Optional Arduino libraries (not required by default)
├─python/             Backend services & tools
│  ├─server.py        Deepgram bridge + telemetry REST API
│  └─make_clip.py     Utility to assemble WAV test clips
├─src/
│  ├─main.cpp         ESP32 audio bridge entry point
│  └─SensorManager.cpp  Fall/vitals/temperature logic + LED rendering
├─platformio.ini      PlatformIO project definition
├─README.md           Chinese documentation
└─README_EN.md        English documentation
```

## Getting Started
### 1. Hardware
- FireBeetle 2 ESP32-E (or compatible ESP32 board)
- INMP441 I2S microphone, MAX98357A I2S DAC/amplifier
- MPU6050 IMU, MAX30102 pulse-oximeter, I²C temperature sensor
- 16-pixel NeoPixel ring (data pin defaults to GPIO 4)

### 2. Firmware (PlatformIO)
1. Install [PlatformIO](https://platformio.org/) (VS Code extension recommended).
2. Edit `src/main.cpp` to set Wi-Fi credentials and the Python server IP/port.
3. Connect the board over USB and run `pio run --target upload`.

### 3. Python Backend
1. Install dependencies:
   ```bash
   cd python
   pip install -r requirements.txt  # or install aiohttp, deepgram, numpy, etc. manually
   ```
2. Provide environment variables (via shell or `python/.env`):
   - `DEEPGRAM_API_KEY`
   - `ESP_LISTEN_HOST` / `ESP_LISTEN_PORT` (default `0.0.0.0:9000`)
   - `BACKEND_PORT` (default `8000`)
3. Launch the server:
   ```bash
   python server.py
   ```
4. When the ESP32 connects, it streams microphone PCM to the backend. The server forwards audio to Deepgram, handles playback commands, and records telemetry snapshots.

### 4. Deepgram Agent Flow
- The backend establishes an Agent socket and manages turn-taking—microphone uploads pause while the agent speaks.
- Every 10 seconds the server refreshes the agent prompt with the most recent sensor readings (or fallback values if telemetry is stale), so questions like "What's my heart rate?" return contextual answers.
- Agent replies are stored in `python/data/agent-output.wav` and played back over the ESP32 speaker.

## LED Status Cheat Sheet
- **Calibrating**: blue breathing during IMU baseline
- **Monitoring**: white chase indicating idle listening
- **Post Impact**: amber blink during the post-impact window
- **Fall Detected**: red strobe to highlight an active alert
- **Speech Listening**: dim white chase when the agent is listening
- **Speech Speaking**: soft white breathing during playback

## API Endpoints
- `POST /api/devices/{deviceId}/telemetry` – ingest telemetry (fall, vitals, temperature)
- `POST /api/audio/play` – queue a clip (`{"clip": "...wav"}` or `{"url": "..."}`) for the ESP32
- `POST /api/audio/playback_done` – called by the firmware after playback to resume mic streaming immediately
- `GET /api/devices` – fetch latest telemetry snapshot, alerts, and bridge status

## Troubleshooting
- **Mic resumes too late** – confirm the ESP32 can reach `/api/audio/playback_done`; firewall or base URL issues will force the fallback timer.
- **Negative temperature / nonsense vitals** – the server clamps out-of-range readings and substitutes nominal values; inspect sensor wiring or calibration if problems persist.
- **Agent refuses telemetry questions** – ensure the backend logs show “[Vitals] Prompt refreshed …”. Missing Deepgram credentials or connectivity issues will stop the periodic prompt updates.

## Contributing
Issues and pull requests are welcome. Please coordinate larger changes so the firmware and Python backend evolve together.
