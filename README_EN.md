# Audio Bridge & Remote Monitoring System
[中文版说明](README.md)

## Overview
This project combines an ESP32-based audio bridge with on-device fall detection and a Python backend that integrates with the Deepgram Agent API. The firmware streams microphone audio over Wi-Fi, plays remote WAV clips through an I2S amplifier, and drives a NeoPixel ring to visualise the fall-detection state. The Python server relays audio to Deepgram and exposes lightweight endpoints for queuing playback.

## Key Features
- **Full-duplex audio bridge**: The ESP32 pushes PCM frames from an INMP441 microphone while automatically pausing uploads during local WAV playback to avoid feedback.
- **Download-and-play queue**: The Python server can enqueue clips (local or remote URLs); the ESP32 downloads, plays, and unmutes the microphone once playback ends.
- **Fall detection**: Built-in MPU6050 processing detects impacts and sustained tilt, publishing alerts to the backend.
- **NeoPixel status ring**: Visual states for calibration, monitoring, fall alerts, and speech listening/speaking overrides.

## Repository Layout
```
├─include/            Shared C++ headers (TelemetryClient, SensorManager, ...)
├─lib/                Optional Arduino libraries (not required by default)
├─python/             Backend services & tools
│  ├─server.py        Deepgram bridge + audio control API
│  └─make_clip.py     Utility to assemble WAV test clips
├─src/
│  ├─main.cpp         ESP32 audio bridge entry point
│  └─SensorManager.cpp  Fall detection logic + LED rendering
├─platformio.ini      PlatformIO project definition
├─README.md           Chinese documentation
└─README_EN.md        English documentation
```

## Getting Started
### 1. Hardware
- FireBeetle 2 ESP32-E (or compatible ESP32 board)
- INMP441 I2S microphone, MAX98357A I2S DAC/amplifier
- MPU6050 IMU for fall detection
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
4. When the ESP32 connects, it streams microphone PCM to the backend. The server forwards audio to Deepgram and handles playback commands.

### 4. Deepgram Agent Flow
- The backend establishes an Agent socket and manages turn-taking—microphone uploads pause while the agent speaks.
- Agent replies are stored in `python/data/agent-output.wav` and played back over the ESP32 speaker.

## LED Status Cheat Sheet
- **Calibrating**: blue breathing during IMU baseline
- **Monitoring**: white chase indicating idle listening
- **Post Impact**: amber blink during the post-impact window
- **Fall Detected**: red strobe to highlight an active alert
- **Speech Listening**: dim white chase when the agent is listening
- **Speech Speaking**: soft white breathing during playback

## API Endpoints
- `POST /api/audio/play` – queue a clip (`{"clip": "...wav"}` or `{"url": "..."}`) for the ESP32
- `POST /api/audio/playback_done` – called by the firmware after playback to resume mic streaming immediately

## Troubleshooting
- **Mic resumes too late** – confirm the ESP32 can reach `/api/audio/playback_done`; firewall or base URL issues will force the fallback timer.
- **Agent ignores playback commands** – ensure the ESP32 is connected (check the server logs) and that `/api/audio/playback_done` requests succeed so the microphone resumes promptly.

## Contributing
Issues and pull requests are welcome. Please coordinate larger changes so the firmware and Python backend evolve together.
