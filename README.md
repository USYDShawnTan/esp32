# Audio Agent Bridge (ESP32 + Deepgram)

## 项目简介
本工程演示了 FireBeetle 2 ESP32-E 作为“语音管家”使用时的完整链路：设备端采集 INMP441 麦克风音频并实时推送到本地 Python 服务端，后端接入 Deepgram Agent 进行听写与应答，同时将返回的 WAV 音频通过 HTTP 下发给 ESP32 上的 MAX98357A I2S 功放播报。现已整合跌倒检测、心率/血氧、体温等传感器数据，后端提供 REST API 与简洁的 Web 控制台，支持语音播报触发与状态查询。

## 目录结构
- `src/main.cpp`：ESP32 音频桥固件，负责麦克风采集、TCP 推流、HTTP 下载与播放队列。
- `src/SensorManager.cpp`：跌倒检测、生命体征、体温传感器、NeoPixel 状态灯以及遥测事件的整合模块。
- `src/IronManEngineDetection.ino.disabled`：原始示例草稿，代码已迁移到模块化实现。
- `python/server.py`：Python 后端，会话转发、REST API 与静态资源托管。
- `python/data/`：ESP32 可下载的 WAV 音频文件夹（预制提示音、TTS 缓存等）。
- `python/static/`：纯 HTML/JS 控制台资源目录。
- `python/.env`：Deepgram API Key、HTTP_BASE_URL 等环境变量。
- `platformio.ini`：PlatformIO 构建配置。

## 前置条件
### 硬件
| 模块 | 作用 |
| ---- | ---- |
| FireBeetle 2 ESP32-E | 主控板 |
| INMP441 | I2S 麦克风（BCLK=14、LRC=12、DIN=23、DOUT=34） |
| MAX98357A | I2S 功放/喇叭 |
| MPU6050 / MAX30102 / MAX30205 等 | 跌倒、心率/血氧、体温传感器（默认 I2C SDA=21、SCL=22，可在 `SensorManager.cpp` 调整） |
| WS2812/NeoPixel 灯环 | 状态指示（默认 GPIO4） |

### 软件
- PlatformIO CLI 或 VS Code + PlatformIO 扩展
- Python 3.9 +
- pip（建议最新版）
- Deepgram 账号与 API Key（写入 `DEEPGRAM_API_KEY`）
- PlatformIO 自动拉取的库：`Adafruit NeoPixel`、`SparkFun_MAX3010x_Sensor_Library`

### 网络
- ESP32 与 Python 后端位于同一局域网
- 默认端口：TCP 桥接 9000，HTTP API/前端 8000（可通过环境变量覆盖）

## Python 端环境部署
```powershell
cd python
python -m venv .venv
.venv\Scripts\Activate.ps1
pip install --upgrade pip
pip install deepgram-sdk aiohttp
```

`.env` 示例：
```env
DEEPGRAM_API_KEY="your_key_here"
BACKEND_HOST="0.0.0.0"
BACKEND_PORT="8000"
HTTP_BASE_URL="http://192.168.137.1:8000/data"
```

## 后端运行
```powershell
cd python
.venv\Scripts\Activate.ps1
python server.py
```
终端会显示：
```
HTTP API available at http://0.0.0.0:8000
Listening for ESP32 on ('0.0.0.0', 9000)
```
浏览器访问 `http://<服务器IP>:8000/` 打开控制台，`/data/` 提供 WAV 下载。

## ESP32 烧录
1. 更新 `src/main.cpp` 里的 Wi-Fi SSID/密码、后端 IP (`MIC_SERVER_HOST`)，并按需改 `TELEMETRY_BASE_URL`、`TELEMETRY_DEVICE_ID`。
2. 连接开发板，执行：
   ```powershell
   pio run --target upload
   ```
3. 串口或网页日志中可查看 “Booting audio bridge firmware”、Wi-Fi 地址等信息。

## 控制台能力
- **Telemetry**：每秒刷新跌倒状态、HR/SpO₂、体温。
- **Recent Events & Logs**：展示 ESP32 上报的关键日志（Wi-Fi 连通、传感器上线、跌倒触发/恢复、音频排队失败等），无需依赖串口。
- **Audio Controls**：按钮触发 `alert_fall_detected.wav`、`status_all_clear.wav`，或输入任意 URL 播放自定义 WAV。

## REST API 摘要
| 方法 | 路径 | 说明 |
| ---- | ---- | ---- |
| `GET` | `/api/devices` | 获取最新遥测、事件、连接状态 |
| `POST` | `/api/devices/{id}/telemetry` | ESP32 上报传感数据 |
| `POST` | `/api/devices/{id}/events` | ESP32 上报告警/日志 |
| `POST` | `/api/audio/play` | 排队播放指令（支持 `url`、`clip`、`command`） |
| 静态 | `/data/` | WAV 下载目录 |
| 静态 | `/static/` | 控制台静态资源 |

## 传感器与 LED 集成
- 后台任务初始化 MPU6050、MAX30102、MAX30205/LM75/TMP102，并驱动 NeoPixel 灯环显示状态（校准、监控、冲击、跌倒）。
- 遥测通过 `TelemetryPayloadBuilder` + `TelemetryClient` 每秒上报；跌倒、血氧偏低、体温过高会附带 `alerts`。
- 跌倒时播放 `alert_fall_detected.wav`，恢复时播放 `status_all_clear.wav`。
- 关键日志通过 `postLog()` 自动上传，网页 “Recent Events & Logs” 即时显示。
- 若需调整引脚或阈值，编辑 `src/SensorManager.cpp` 顶部常量。

## WAV 资源
将任意 16-bit 单声道 WAV 放入 `python/data/`，后端即提供下载。Deepgram 生成的 `agent-output.wav` 也位于该目录，可供播放回放。

## 常见问题
- **Wi-Fi 连接失败**：检查 `WIFI_SSID`/`WIFI_PASSWORD`。
- **HTTP 404**：确认 `TELEMETRY_BASE_URL` 指向正确主机（例如 `http://192.168.137.1:8000`）。
- **音频无响应**：查看控制台日志是否有 `[AUDIO][ERR]` 或后端 `/api/audio/play` 返回异常。
- **麦克风/传感器冲突**：INMP441 DOUT 使用 GPIO34，I2C 走 21/22，确保接线一致。

## 停止服务
- 后端 `Ctrl+C` 停止 `python/server.py`。
- 退出虚拟环境：`deactivate`。
- 按需拔掉 ESP32 或切断电源。

---
后续可继续调试阈值、增加更多事件类型或加入 WebSocket 实时推送等扩展功能。
