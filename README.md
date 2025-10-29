# 音频桥接与远程监测系统

[English Version](README_EN.md)

## 项目概述

本项目基于 ESP32（FireBeetle 2 ESP32-E）实现语音桥接与跌倒检测，同时配套 Python 服务器与 Deepgram 语音代理集成。设备通过 I2S 麦克风与扬声器实现全双工音频，采集 MPU6050 跌倒状态并驱动 LED 指示，同时支持远程触发音频播放。

## 主要功能

- **语音桥接**：ESP32 以 PCM 流方式推送麦克风数据，Python 服务器与 Deepgram Agent 交互，实现语音问答与自定义音频播放。
- **音频播放队列**：服务器可下发播放命令（WAV），固件自动下载、播放并在播放期间暂停上行麦克风数据以避免回声。
- **跌倒检测**：基于 MPU6050 的姿态与冲击分析，将跌倒状态与告警同步到后端。
- **LED 状态指示**：NeoPixel 灯环指示校准、监控、跌倒警报、语音监听/播报等多种状态。

## 目录结构

```
├─include/           C++ 头文件 (TelemetryClient, SensorManager 等)
├─lib/               可选 Arduino 库（默认未使用）
├─python/            Python 服务与工具脚本
│  ├─server.py       Deepgram Agent 音频桥接 & 播放 API
│  └─make_clip.py    WAV 测试音频生成脚本
├─src/               ESP32 固件源码
│  ├─main.cpp        音频桥接、播放、麦克风上传
│  └─SensorManager.cpp  跌倒检测与 LED 控制
├─platformio.ini     PlatformIO 项目配置
├─README.md / README_EN.md
```

## 快速上手

### 1. 硬件准备

- FireBeetle 2 ESP32-E（或兼容 ESP32 开发板）
- INMP441 I2S 麦克风、MAX98357A I2S 扬声器
- MPU6050 IMU（用于跌倒检测）
- NeoPixel（16 灯环，数据引脚默认 4）

### 2. 固件编译烧录

1. 安装 [PlatformIO](https://platformio.org/)，推荐使用 VS Code 插件。
2. 修改 `src/main.cpp` 中的 Wi-Fi SSID、密码以及 Python 服务端 IP。
3. USB 连接开发板，执行 `pio run --target upload` 编译并烧录。

### 3. Python 服务器

1. 安装依赖：
   ```bash
   cd python
   pip install -r requirements.txt
   ```
2. 配置必要环境变量（可在 `python/.env` 中设定）：
   - `DEEPGRAM_API_KEY`：Deepgram API 密钥
   - `ESP_LISTEN_HOST` / `ESP_LISTEN_PORT`：ESP32 音频流监听地址（默认 0.0.0.0:9000）
   - `BACKEND_PORT`：HTTP API 端口（默认 8000）
3. 启动服务：
   ```bash
   python server.py
   ```
4. ESP32 连接后会向服务器推送麦克风 PCM 数据，服务器转发给 Deepgram Agent 并接受播放/控制指令。

### 4. Deepgram Agent 集成

`server.py` 会与 Deepgram Agent 建立 Socket 连接并保持会话：

- 当用户或 Agent 说话时，服务器自动开/关上行麦克风以避免回声。
- 服务器会定期将最新的跌倒状态写入提示词，方便 Agent 在对话中引用实时数据。
- Agent 的回复音频保存为 `python/data/agent-output.wav` 并回放至 ESP32。

## LED 状态指示摘要

- **Calibrating**：蓝色呼吸（IMU 校准阶段）
- **Monitoring**：白色旋转追逐灯（待命监听）
- **Post Impact**：琥珀色闪烁（跌倒后冷却）
- **Fall Detected**：红色快闪（跌倒警报）
- **Speech Listening**：白色追光（语音监听）
- **Speech Speaking**：白色呼吸（语音播报）

## 遥测与接口

- `POST /api/audio/play`：服务器向 ESP32 下发播放命令（`{"clip": "...wav"}` 或 `{"url": "..."}`）。
- `POST /api/audio/playback_done`：ESP32 播放结束通知，服务器即时恢复麦克风。

## 常见问题

- **麦克风延迟恢复**：若播放结束后仍看到 “fallback” 提示，请确认 ESP32 能访问 Python 服务器的 `playback_done` 接口以及网络防火墙设置。
- **Deepgram 连接失败或 Agent 未响应播放命令**：检查 `DEEPGRAM_API_KEY`、网络连通性，以及服务器日志，确认 `/api/audio/playback_done` 已成功调用。

## 贡献

欢迎提交 Issue 或 Pull Request。建议在修改前先讨论需求与方案以保持固件与服务器同步演进。
