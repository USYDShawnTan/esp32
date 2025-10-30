# 语音桥接与遥测系统

[English Version](README_EN.md)

## 项目简介

本项目基于 FireBeetle 2 ESP32-E + Python 后端，构建「语音助理 + 本地传感器监控」系统。ESP32 通过 I2S 采集麦克风数据推送到 Python 服务，再由 Deepgram Agent 负责对话、语音合成；Python 服务反向下发音频或控制命令；本地通过 MPU6050/NeoPixel 提供握持检测、跌倒提示与语音状态灯效。

## 核心亮点

- **全双工语音链路**：ESP32 推送 16 kHz PCM 流到 Python，Deepgram 完成识别/回答；返回的 24 kHz WAV 通过 ESP32 下载播放。
- **三段式语音状态**：启动预置问候语后保持静默，只有拿起设备触发 `SPEECH LISTENING` 时才真正开麦；对话期间自动在 Listening/Speaking 状态之间切换，播放完成后再按握持状态恢复。
- **姿态/跌倒检测**：基于 MPU6050 的拿起/放下判定、剧烈运动过载检测。剧烈摇晃时强制 Overload 场景（红灯闪烁），优先级高于语音灯效。
- **遥测与控制 API**：Python 后端同时提供 HTTP API，支持外部触发播放、通知播放完成、显式控制监听状态等。

## 目录结构

```
include/               C++ 头文件 (TelemetryClient, SensorManager 等)
python/
  server.py            深度语音桥接服务 & REST API
  make_clip.py         WAV 预处理/剪辑工具
src/
  main.cpp             ESP32 主流程：音频上报、控制下发
  SensorManager.cpp    IMU 状态机 + LED 场景渲染
platformio.ini         PlatformIO 工程配置
README.md / README_EN.md
```

## 硬件准备

- FireBeetle 2 ESP32-E
- INMP441 I2S 麦克风
- MAX98357A I2S 功放
- MPU6050 六轴传感器
- NeoPixel LED 环（默认 16 颗）
- 5 V/3.3 V 供电及杜邦线

## 固件部署

1. 安装 [PlatformIO](https://platformio.org/)（推荐 VS Code 插件）。
2. 根据网络环境修改 `src/main.cpp` 中的 Wi-Fi SSID、密码以及 Python 服务器 IP/端口常量。
3. 连接 USB，执行：
   ```bash
   pio run --target upload
   ```
4. 如需串口日志，可额外运行 `pio device monitor`。

## Python 服务

1. 安装依赖：
   ```bash
   cd python
   pip install -r requirements.txt
   ```
2. 配置 `python/.env`（或外部环境变量）：
   - `DEEPGRAM_API_KEY` Deepgram 访问密钥
   - `ESP_LISTEN_HOST` / `ESP_LISTEN_PORT` ESP32 音频上报监听地址（默认 `0.0.0.0:9000`）
   - `BACKEND_PORT` HTTP API 端口（默认 8000）
   - 如需自定义静态音频目录，可调整 `HTTP_BASE_URL`
3. 启动服务：
   ```bash
   python server.py
   ```
4. ESP32 连上后会看到握手日志、Deepgram 会话日志、播放命令等。

## 语音交互流程

1. **开机问候**：ESP32 连接成功 → Python 端发送 Agent 设置 → 检测到 `data/agent-greeting.wav` 时，先下发到 ESP32 播放，同时强制关闭麦克风并阻止任何自动恢复。
2. **握持解锁**：SensorManager 检测到「拿起」（`ctx.inHand`）时，触发 `sensorManagerTriggerSpeechListening()`，并通过 TelemetryClient 调用 `/api/mic/listen` 让 Python 服务重新开麦。
3. **实时对话**：
   - Deepgram 捕获到用户讲话 → Python 记录 `AgentV1UserStartedSpeakingEvent`，若麦克风已解锁则继续上传音频。
   - Agent 开口 → `AgentV1AgentStartedSpeakingEvent` 暂停上行流、切换 LED 至 Speaking → TTS 结果保存为 `agent-output.wav` 并下发播放。
   - 播放完成 → Python 根据当前是否仍需手动解锁决定是否自动 `schedule_mic_unmute`。
4. **放下休眠**：SensorManager 侦测到「放下」会通知 Python 关闭麦克风并清除语音覆盖；LED 返回 Idle 状态。

## HTTP API

| 接口 | 方法 | 说明 |
| ---- | ---- | ---- |
| `/api/audio/play` | POST | 参数：`{"clip": "xxx.wav"}` 或 `{"url": "http://..."}`。排队播放音频（会暂停麦克风）。 |
| `/api/audio/playback_done` | POST | 由 ESP32 在播放结束时回调，Python 服务收到后恢复后续逻辑。 |
| `/api/mic/listen` | POST | 参数：`{"active": true/false}`。允许外部显式控制麦克风开关（会同步影响语音灯效）。 |

> 所有接口返回 JSON，失败时会带 HTTP 状态码与错误描述。

## LED 场景速览

- **Initializing**：蓝色进度条（IMU 校准）
- **Idle**：淡蓝呼吸
- **Ready**：银色跑马灯（拿起状态）
- **Overload**：红色闪烁（剧烈晃动）— 优先级最高，会覆盖语音动画
- **Speech Listening**：蓝色扫光
- **Speech Speaking**：白色脉冲

## 常见问题

- **问候后一直静音**：确认已拿起设备或通过 `/api/mic/listen` 激活监听。若传感器状态不正确，检查 MPU6050 接线及串口日志。
- **Deepgram 会话中断**：检查 API Key 是否有效，网络是否可达；必要时重启 Python 服务重新建立 socket。
- **播放延迟或失败**：观察 ESP32 串口 `HTTP GET` 日志，确认电脑 HTTP API 可访问且 WAV 文件存在；必要时降低网络负载或增加 `HTTPClient` 超时时间。

## 贡献

欢迎通过 Issue/PR 反馈问题或提交改进。提交代码前请确保：

1. PlatformIO 构建通过。
2. Python 服务 `python -m compileall python/server.py` 无语法错误。
3. 更新 README/README_EN（若涉及文档变动）。

感谢使用与贡献！
