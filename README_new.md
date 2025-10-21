# Audio Agent Bridge (ESP32 + Deepgram)

## 项目简介
本项目演示如何让 FireBeetle 2 ESP32-E 采集 INMP441 麦克风的音频，通过 Wi-Fi 推送到笔记本上的 Python 桥接程序，再调用 Deepgram Agent 获取回复，并把生成的 WAV 音频回传给板子，通过 MAX98357A I2S 功放播放。

## 目录结构
- `src/main.cpp`：ESP32 固件，负责麦克风采集、TCP 发送和播放命令处理。
- `python/server.py`：Python 端桥接程序，转发音频给 Deepgram Agent 并下发播放指令。
- `python/.env`：存放 `DEEPGRAM_API_KEY` 等密钥。
- `platformio.ini`：PlatformIO 环境配置。

## 前置条件
1. **硬件**
   - FireBeetle 2 ESP32-E
   - INMP441 I2S 麦克风
   - MAX98357A I2S 功放
   - 扬声器、锂电池
2. **软件**
   - PlatformIO CLI 或 VS Code + PlatformIO 插件
   - Python 3.9 及以上
   - Deepgram 账号与 API Key (`DEEPGRAM_API_KEY`)
3. **网络**
   - ESP32 与运行 Python 的电脑需在同一局域网或热点下
   - 电脑上需可运行一个简单 HTTP 服务器（Python 标准库即可）

## Python 端环境配置
```powershell
cd python
python -m venv .venv
.venv\Scripts\Activate.ps1
pip install --upgrade pip
pip install deepgram-sdk
```

将 Deepgram Key 写入 `.env`（已准备好示例文件）：
```env
DEEPGRAM_API_KEY="your_key_here"
```

如果你需要使用不同的 HTTP 端口或主机，请同步修改：
- `python/server.py` 中的 `HTTP_BASE_URL`
- `src/main.cpp` 中的 `MIC_SERVER_HOST`、`MIC_SERVER_PORT`

## 启动流程
1. **启动 HTTP 服务器（提供 WAV 下载）**
   ```powershell
   cd python
   python -m http.server 8000
   ```
   建议在单独的终端窗口运行，保持开启。

2. **启动 Deepgram 桥接程序**
   ```powershell
   cd python
.venv\Scripts\Activate.ps1
python server.py
```
   终端看到 `Listening for ESP32 on ('0.0.0.0', 9000)` 表示就绪。

3. **刷新并烧录 ESP32 固件**
   ```powershell
   cd ..
   pio run --target upload
```
   或者在 VS Code 的 PlatformIO 面板中点击 `Upload`。

4. **供电并连接 ESP32**
   - 上电后，固件会自动连接在 `main.cpp` 中配置的 Wi-Fi。
   - 连接成功后，会向 Python 桥接程序发起 TCP 连接并推流 PCM。

5. **体验对话**
   - 对麦克风讲话。
   - Deepgram Agent 返回语音后，Python 端会保存 `output-*.wav` 并向 ESP32 发出 `PLAY <url>` 指令。
   - ESP32 通过 HTTP 下载 WAV 并通过 MAX98357A 播放。

## 常见问题排查
- **ESP32 无法连上 Wi-Fi**
  - 检查 `src/main.cpp` 中的 `WIFI_SSID` 和 `WIFI_PASSWORD` 是否正确。
- **Python 端报错找不到 Deepgram**
  - 确认已激活虚拟环境并执行 `pip install deepgram-sdk`。
- **播放失败或无声**
  - 查看串口输出是否收到 `PLAY` 指令。
  - 确认 HTTP 服务器运行在 `HTTP_BASE_URL` 指定的地址和端口。
  - 检查 I2S 线序及扬声器供电。
- **需要更换网络或端口**
  - 同时修改 ESP32 固件中的常量以及 Python 脚本中的 `LISTEN_HOST` / `LISTEN_PORT` 和 `HTTP_BASE_URL`。

## 关停
1. 在 Python 终端按 `Ctrl+C` 结束 `server.py`。
2. 关闭 HTTP 服务器窗口。
3. 给 ESP32 断电或停止串口监视器。

完成以上步骤，即可在演示时通过锂电池供电、依靠 Wi-Fi 完成端到端语音交互。祝演示顺利!

