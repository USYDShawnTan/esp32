import asyncio
import os

from deepgram import AsyncDeepgramClient
from server import DATA_DIR, load_env_files

CLIP_NAME = "alert_fall_detected.wav"
PROMPT = "Warning. A fall has been detected. Please assist immediately."


async def main() -> None:
  load_env_files()
  api_key = os.getenv("DEEPGRAM_API_KEY")
  if not api_key:
    raise RuntimeError("DEEPGRAM_API_KEY 未设置，无法调用 Deepgram Speak API。")

  client = AsyncDeepgramClient(api_key=api_key)
  audio_client = client.speak.v1.audio
  stream = audio_client.generate(
      text=PROMPT,
      model="aura-2-hyperion-en",
      encoding="linear16",
      container="wav",
      sample_rate=24000,
  )

  buffer = bytearray()
  async for chunk in stream:
    buffer.extend(chunk)

  target = DATA_DIR / CLIP_NAME
  target.write_bytes(buffer)
  print(f"已生成 WAV 文件：{target}")


if __name__ == "__main__":
  asyncio.run(main())
