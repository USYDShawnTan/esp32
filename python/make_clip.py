import asyncio
import os

from deepgram import AsyncDeepgramClient
from server import DATA_DIR, load_env_files

CLIP_NAME = "hearing.wav"
PROMPT = "I'm Hearing now~ How can I assist you?"


async def main() -> None:
  load_env_files()
  api_key = os.getenv("DEEPGRAM_API_KEY")
  if not api_key:
    raise RuntimeError("Please set DEEPGRAM_API_KEY before running the Deepgram bridge.")

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
  print(f"WAV：{target}")


if __name__ == "__main__":
  asyncio.run(main())
