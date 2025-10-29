import asyncio
import json
import os
import sys
import time
from contextlib import suppress
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional, Callable

from aiohttp import web
from deepgram import AsyncDeepgramClient
from deepgram.core.events import EventType
from deepgram.extensions.types.sockets import (
    AgentV1Agent,
    AgentV1AgentAudioDoneEvent,
    AgentV1AgentStartedSpeakingEvent,
    AgentV1AgentThinkingEvent,
    AgentV1AudioConfig,
    AgentV1AudioInput,
    AgentV1AudioOutput,
    AgentV1ControlMessage,
    AgentV1ConversationTextEvent,
    AgentV1DeepgramSpeakProvider,
    AgentV1ErrorEvent,
    AgentV1Flags,
    AgentV1Listen,
    AgentV1ListenProvider,
    AgentV1SettingsAppliedEvent,
    AgentV1SettingsMessage,
    AgentV1SpeakProviderConfig,
    AgentV1Think,
    AgentV1WarningEvent,
    AgentV1WelcomeMessage,
    AgentV1UserStartedSpeakingEvent,
    AgentV1OpenAiThinkProvider,
)

LISTEN_HOST = os.getenv("ESP_LISTEN_HOST", "0.0.0.0")
LISTEN_PORT = int(os.getenv("ESP_LISTEN_PORT", "9000"))
INPUT_SAMPLE_RATE = 16000
OUTPUT_SAMPLE_RATE = 24000
HTTP_HOST = os.getenv("BACKEND_HOST", "0.0.0.0")
HTTP_PORT = int(os.getenv("BACKEND_PORT", "8000"))
HTTP_BASE_URL = os.getenv("HTTP_BASE_URL", f"http://192.168.137.1:{HTTP_PORT}/data")

BASE_DIR = Path(__file__).resolve().parent
DATA_DIR = BASE_DIR / "data"
DATA_DIR.mkdir(parents=True, exist_ok=True)
AGENT_OUTPUT_CLIP = "agent-output.wav"
PRESET_GREETING_CLIP = "agent-greeting.wav"
SAFE_UNMUTE_PAD = 0.05  # seconds of extra cushion before resuming microphone
PLAYBACK_FALLBACK_EXTRA = 0.2  # additional seconds before fallback kicks in
BASE_AGENT_PROMPT = (
    "You are J.A.R.V.I.S., Tony Stark's trusted AI assistant. "
    "Reply in short, natural sentences. "
    "Keep answers to one or two sentences unless the user asks for more detail. "
    "Be clear, confident, and avoid filler."
)


def iso_timestamp(ts: Optional[float] = None) -> str:
    if ts is None:
        ts = time.time()
    return (
        datetime.fromtimestamp(ts, tz=timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


class BridgeState:
    def __init__(self) -> None:
        self._writer: Optional[asyncio.StreamWriter] = None
        self._command_queue: Optional[asyncio.Queue[str]] = None
        self._writer_lock = asyncio.Lock()
        self._playback_finished_handler: Optional[Callable[[], None]] = None
        self.connected = asyncio.Event()
        self.peer: Optional[str] = None
        self._last_enqueued_monotonic: Optional[float] = None
        self._last_sent_monotonic: Optional[float] = None
        self._last_command: Optional[str] = None

    async def attach(self, writer: asyncio.StreamWriter, peer: Optional[str]) -> None:
        self._writer = writer
        self._command_queue = asyncio.Queue(maxsize=32)
        self.peer = peer
        self.connected.set()

    async def detach(self) -> None:
        self.connected.clear()
        self.peer = None
        self._writer = None
        self._command_queue = None
        self._playback_finished_handler = None

    async def enqueue_command(self, command: str) -> None:
        if not self._command_queue:
            raise RuntimeError("ESP32 bridge is not connected.")
        command = command.strip()
        await self._command_queue.put(command)
        self._last_enqueued_monotonic = time.monotonic()
        self._last_command = command
        print(f"[Bridge] Command queued: {command}")

    async def dispatch_commands(self, stop_event: asyncio.Event) -> None:
        if not self._command_queue:
            return
        try:
            while not stop_event.is_set():
                try:
                    command = await asyncio.wait_for(self._command_queue.get(), timeout=1.0)
                except asyncio.TimeoutError:
                    continue
                if not command:
                    continue
                writer = self._writer
                if not writer or writer.is_closing():
                    raise RuntimeError("ESP32 writer unavailable while dispatching commands.")
                async with self._writer_lock:
                    writer.write(f"{command}\n".encode("utf-8"))
                    try:
                        await writer.drain()
                    except ConnectionResetError as exc:
                        raise RuntimeError(f"ESP command dispatch failed: {exc}") from exc
                now = time.monotonic()
                elapsed = None
                if self._last_enqueued_monotonic is not None:
                    elapsed = now - self._last_enqueued_monotonic
                self._last_sent_monotonic = now
                if elapsed is None:
                    print(f"[Bridge] Command sent to ESP: {command}")
                else:
                    print(f"[Bridge] Command sent to ESP: {command} (dispatch delay {elapsed:.3f}s)")
        except asyncio.CancelledError:
            pass
        except Exception as exc:
            print(f"Command dispatcher stopped: {exc}")
            stop_event.set()

    def status(self) -> Dict[str, Any]:
        return {"connected": self.connected.is_set(), "peer": self.peer}

    def register_playback_finished_handler(self, handler: Optional[Callable[[], None]]) -> None:
        self._playback_finished_handler = handler

    def notify_playback_finished(self) -> None:
        if self._playback_finished_handler:
            self._playback_finished_handler()


bridge_state = BridgeState()


def load_env_files() -> None:
    for candidate in (BASE_DIR / ".env", BASE_DIR.parent / ".env"):
        if not candidate.is_file():
            continue
        with candidate.open("r", encoding="utf-8") as fh:
            for raw_line in fh:
                line = raw_line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, value = line.split("=", 1)
                key = key.strip()
                if not key or key in os.environ:
                    continue
                os.environ[key] = value.strip().strip('"').strip("'")


def ensure_api_key() -> str:
    load_env_files()
    key = os.getenv("DEEPGRAM_API_KEY")
    if not key:
        raise RuntimeError("Please set DEEPGRAM_API_KEY before running the Deepgram bridge.")
    return key


def build_settings() -> AgentV1SettingsMessage:
    return AgentV1SettingsMessage(
        audio=AgentV1AudioConfig(
            input=AgentV1AudioInput(encoding="linear16", sample_rate=INPUT_SAMPLE_RATE),
            output=AgentV1AudioOutput(encoding="linear16", sample_rate=OUTPUT_SAMPLE_RATE, container="wav"),
        ),
        agent=AgentV1Agent(
            language="en",
            listen=AgentV1Listen(
                provider=AgentV1ListenProvider(model="nova-3"),
            ),
            think=AgentV1Think(
                provider=AgentV1OpenAiThinkProvider(model="gpt-4o-mini"),
                prompt=BASE_AGENT_PROMPT,
            ),
            speak=AgentV1SpeakProviderConfig(
                provider=AgentV1DeepgramSpeakProvider(model="aura-2-hyperion-en"),
            ),
        ),
        flags=AgentV1Flags(history=True),
    )


def build_wav_header(
    data_size: int,
    sample_rate: int = OUTPUT_SAMPLE_RATE,
    bits_per_sample: int = 16,
    channels: int = 1,
) -> bytes:
    chunk_size = 36 + data_size
    byte_rate = sample_rate * channels * bits_per_sample // 8
    block_align = channels * bits_per_sample // 8
    header = bytearray(44)

    header[0:4] = b"RIFF"
    header[4:8] = chunk_size.to_bytes(4, "little")
    header[8:12] = b"WAVE"

    header[12:16] = b"fmt "
    header[16:20] = (16).to_bytes(4, "little")  # PCM
    header[20:22] = (1).to_bytes(2, "little")  # PCM format
    header[22:24] = channels.to_bytes(2, "little")
    header[24:28] = sample_rate.to_bytes(4, "little")
    header[28:32] = byte_rate.to_bytes(4, "little")
    header[32:34] = block_align.to_bytes(2, "little")
    header[34:36] = bits_per_sample.to_bytes(2, "little")

    header[36:40] = b"data"
    header[40:44] = data_size.to_bytes(4, "little")
    return bytes(header)


async def stream_esp_audio(
    reader: asyncio.StreamReader,
    socket,
    stop_event: asyncio.Event,
    mic_gate: asyncio.Event,
) -> None:
    try:
        while not stop_event.is_set():
            header = await reader.readexactly(4)
            frame_len = int.from_bytes(header, "big")
            if frame_len == 0:
                continue
            payload = await reader.readexactly(frame_len)
            if mic_gate.is_set():
                await socket.send_media(payload)
    except asyncio.IncompleteReadError:
        print("ESP stream ended.")
        stop_event.set()
    except asyncio.CancelledError:
        pass
    except ConnectionResetError as exc:
        print(f"ESP connection reset: {exc}")
        stop_event.set()


async def send_keep_alive(socket, stop_event: asyncio.Event) -> None:
    try:
        while not stop_event.is_set():
            await asyncio.sleep(5)
            await socket.send_control(AgentV1ControlMessage())
    except asyncio.CancelledError:
        pass


async def run_session(reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
    api_key = ensure_api_key()
    client = AsyncDeepgramClient(api_key=api_key)
    stop_event = asyncio.Event()
    mic_gate = asyncio.Event()
    mic_gate.set()
    audio_buffer = bytearray()
    command_dispatch_task = asyncio.create_task(bridge_state.dispatch_commands(stop_event))
    pending_unmute_task: Optional[asyncio.Task] = None

    def cancel_pending_unmute() -> None:
        nonlocal pending_unmute_task
        if pending_unmute_task and not pending_unmute_task.done():
            pending_unmute_task.cancel()
        pending_unmute_task = None

    def schedule_mic_unmute(delay: float, *, label: str | None = None) -> None:
        nonlocal pending_unmute_task
        cancel_pending_unmute()

        async def _unmute_after_delay() -> None:
            try:
                await asyncio.sleep(delay + SAFE_UNMUTE_PAD)
            except asyncio.CancelledError:
                return
            if mic_gate.is_set():
                return
            mic_gate.set()
            if label:
                print(f"Microphone stream resumed after playback (fallback: {label}).")
            else:
                print("Microphone stream resumed after playback.")

        pending_unmute_task = asyncio.create_task(_unmute_after_delay())

    def resume_microphone_immediately(reason: str) -> None:
        cancel_pending_unmute()
        if not mic_gate.is_set():
            mic_gate.set()
            print(f"Microphone stream resumed immediately ({reason}).")
        else:
            print(f"Playback-finished signal received ({reason}), microphone already open.")

    async def queue_command(command: str) -> None:
        try:
            await bridge_state.enqueue_command(command)
        except RuntimeError as err:
            print(f"Failed to queue command '{command}': {err}")

    def estimate_wav_duration(filepath: Path) -> float:
        try:
            size = filepath.stat().st_size
        except OSError as exc:
            print(f"Failed to stat {filepath}: {exc}")
            return 0.0
        if size <= 44:
            return 0.0
        payload = size - 44  # assume standard WAV header
        samples = payload // 2  # 16-bit mono
        return samples / float(OUTPUT_SAMPLE_RATE)

    greeting_played = False
    bridge_state.register_playback_finished_handler(
        lambda: resume_microphone_immediately("ESP playback finished")
    )

    async with client.agent.v1.connect() as socket:
        async def handle_message(message) -> None:
            nonlocal audio_buffer, greeting_played

            if isinstance(message, bytes):
                audio_buffer.extend(message)
            elif isinstance(message, AgentV1WelcomeMessage):
                print(f"Agent connected. Request ID: {message.request_id}")
            elif isinstance(message, AgentV1SettingsAppliedEvent):
                print("Agent settings applied.")
                if not greeting_played:
                    preset_path = DATA_DIR / PRESET_GREETING_CLIP
                    if preset_path.is_file():
                        mic_gate.clear()
                        duration = estimate_wav_duration(preset_path)
                        base_delay = duration if duration > 0 else 3.0
                        schedule_mic_unmute(base_delay, label="greeting fallback")
                        await queue_command(f"PLAY {HTTP_BASE_URL.rstrip('/')}/{PRESET_GREETING_CLIP}")
                        greeting_played = True
                    else:
                        print("Greeting clip not found, skipping preset playback.")
            elif isinstance(message, AgentV1AgentThinkingEvent):
                print("Agent is thinking...")
            elif isinstance(message, AgentV1UserStartedSpeakingEvent):
                cancel_pending_unmute()
                if not mic_gate.is_set():
                    mic_gate.set()
                    print("User started speaking. Microphone stream resumed.")
                else:
                    print("User started speaking.")
            elif isinstance(message, AgentV1AgentStartedSpeakingEvent):
                audio_buffer = bytearray()
                cancel_pending_unmute()
                if mic_gate.is_set():
                    mic_gate.clear()
                    print("Agent started speaking. Microphone stream paused.")
                else:
                    print("Agent started speaking.")
            elif isinstance(message, AgentV1ConversationTextEvent):
                speaker = "Assistant" if message.role == "assistant" else "User"
                content_text = str(message.content)
                print(f"[{speaker}] {content_text}")
            elif getattr(message, "type", "").lower() == "history":
                role = getattr(message, "role", "unknown")
                content_text = str(getattr(message, "content", ""))
                print(f"[History/{role}] {content_text}")
            elif isinstance(message, AgentV1AgentAudioDoneEvent):
                if audio_buffer:
                    filename = AGENT_OUTPUT_CLIP
                    filepath = DATA_DIR / filename
                    header = build_wav_header(len(audio_buffer))
                    with filepath.open("wb") as f:
                        f.write(header)
                        f.write(audio_buffer)
                    duration = len(audio_buffer) / (OUTPUT_SAMPLE_RATE * 2)
                    print(
                        f"Agent audio saved to {filepath} "
                        f"(payload {len(audio_buffer)} bytes, ~{duration:.2f}s)"
                    )
                    await queue_command(f"PLAY {HTTP_BASE_URL.rstrip('/')}/{filename}")
                    schedule_mic_unmute(
                        duration + PLAYBACK_FALLBACK_EXTRA, label="agent playback fallback"
                    )
                    audio_buffer = bytearray()
                else:
                    if not mic_gate.is_set():
                        mic_gate.set()
                        print("Agent audio done without payload. Microphone stream resumed.")
            elif isinstance(message, AgentV1WarningEvent):
                print(f"Agent warning: {message}")
            elif isinstance(message, AgentV1ErrorEvent):
                print(f"Agent error event: {message}")
                stop_event.set()
            else:
                print("Received event:", message)

        async def handle_error(error) -> None:
            print(f"Agent reported error: {error}")
            stop_event.set()

        async def handle_close(_data) -> None:
            print("Agent connection closed.")
            stop_event.set()

        socket.on(EventType.MESSAGE, handle_message)
        socket.on(EventType.ERROR, handle_error)
        socket.on(EventType.CLOSE, handle_close)

        listener_task = asyncio.create_task(socket.start_listening())
        keep_alive_task = asyncio.create_task(send_keep_alive(socket, stop_event))
        sender_task = asyncio.create_task(stream_esp_audio(reader, socket, stop_event, mic_gate))

        try:
            await socket.send_settings(build_settings())
            print("Agent settings sent, streaming audio...")
            await stop_event.wait()
        except KeyboardInterrupt:
            print("\nReceived stop signal, shutting down session...")
            stop_event.set()
        finally:
            bridge_state.register_playback_finished_handler(None)
            cancel_pending_unmute()
            for task in (sender_task, keep_alive_task, command_dispatch_task):
                if task:
                    task.cancel()
            stop_event.set()
            with suppress(asyncio.CancelledError):
                await sender_task
                await keep_alive_task
                await command_dispatch_task
            with suppress(asyncio.CancelledError):
                listener_task.cancel()
                await listener_task

    if not writer.is_closing():
        writer.close()
        with suppress(Exception):
            await writer.wait_closed()
    print("Session closed, resources released.")


async def handle_client(reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
    peer = writer.get_extra_info("peername")
    display_peer = None
    if isinstance(peer, tuple) and len(peer) >= 2:
        display_peer = f"{peer[0]}:{peer[1]}"
    else:
        display_peer = str(peer)

    if bridge_state.connected.is_set():
        print(f"Rejecting ESP connection from {display_peer}; another client is active.")
        writer.close()
        await writer.wait_closed()
        return

    print(f"ESP connected from {display_peer}")
    try:
        handshake = await asyncio.wait_for(reader.readline(), timeout=10)
    except asyncio.TimeoutError:
        print("Handshake timeout.")
        writer.close()
        await writer.wait_closed()
        return

    if not handshake:
        print("Received empty handshake, closing connection.")
        writer.close()
        await writer.wait_closed()
        return

    try:
        parts = handshake.decode("utf-8").strip().split()
    except UnicodeDecodeError:
        print("Failed to decode handshake line.")
        writer.close()
        await writer.wait_closed()
        return

    if len(parts) != 4 or parts[0].upper() != "PCM":
        print(f"Unexpected handshake line: {handshake!r}")
        writer.close()
        await writer.wait_closed()
        return

    sample_rate = int(parts[1])
    bits = int(parts[2])
    channels = int(parts[3])

    if sample_rate != INPUT_SAMPLE_RATE or bits != 16 or channels != 1:
        print(
            f"Warning: ESP reported PCM {sample_rate} Hz, {bits} bit, {channels} channel(s). "
            f"Expected {INPUT_SAMPLE_RATE} Hz, 16 bit, mono."
        )

    await bridge_state.attach(writer, display_peer)

    try:
        await run_session(reader, writer)
    finally:
        await bridge_state.detach()


async def handle_post_audio_play(request: web.Request) -> web.Response:
    if not bridge_state.connected.is_set():
        raise web.HTTPServiceUnavailable(text="ESP32 bridge is not connected.")

    try:
        payload = await request.json()
    except json.JSONDecodeError:
        raise web.HTTPBadRequest(text="Invalid JSON payload.")

    command = payload.get("command")
    url = payload.get("url")
    clip = payload.get("clip")

    if clip:
        clip = clip.lstrip("/")
        url = f"{HTTP_BASE_URL.rstrip('/')}/{clip}"

    if not command:
        if not url:
            raise web.HTTPBadRequest(text="Provide either 'url', 'clip', or 'command'.")
        command = f"PLAY {url}"

    try:
        await bridge_state.enqueue_command(command)
    except RuntimeError as err:
        raise web.HTTPServiceUnavailable(text=str(err)) from err

    return web.json_response({"status": "queued", "command": command})


async def handle_post_audio_playback_done(_request: web.Request) -> web.Response:
    if not bridge_state.connected.is_set():
        raise web.HTTPServiceUnavailable(text="ESP32 bridge is not connected.")
    now = time.monotonic()
    elapsed_from_send = None
    if bridge_state._last_sent_monotonic is not None:
        elapsed_from_send = now - bridge_state._last_sent_monotonic
    if elapsed_from_send is None:
        print("Playback completion notification received from ESP.")
    else:
        print(
            f"Playback completion notification received from ESP "
            f"(elapsed {elapsed_from_send:.3f}s since last command)."
        )
    bridge_state.notify_playback_finished()
    return web.json_response({"status": "ok"})


async def handle_root(_request: web.Request) -> web.StreamResponse:
    return web.Response(
        text=(
            "ESP32 audio bridge backend is running.\n"
            "Use /api/audio/play to queue clips for the ESP32."
        ),
        content_type="text/plain",
    )


def create_app() -> web.Application:
    app = web.Application()
    app.add_routes(
        [
            web.get("/", handle_root),
            web.post("/api/audio/play", handle_post_audio_play),
            web.post("/api/audio/playback_done", handle_post_audio_playback_done),
        ]
    )
    app.router.add_static("/data/", DATA_DIR, show_index=True)
    return app


async def run_server() -> None:
    app = create_app()
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, HTTP_HOST, HTTP_PORT)
    await site.start()
    print(f"HTTP API available at http://{HTTP_HOST}:{HTTP_PORT}")

    server = await asyncio.start_server(handle_client, LISTEN_HOST, LISTEN_PORT)
    addr = ", ".join(str(sock.getsockname()) for sock in server.sockets or [])
    print(f"Listening for ESP32 on {addr}")
    try:
        async with server:
            await server.serve_forever()
    except asyncio.CancelledError:
        pass
    finally:
        await runner.cleanup()


def main() -> None:
    try:
        asyncio.run(run_server())
    except RuntimeError as err:
        print(err)
        sys.exit(1)


if __name__ == "__main__":
    main()
