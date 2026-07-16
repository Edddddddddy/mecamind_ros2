from __future__ import annotations

import json
import os
import shutil
import subprocess
import time
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .aliyun_clients import AliyunSpeechClient


def parse_audio_file_payload(payload: str) -> str:
    clean = payload.strip()
    if not clean:
        raise ValueError("empty audio path payload")
    if clean.startswith("{"):
        data: Any = json.loads(clean)
        if not isinstance(data, dict):
            raise ValueError("audio path JSON payload must be an object")
        clean = str(data.get("path", data.get("audio_path", ""))).strip()
    if not clean:
        raise ValueError("audio path payload has no path")
    return clean


def select_microphone_backend(
    requested: str,
    environment: Mapping[str, str] | None = None,
    which: Callable[[str], str | None] = shutil.which,
) -> tuple[str, str]:
    normalized = requested.strip().lower()
    if normalized not in {"auto", "arecord", "ffmpeg_pulse"}:
        raise ValueError("recorder_backend must be auto, arecord, or ffmpeg_pulse")

    if normalized == "arecord":
        executable = which("arecord")
        if not executable:
            raise RuntimeError("arecord not found; install alsa-utils")
        return normalized, executable
    if normalized == "ffmpeg_pulse":
        executable = which("ffmpeg")
        if not executable:
            raise RuntimeError("ffmpeg not found")
        return normalized, executable

    env = environment if environment is not None else os.environ
    ffmpeg = which("ffmpeg")
    if env.get("PULSE_SERVER") and ffmpeg:
        return "ffmpeg_pulse", ffmpeg
    arecord = which("arecord")
    if arecord:
        return "arecord", arecord
    if ffmpeg:
        return "ffmpeg_pulse", ffmpeg
    raise RuntimeError("no microphone recorder found; install alsa-utils or ffmpeg")


def build_microphone_record_command(
    backend: str,
    executable: str,
    output_path: str | Path,
    device: str,
    duration_sec: float,
    sample_rate: int,
    channels: int,
) -> list[str]:
    if duration_sec <= 0.0:
        raise ValueError("duration_sec must be positive")
    if sample_rate <= 0:
        raise ValueError("sample_rate must be positive")
    if channels <= 0:
        raise ValueError("channels must be positive")

    path = str(Path(output_path).expanduser())
    if backend == "arecord":
        command = [
            executable,
            "-q",
            "-t",
            "wav",
            "-f",
            "S16_LE",
            "-r",
            str(sample_rate),
            "-c",
            str(channels),
            "-d",
            str(max(1, int(round(duration_sec)))),
        ]
        if device:
            command.extend(["-D", device])
        command.append(path)
        return command
    if backend == "ffmpeg_pulse":
        return [
            executable,
            "-nostdin",
            "-loglevel",
            "error",
            "-y",
            "-f",
            "pulse",
            "-i",
            device or "default",
            "-t",
            f"{duration_sec:.3f}",
            "-ac",
            str(channels),
            "-ar",
            str(sample_rate),
            "-c:a",
            "pcm_s16le",
            path,
        ]
    raise ValueError(f"unsupported recorder backend: {backend}")


def record_microphone_clip(
    command: Sequence[str],
    output_path: str | Path,
    timeout_sec: float,
    runner: Callable[..., subprocess.CompletedProcess] = subprocess.run,
) -> Path:
    path = Path(output_path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    runner(
        list(command),
        check=True,
        timeout=timeout_sec,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    if not path.is_file() or path.stat().st_size <= 44:
        raise RuntimeError(f"microphone recorder produced no usable WAV data: {path}")
    return path


class MicrophoneRecorderNode(Node):
    """Record a push-to-talk WAV clip and feed the existing file ASR topic."""

    def __init__(self) -> None:
        super().__init__("mecamind_microphone_recorder")
        self.declare_parameter("audio_file_topic", "/mecamind/audio_file")
        self.declare_parameter("status_topic", "/mecamind/microphone_status")
        self.declare_parameter("record_service", "/mecamind/record_voice")
        self.declare_parameter("recorder_backend", "auto")
        self.declare_parameter("device", "default")
        self.declare_parameter("duration_sec", 4.0)
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("channels", 1)
        self.declare_parameter("output_dir", str(Path("~/.ros/mecamind_mic").expanduser()))

        self.audio_pub = self.create_publisher(
            String, str(self.get_parameter("audio_file_topic").value), 10
        )
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.create_service(
            Trigger,
            str(self.get_parameter("record_service").value),
            self._record_cb,
        )
        self._recording = False
        self.get_logger().info(
            "MecaMind microphone ready; call the record_voice service and speak after recording starts"
        )

    def _publish_status(self, state: str, detail: str = "", path: str = "") -> None:
        message = String()
        message.data = json.dumps(
            {"state": state, "detail": detail, "path": path},
            ensure_ascii=False,
        )
        self.status_pub.publish(message)

    def _record_cb(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self._recording:
            response.success = False
            response.message = "microphone recording already in progress"
            return response

        self._recording = True
        output_path = (
            Path(str(self.get_parameter("output_dir").value)).expanduser()
            / f"mecamind_mic_{int(time.time() * 1000)}.wav"
        )
        try:
            backend, executable = select_microphone_backend(
                str(self.get_parameter("recorder_backend").value)
            )
            duration_sec = float(self.get_parameter("duration_sec").value)
            command = build_microphone_record_command(
                backend=backend,
                executable=executable,
                output_path=output_path,
                device=str(self.get_parameter("device").value),
                duration_sec=duration_sec,
                sample_rate=int(self.get_parameter("sample_rate").value),
                channels=int(self.get_parameter("channels").value),
            )
            self._publish_status("recording", f"backend={backend}", str(output_path))
            recorded_path = record_microphone_clip(
                command,
                output_path,
                timeout_sec=max(5.0, duration_sec + 5.0),
            )
            audio = String()
            audio.data = str(recorded_path)
            self.audio_pub.publish(audio)
            self._publish_status("recorded", f"backend={backend}", str(recorded_path))
            response.success = True
            response.message = str(recorded_path)
        except Exception as exc:  # noqa: BLE001
            detail = str(exc)
            self.get_logger().error(f"Microphone recording failed: {detail}")
            self._publish_status("error", detail, str(output_path))
            response.success = False
            response.message = detail
        finally:
            self._recording = False
        return response


class AliyunAsrFileNode(Node):
    def __init__(self) -> None:
        super().__init__("mecamind_aliyun_asr_file")
        self.declare_parameter("audio_file_topic", "/mecamind/audio_file")
        self.declare_parameter("voice_command_topic", "/mecamind/voice_command")
        self.declare_parameter("asr_result_topic", "/mecamind/asr_result")
        self.declare_parameter("api_key_env", "DASHSCOPE_API_KEY")
        self.declare_parameter("websocket_url", "")
        self.declare_parameter("asr_model", "paraformer-realtime-v2")
        self.declare_parameter("sample_rate", 16000)

        self.client = AliyunSpeechClient(
            api_key_env=str(self.get_parameter("api_key_env").value),
            websocket_url=str(self.get_parameter("websocket_url").value),
            asr_model=str(self.get_parameter("asr_model").value),
            sample_rate=int(self.get_parameter("sample_rate").value),
        )
        self.create_subscription(
            String,
            str(self.get_parameter("audio_file_topic").value),
            self._audio_cb,
            10,
        )
        self.voice_pub = self.create_publisher(
            String,
            str(self.get_parameter("voice_command_topic").value),
            10,
        )
        self.result_pub = self.create_publisher(
            String,
            str(self.get_parameter("asr_result_topic").value),
            10,
        )
        self.get_logger().info("MecaMind Aliyun ASR file bridge ready")

    def _audio_cb(self, msg: String) -> None:
        try:
            audio_path = parse_audio_file_payload(msg.data)
            text = self.client.recognize_file(audio_path)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Aliyun ASR failed: {exc}")
            return

        result = String()
        result.data = json.dumps({"text": text, "source": "aliyun_asr"}, ensure_ascii=False)
        self.result_pub.publish(result)

        command = String()
        command.data = text
        self.voice_pub.publish(command)


class AliyunTtsNode(Node):
    def __init__(self) -> None:
        super().__init__("mecamind_aliyun_tts")
        self.declare_parameter("input_topic", "/mecamind/robot_reply")
        self.declare_parameter("output_topic", "/mecamind/tts_audio_file")
        self.declare_parameter("api_key_env", "DASHSCOPE_API_KEY")
        self.declare_parameter("websocket_url", "")
        self.declare_parameter("tts_model", "cosyvoice-v3-flash")
        self.declare_parameter("tts_voice", "longanyang")
        self.declare_parameter("output_dir", str(Path("~/.ros/mecamind_tts").expanduser()))
        self.declare_parameter("audio_format", "mp3")

        self.client = AliyunSpeechClient(
            api_key_env=str(self.get_parameter("api_key_env").value),
            websocket_url=str(self.get_parameter("websocket_url").value),
            tts_model=str(self.get_parameter("tts_model").value),
            tts_voice=str(self.get_parameter("tts_voice").value),
        )
        self.create_subscription(
            String,
            str(self.get_parameter("input_topic").value),
            self._text_cb,
            10,
        )
        self.pub = self.create_publisher(String, str(self.get_parameter("output_topic").value), 10)
        self.get_logger().info("MecaMind Aliyun TTS bridge ready")

    def _text_cb(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        try:
            path = self.client.synthesize_to_file(
                text,
                str(self.get_parameter("output_dir").value),
                audio_format=str(self.get_parameter("audio_format").value),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Aliyun TTS failed: {exc}")
            return
        output = String()
        output.data = str(path)
        self.pub.publish(output)


def asr_main(args=None) -> None:
    rclpy.init(args=args)
    node = AliyunAsrFileNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def microphone_main(args=None) -> None:
    rclpy.init(args=args)
    node = MicrophoneRecorderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def tts_main(args=None) -> None:
    rclpy.init(args=args)
    node = AliyunTtsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    asr_main()
