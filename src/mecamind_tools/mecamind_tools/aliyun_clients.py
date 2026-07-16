from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import time
from typing import Any, Callable, Dict, Iterable
from urllib import error, request

import yaml


DEFAULT_DASHSCOPE_BASE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"
LOCAL_CREDENTIALS_FILENAME = "mecamind_aliyun.local.yaml"


@dataclass(frozen=True)
class AliyunTaskPlan:
    intent: str
    target: str = ""
    requires_confirmation: bool = False
    reply: str = ""


def _env_value(name: str) -> str:
    return os.environ.get(name, "").strip()


def _credential_config_candidates() -> list[Path]:
    candidates = []
    configured_path = _env_value("MECAMIND_ALIYUN_CONFIG")
    if configured_path:
        candidates.append(Path(configured_path).expanduser())

    candidates.extend(
        [
            Path(__file__).resolve().parents[1] / "config" / LOCAL_CREDENTIALS_FILENAME,
            Path.cwd() / "config" / LOCAL_CREDENTIALS_FILENAME,
            Path("~/.config/mecamind").expanduser() / LOCAL_CREDENTIALS_FILENAME,
        ]
    )
    return list(dict.fromkeys(path.resolve() for path in candidates))


def load_api_key_from_config(
    env_name: str = "DASHSCOPE_API_KEY",
    config_file: str | Path | None = None,
) -> str:
    candidates = (
        [Path(config_file).expanduser().resolve()]
        if config_file
        else _credential_config_candidates()
    )
    for path in candidates:
        if not path.is_file():
            continue
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        if not isinstance(data, dict):
            raise RuntimeError(f"Aliyun credential config must be a mapping: {path}")
        aliyun = data.get("aliyun", {})
        if aliyun and not isinstance(aliyun, dict):
            raise RuntimeError(f"'aliyun' must be a mapping in credential config: {path}")
        value = str(
            data.get(env_name)
            or data.get("dashscope_api_key")
            or aliyun.get("api_key", "")
        ).strip()
        if value:
            return value
        raise RuntimeError(f"Aliyun API key is empty in credential config: {path}")
    return ""


def _require_api_key(env_name: str = "DASHSCOPE_API_KEY") -> str:
    value = _env_value(env_name)
    if not value:
        value = load_api_key_from_config(env_name)
        if value:
            os.environ[env_name] = value
    if not value:
        raise RuntimeError(
            f"Missing {env_name}; set it in the shell or create config/{LOCAL_CREDENTIALS_FILENAME}."
        )
    return value


def _strip_json_fence(text: str) -> str:
    clean = text.strip()
    fenced = re.search(r"```(?:json)?\s*(.*?)\s*```", clean, flags=re.DOTALL | re.IGNORECASE)
    if fenced:
        return fenced.group(1).strip()
    start = clean.find("{")
    end = clean.rfind("}")
    if start >= 0 and end > start:
        return clean[start : end + 1]
    return clean


def parse_task_plan_payload(payload: str, fallback_text: str = "") -> AliyunTaskPlan:
    data = json.loads(_strip_json_fence(payload))
    if not isinstance(data, dict):
        raise ValueError("LLM task payload must be a JSON object")
    intent = str(data.get("intent", "unknown")).strip().lower() or "unknown"
    allowed = {"stop", "cancel", "patrol", "follow", "mapping", "navigate", "unknown"}
    if intent not in allowed:
        intent = "unknown"
    return AliyunTaskPlan(
        intent=intent,
        target=str(data.get("target", "") or ""),
        requires_confirmation=bool(data.get("requires_confirmation", intent == "unknown")),
        reply=str(data.get("reply", "") or fallback_text),
    )


def build_task_parser_messages(text: str) -> list[dict[str, str]]:
    system = (
        "You are the task parser for a ROS 2 indoor mobile robot. "
        "Convert the user's command into strict JSON only. "
        "Allowed intents are stop, cancel, patrol, follow, mapping, navigate, unknown. "
        "Use target for room or waypoint names. "
        "Set requires_confirmation true when the command is ambiguous or risky. "
        "The JSON schema is: "
        '{"intent":"navigate","target":"bedroom","requires_confirmation":false,"reply":"OK"}'
    )
    return [
        {"role": "system", "content": system},
        {"role": "user", "content": text},
    ]


class AliyunLlmClient:
    def __init__(
        self,
        model: str = "qwen-plus",
        api_key_env: str = "DASHSCOPE_API_KEY",
        base_url: str = DEFAULT_DASHSCOPE_BASE_URL,
        timeout_sec: float = 15.0,
        opener: Callable[[request.Request, float], Any] | None = None,
    ) -> None:
        self.model = model
        self.api_key_env = api_key_env
        self.base_url = base_url.rstrip("/")
        self.timeout_sec = timeout_sec
        self._opener = opener or request.urlopen

    def parse_task(self, text: str) -> AliyunTaskPlan:
        api_key = _require_api_key(self.api_key_env)
        payload = {
            "model": self.model,
            "messages": build_task_parser_messages(text),
            "temperature": 0.0,
            "response_format": {"type": "json_object"},
        }
        raw = json.dumps(payload).encode("utf-8")
        req = request.Request(
            f"{self.base_url}/chat/completions",
            data=raw,
            headers={
                "Authorization": f"Bearer {api_key}",
                "Content-Type": "application/json",
            },
            method="POST",
        )
        try:
            with self._opener(req, timeout=self.timeout_sec) as resp:
                body = resp.read().decode("utf-8")
        except error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"Aliyun LLM request failed: HTTP {exc.code}: {detail}") from exc
        except error.URLError as exc:
            raise RuntimeError(f"Aliyun LLM request failed: {exc.reason}") from exc

        data = json.loads(body)
        content = data["choices"][0]["message"]["content"]
        return parse_task_plan_payload(content, fallback_text=text)


def _set_dashscope_runtime(api_key_env: str, websocket_url: str = "") -> Any:
    try:
        import dashscope  # type: ignore
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError("DashScope SDK is required for Aliyun ASR/TTS: pip install -U dashscope") from exc
    dashscope.api_key = _require_api_key(api_key_env)
    if websocket_url:
        dashscope.base_websocket_api_url = websocket_url
    return dashscope


def _audio_format_from_path(path: str | Path, fallback: str = "wav") -> str:
    suffix = Path(path).suffix.lower().lstrip(".")
    return suffix or fallback


def extract_recognition_text(sentences: Any) -> str:
    if isinstance(sentences, str):
        return sentences.strip()
    if isinstance(sentences, dict):
        text = sentences.get("text", "")
        if text:
            return str(text).strip()
        nested = sentences.get("sentences", [])
        return extract_recognition_text(nested)
    if isinstance(sentences, list):
        parts = [
            str(item.get("text", "")).strip()
            for item in sentences
            if isinstance(item, dict) and item.get("text")
        ]
        return "".join(parts)
    return str(sentences or "").strip()


class AliyunSpeechClient:
    def __init__(
        self,
        api_key_env: str = "DASHSCOPE_API_KEY",
        websocket_url: str = "",
        asr_model: str = "paraformer-realtime-v2",
        tts_model: str = "cosyvoice-v3-flash",
        tts_voice: str = "longanyang",
        sample_rate: int = 16000,
    ) -> None:
        self.api_key_env = api_key_env
        self.websocket_url = websocket_url
        self.asr_model = asr_model
        self.tts_model = tts_model
        self.tts_voice = tts_voice
        self.sample_rate = sample_rate

    def recognize_file(
        self,
        audio_path: str | Path,
        language_hints: Iterable[str] | None = ("zh", "en"),
    ) -> str:
        _set_dashscope_runtime(self.api_key_env, self.websocket_url)
        from http import HTTPStatus
        from dashscope.audio.asr import Recognition  # type: ignore

        audio_path = str(Path(audio_path).expanduser())
        recognition = Recognition(
            model=self.asr_model,
            format=_audio_format_from_path(audio_path),
            sample_rate=self.sample_rate,
            language_hints=list(language_hints or []),
            callback=None,
        )
        result = recognition.call(audio_path)
        if result.status_code != HTTPStatus.OK:
            raise RuntimeError(f"Aliyun ASR failed: {getattr(result, 'message', result)}")
        text = extract_recognition_text(result.get_sentence())
        if not text:
            raise RuntimeError("Aliyun ASR returned no recognized text")
        return text

    def synthesize_to_file(
        self,
        text: str,
        output_dir: str | Path,
        audio_format: str = "mp3",
    ) -> Path:
        _set_dashscope_runtime(self.api_key_env, self.websocket_url)
        from dashscope.audio.tts_v2 import SpeechSynthesizer  # type: ignore

        root = Path(output_dir).expanduser()
        root.mkdir(parents=True, exist_ok=True)
        safe_stamp = int(time.time() * 1000)
        path = root / f"mecamind_tts_{safe_stamp}.{audio_format.lower()}"
        synthesizer = SpeechSynthesizer(model=self.tts_model, voice=self.tts_voice)
        audio = synthesizer.call(text)
        if not audio:
            raise RuntimeError("Aliyun TTS returned empty audio")
        path.write_bytes(audio)
        return path
