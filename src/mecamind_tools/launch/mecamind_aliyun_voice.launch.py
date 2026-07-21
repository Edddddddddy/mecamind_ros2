"""语音交互链路的启动文件（阿里云 ASR/TTS，进阶课内容）。

启动录音、语音识别（ASR）、语音合成（TTS）等节点，
api_key 通过环境变量传入（默认 DASHSCOPE_API_KEY），
避免把密钥写进代码或配置文件。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    api_key_env = LaunchConfiguration("api_key_env")
    websocket_url = LaunchConfiguration("websocket_url")
    audio_file_topic = LaunchConfiguration("audio_file_topic")
    voice_command_topic = LaunchConfiguration("voice_command_topic")
    asr_result_topic = LaunchConfiguration("asr_result_topic")
    task_command_topic = LaunchConfiguration("task_command_topic")
    robot_reply_topic = LaunchConfiguration("robot_reply_topic")
    tts_audio_file_topic = LaunchConfiguration("tts_audio_file_topic")
    dashscope_base_url = LaunchConfiguration("dashscope_base_url")
    llm_model = LaunchConfiguration("llm_model")
    asr_model = LaunchConfiguration("asr_model")
    tts_model = LaunchConfiguration("tts_model")
    tts_voice = LaunchConfiguration("tts_voice")
    tts_output_dir = LaunchConfiguration("tts_output_dir")
    enable_microphone = LaunchConfiguration("enable_microphone")
    enable_task_scheduler = LaunchConfiguration("enable_task_scheduler")
    enable_tts = LaunchConfiguration("enable_tts")
    microphone_backend = LaunchConfiguration("microphone_backend")
    microphone_device = LaunchConfiguration("microphone_device")
    microphone_duration_sec = LaunchConfiguration("microphone_duration_sec")
    microphone_status_topic = LaunchConfiguration("microphone_status_topic")
    record_voice_service = LaunchConfiguration("record_voice_service")
    microphone_output_dir = LaunchConfiguration("microphone_output_dir")

    return LaunchDescription(
        [
            DeclareLaunchArgument("api_key_env", default_value="DASHSCOPE_API_KEY"),
            DeclareLaunchArgument("websocket_url", default_value=""),
            DeclareLaunchArgument("audio_file_topic", default_value="/mecamind/audio_file"),
            DeclareLaunchArgument("voice_command_topic", default_value="/mecamind/voice_command"),
            DeclareLaunchArgument("asr_result_topic", default_value="/mecamind/asr_result"),
            DeclareLaunchArgument("task_command_topic", default_value="/mecamind/task_command"),
            DeclareLaunchArgument("robot_reply_topic", default_value="/mecamind/robot_reply"),
            DeclareLaunchArgument("tts_audio_file_topic", default_value="/mecamind/tts_audio_file"),
            DeclareLaunchArgument(
                "dashscope_base_url",
                default_value="https://dashscope.aliyuncs.com/compatible-mode/v1",
            ),
            DeclareLaunchArgument("llm_model", default_value="qwen-plus"),
            DeclareLaunchArgument("asr_model", default_value="paraformer-realtime-v2"),
            DeclareLaunchArgument("tts_model", default_value="cosyvoice-v3-flash"),
            DeclareLaunchArgument("tts_voice", default_value="longanyang"),
            DeclareLaunchArgument("tts_output_dir", default_value="~/.ros/mecamind_tts"),
            DeclareLaunchArgument("enable_microphone", default_value="true"),
            DeclareLaunchArgument("enable_task_scheduler", default_value="true"),
            DeclareLaunchArgument("enable_tts", default_value="true"),
            DeclareLaunchArgument("microphone_backend", default_value="auto"),
            DeclareLaunchArgument("microphone_device", default_value="default"),
            DeclareLaunchArgument("microphone_duration_sec", default_value="4.0"),
            DeclareLaunchArgument(
                "microphone_status_topic",
                default_value="/mecamind/microphone_status",
            ),
            DeclareLaunchArgument(
                "record_voice_service",
                default_value="/mecamind/record_voice",
            ),
            DeclareLaunchArgument(
                "microphone_output_dir",
                default_value="~/.ros/mecamind_mic",
            ),
            Node(
                package="mecamind_tools",
                executable="mecamind_microphone_recorder",
                name="mecamind_microphone_recorder",
                output="screen",
                condition=IfCondition(enable_microphone),
                parameters=[
                    {"audio_file_topic": audio_file_topic},
                    {"status_topic": microphone_status_topic},
                    {"record_service": record_voice_service},
                    {"recorder_backend": microphone_backend},
                    {"device": microphone_device},
                    {
                        "duration_sec": ParameterValue(
                            microphone_duration_sec,
                            value_type=float,
                        )
                    },
                    {"output_dir": microphone_output_dir},
                ],
            ),
            Node(
                package="mecamind_tools",
                executable="mecamind_aliyun_asr_file",
                name="mecamind_aliyun_asr_file",
                output="screen",
                parameters=[
                    {"api_key_env": api_key_env},
                    {"websocket_url": websocket_url},
                    {"audio_file_topic": audio_file_topic},
                    {"voice_command_topic": voice_command_topic},
                    {"asr_result_topic": asr_result_topic},
                    {"asr_model": asr_model},
                ],
            ),
            Node(
                package="mecamind_tools",
                executable="mecamind_task_scheduler",
                name="mecamind_aliyun_task_scheduler",
                output="screen",
                condition=IfCondition(enable_task_scheduler),
                parameters=[
                    {"provider": "aliyun"},
                    {"api_key_env": api_key_env},
                    {"dashscope_base_url": dashscope_base_url},
                    {"llm_model": llm_model},
                    {"input_topic": voice_command_topic},
                    {"output_topic": task_command_topic},
                    {"reply_topic": robot_reply_topic},
                ],
            ),
            Node(
                package="mecamind_tools",
                executable="mecamind_aliyun_tts",
                name="mecamind_aliyun_tts",
                output="screen",
                condition=IfCondition(enable_tts),
                parameters=[
                    {"api_key_env": api_key_env},
                    {"websocket_url": websocket_url},
                    {"input_topic": robot_reply_topic},
                    {"output_topic": tts_audio_file_topic},
                    {"tts_model": tts_model},
                    {"tts_voice": tts_voice},
                    {"output_dir": tts_output_dir},
                ],
            ),
        ]
    )
