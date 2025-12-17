import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from service_define.srv import SetString

import os
from openai import OpenAI

# =====================
# LLM API (示例实现)
# =====================

def call_llm_api(user_text: str) -> str:
    """
    调用大模型 API，根据输入文本返回回复。
    请在此替换为你真实使用的大模型 SDK / HTTP API。
    """
    client = OpenAI(
        api_key="sk-24b144b67ddf485aaf50335fbbe5c02e",
        base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
    )
    completion = client.chat.completions.create(
        model="qwen3-max",
        messages=[
            {"role": "system", "content": "你是小算算，是一个能够进行语言交流的助手。根据用户的说的话进行回复，回复要尽量简洁，并且回复里不要出现语言文字和标点之外的符号。"},
            {"role": "user", "content": user_text},
        ],
        stream=True
    )
    reply = ""
    for chunk in completion:
        print(chunk.choices[0].delta.content, end="", flush=True)
        reply += chunk.choices[0].delta.content
    return reply


class TalkNode(Node):
    def __init__(self):
        super().__init__('talk_node')

        # ---- parameters ----
        self.wake_word = '你好'
        self.standby_timeout = 120.0

        # ---- state ----
        self.active = False
        self.last_active_time: float = time.time()
        self.lock = threading.Lock()
        self.tts_speaking = False

        # ---- ROS interfaces ----
        self.asr_sub = self.create_subscription(
            String,
            '/asr_result',
            self.asr_callback,
            10
        )
        self.tts_life_sub = self.create_subscription(
            String,
            'tts_life',
            self.tts_life_callback,
            10
        )
        self.face_sub = self.create_subscription(
            String,
            '/face_recognition_result',
            self.face_callback,
            10
        )

        self.tts_client = self.create_client(SetString, 'tts_service_wait')
        self.get_logger().info('Waiting for tts_service_wait...')
        self.tts_client.wait_for_service()
        self.get_logger().info('TTS service connected.')

        # ---- timer for standby check ----
        self.timer = self.create_timer(1.0, self.check_timeout)

        self.get_logger().info('Talk node initialized. Entering standby state.')

    # =====================
    # ASR callback
    # =====================
    def asr_callback(self, msg: String):
        if self.tts_speaking:
            return 

        text = msg.data.strip()
        self.tts_speaking = True
        if not text:
            return

        self.get_logger().info(f"ASR heard: {text}")

        with self.lock:
            if self.wake_word in text and self.active == False:
                self.activate()
                return

            if self.active == False:
                return

            self.last_active_time = time.time()

        reply = call_llm_api(text)
        self.speak(reply)

    # =====================
    # State management
    # =====================
    def activate(self, greeting="你好"):
        """Activate dialogue mode and greet."""
        if not self.active:
            self.get_logger().info('Wake word detected. Activating dialogue.')
        else:
            self.get_logger().info('Wake word detected. Already active, resetting timer.')

        self.active = True
        self.last_active_time = time.time()
        self.speak(greeting)

    def deactivate(self):
        """Return to standby mode."""
        if self.active:
            self.get_logger().info('No interaction for 2 minutes. Entering standby mode.')
        self.active = False

    def check_timeout(self):
        """Timer callback to check standby timeout."""
        with self.lock:
            if not self.active:
                return
            if time.time() - self.last_active_time > self.standby_timeout:
                self.deactivate()

    # =====================
    # TTS
    # =====================
    def tts_life_callback(self, msg: String):
        if msg.data == 'end':
            self.get_logger().info('TTS ended, ASR enabled')
            self.tts_speaking = False

    def tts_done_callback(self, future):
        try:
            res = future.result()
            if not res.success:
                self.get_logger().error("TTS failed")
        except Exception as e:
            self.get_logger().error(f"TTS exception: {e}")

    def speak(self, text: str):
        self.get_logger().info(f"TTS speak: {text}")
        req = SetString.Request()
        req.data = text

        future = self.tts_client.call_async(req)
        future.add_done_callback(self.tts_done_callback)

    # =====================
    # Face Recognize
    # =====================
    def face_callback(self, msg: String):
        if self.tts_speaking:
            return

        recognized_person = msg.data.strip()
        self.get_logger().info(f"Recognized face: {recognized_person}")
        print((f"Recognized face: {recognized_person}"))

        with self.lock:
            # 如果已激活，重置计时器即可
            if self.active:
                self.last_active_time = time.time()
                return

            # 未激活状态下，通过人脸唤醒
            self.activate(greeting=f"你好，{recognized_person}")


# =====================
# main
# =====================

def main(args=None):
    rclpy.init(args=args)
    node = TalkNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
