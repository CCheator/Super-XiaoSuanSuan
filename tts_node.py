import rclpy
from rclpy.node import Node
from service_define.srv import SetString
import threading
from pydub import AudioSegment
from pydub.playback import play
import dashscope
from dashscope.api_entities.dashscope_response import SpeechSynthesisResponse
from dashscope.audio.tts_v2 import *

import pyaudio
import sounddevice as sd
from loguru import logger
import numpy as np
import time
from std_msgs.msg import String
from datetime import datetime
import requests
from easy_tts_server import create_tts_engine
import os
# os.environ["TRANSFORMERS_OFFLINE"] = "1"
# os.environ["HF_DATASETS_OFFLINE"] = "1"



interrupt_event = threading.Event()
synthesizer = None

#获取时间戳
def get_timestamp():
    now = datetime.now()
    formatted_timestamp = now.strftime("[%Y-%m-%d %H:%M:%S.%f]")
    return formatted_timestamp

#进行句子切分的函数
def split_text(text, chunk_size=2):
    # 使用列表推导式将文本按指定长度切分
    return [text[i:i+chunk_size] for i in range(0, len(text), chunk_size)]

#tts callback
class Callback(ResultCallback):
    _player = None
    _stream = None

    def on_open(self):
        print("websocket is open.")
        self._player = pyaudio.PyAudio()
        self._stream = self._player.open(
            format=pyaudio.paInt16, channels=1, rate=22050, output=True
        )

    def on_complete(self):
        print(get_timestamp() + " speech synthesis task complete successfully.")

    def on_error(self, message: str):
        print(f"speech synthesis task failed, {message}")

    def on_close(self):
        print(get_timestamp() + " websocket is closed.")
        # 停止播放器
        self._stream.stop_stream()
        self._stream.close()
        self._player.terminate()

    def on_event(self, message):
        pass

    def on_data(self, data: bytes) -> None:
        global interrupt_event,synthesizer
        if interrupt_event.is_set():  # 检查中断信号
            print("检测到中断，终止 TTS")
            synthesizer.close() # 立即停止播放
            return
        print(get_timestamp() + " audio result length: " + str(len(data)))
        self._stream.write(data)


class TextToSpeechService(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.get_logger().info("Text-to-Speech service is initializing...")
        
        # 初始化参数和组件
        self.initialize_node()

    def initialize_node(self):
        """自动初始化节点所有组件"""
        self.get_logger().info("Configuring TTS node...")
        
        self.callback = Callback()
        self.model = "cosyvoice-v1"
        self.voice = "longxiaochun"
        
        # 创建服务
        # 等待的tts服务
        self.srv_wait = self.create_service(SetString, 'tts_service_wait', self.tts_callback_wait)
        # 处理tts服务
        self.srv = self.create_service(SetString, 'tts_service', self.tts_callback)
        # 发送说话内容
        self.tts_publisher = self.create_publisher(String, "tts_topic", 10)
        # 负责打断tts
        self.subscription = self.create_subscription(String, 'speech_interrupt', self.interrupt_topic_callback, 10)
        # 用于发布tts结束与否的信息
        self.tts_life_publisher = self.create_publisher(String, 'tts_life', 10)
        
        # 当前是否在说话
        self.state = False
        self.tts = create_tts_engine()
        
        self.get_logger().info("Text-to-Speech service is ready and activated!")

    # 直接播放语音
    def speak(self, text: str):
        print('start_speak')
        try:
            for audio_chunk in self.tts.tts_stream(text, 'zh'):
                print('audio')
                if interrupt_event.is_set():
                    break
                sd.play(audio_chunk, self.tts.sample_rate)
                while not interrupt_event.is_set() and sd.get_stream().active:
                    time.sleep(0.1)
                sd.stop()
            
            self.get_logger().info('generate finish')
            self.state = False
            self.tts_life_publisher.publish(String(data='end'))
            try:
                res = requests.post("http://localhost:8001/expression/neutral")
            except:
                self.get_logger().info('no emotion server')

            return True
        finally:
            if hasattr(self, 'synthesizer') and synthesizer:
                synthesizer.close()

    def tts_callback_wait(self, request, response):
        while self.state:
            time.sleep(0.5)
        self.get_logger().info(f"Received text: {request.data}")
        interrupt_event.clear()
        self.tts_life_publisher.publish(String(data='start'))
        self.tts_publisher.publish(String(data=request.data))
        self.speak(request.data)
        response.success = True
        return response

    def tts_callback(self, request, response):
        """
        这个回调函数接收一个字符串，并调用文本到语音的操作。
        返回一个布尔值，表示操作是否成功。
        """
        self.get_logger().info(f"Received text: {request.data}")
        
        self.state = True
        self.tts_life_publisher.publish(String(data='start'))
        self.tts_publisher.publish(String(data=request.data))
        # 假设这个函数处理文本并返回一个成功或失败的标志
        interrupt_event.clear()
        self.speech_thread = threading.Thread(target=self.speak, args=(request.data,))
        self.speech_thread.start()
        
        # 返回True表示成功，False表示失败
        response.success = True
        return response

    def interrupt_topic_callback(self, msg):
        if not self.state:
            return
        self.get_logger().info("Interrupt signal received via topic.")
        interrupt_event.set()
        self.state = False

    def destroy_node(self):
        """重写销毁节点方法，确保资源清理"""
        self.get_logger().info("Cleaning up TTS node resources...")
        interrupt_event.set()
        
        # 等待语音线程结束
        if hasattr(self, 'speech_thread') and self.speech_thread.is_alive():
            self.speech_thread.join(timeout=1.0)
            
        self.get_logger().info("TTS node resources cleaned up.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    tts_service = TextToSpeechService()
    
    try:
        # 使用标准的spin方式，而不是spin_once循环
        rclpy.spin(tts_service)
    except KeyboardInterrupt:
        tts_service.get_logger().info("Keyboard Interrupt. Shutting down TTS node...")
    finally:
        tts_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
