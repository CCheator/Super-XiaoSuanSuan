#!/usr/bin/env python3

import sys
import os
import time
import numpy as np
import pyaudio
from collections import deque
from typing import Optional, Tuple
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String # For publishing ASR results
from dotenv import load_dotenv


try:
    from easy_asr_server import EasyASR, get_available_pipelines, get_default_pipeline
    from ten_vad import TenVad
except ImportError as e:
    print(f"ERROR: Failed to import ASR/VAD modules. Make sure easy_asr_server and ten_vad are installed and paths are correct.")
    print(f"Details: {e}")
    sys.exit(1)

class CombinedVADASRNode(Node):
    """
    Combined VAD and ASR system as a ROS 2 Node.
    Automatically detects speech activity and performs ASR when speech is detected,
    publishing results to '/asr_result' topic.
    """
    
    def __init__(self, node_name='asr_local_node'):
        super().__init__(node_name)
        self.get_logger().info(f"Initializing {node_name} node...")

        # Declare ROS 2 parameters
        self.declare_parameter('pipeline', 'paraformer')
        self.declare_parameter('device', 'auto')
        self.declare_parameter('hotwords', '算算你好 算算再见 算算醒醒 算算睡会')
        self.declare_parameter('vad_chunk_size', 512)
        self.declare_parameter('vad_threshold', 0.5)
        self.declare_parameter('vad_buffer_size', 4)
        self.declare_parameter('pre_speech_buffer_duration', 0.5)
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('channels', 1)

        # Component instances
        self.asr_engine: Optional[EasyASR] = None
        self.ten_vad_instance: Optional[TenVad] = None
        self.audio_stream: Optional[pyaudio.Stream] = None
        self.p_audio: Optional[pyaudio.PyAudio] = None
        self.asr_result_publisher: Optional[rclpy.publisher.Publisher] = None

        # Audio and VAD state
        self.is_recording = False
        self.audio_data = [] # Stores chunks as float32 numpy arrays
        self.speak_buffer = deque()
        self.nospeak_buffer = deque()
        self.vad_state = 0 # 0 = silence, 1 = speaking
        self.pre_speech_buffer = deque()

        # Threading control
        self._is_active = False
        self._audio_thread = None
        self._audio_thread_running = False
        self._audio_thread_lock = threading.Lock()

        # Initialize node components automatically
        self.initialize_node()

    def initialize_node(self):
        """
        Initialize all node components automatically.
        """
        self.get_logger().info("Auto-initializing node components...")
        
        # Get parameters
        self.pipeline = self.get_parameter('pipeline').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.hotwords = self.get_parameter('hotwords').get_parameter_value().string_value
        print(self.hotwords)
        self.vad_chunk_size = self.get_parameter('vad_chunk_size').get_parameter_value().integer_value
        self.vad_threshold = self.get_parameter('vad_threshold').get_parameter_value().double_value
        self.vad_buffer_size = self.get_parameter('vad_buffer_size').get_parameter_value().integer_value
        self.pre_speech_buffer_duration = self.get_parameter('pre_speech_buffer_duration').get_parameter_value().double_value
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.channels = self.get_parameter('channels').get_parameter_value().integer_value

        self.pre_speech_buffer_maxlen = int(self.pre_speech_buffer_duration * self.sample_rate / self.vad_chunk_size)
        self.pre_speech_buffer = deque(maxlen=self.pre_speech_buffer_maxlen)
        self.speak_buffer = deque(maxlen=self.vad_buffer_size)
        self.nospeak_buffer = deque(maxlen=self.vad_buffer_size)

        self.get_logger().info(f"ASR Pipeline: {self.pipeline}, Device: {self.device}, Hotwords: '{self.hotwords}'")
        self.get_logger().info(f"VAD Chunk Size: {self.vad_chunk_size}, Threshold: {self.vad_threshold}, Buffer Size: {self.vad_buffer_size}")
        self.get_logger().info(f"Pre-speech Buffer Duration: {self.pre_speech_buffer_duration}s ({self.pre_speech_buffer_maxlen} chunks)")

        # Initialize VAD
        try:
            self.get_logger().info("Initializing VAD...")
            self.ten_vad_instance = TenVad(self.vad_chunk_size, self.vad_threshold)
            self.get_logger().info("VAD initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize VAD: {e}")
            self.destroy_node()
            return

        # Initialize ASR
        try:
            self.get_logger().info("Initializing ASR components...")
            self.asr_engine = EasyASR(
                pipeline=self.pipeline,
                device=self.device,
                hotwords=self.hotwords,
                log_level="INFO",
                auto_init=True
            )
            
            if self.asr_engine.is_healthy():
                info = self.asr_engine.get_info()
                self.get_logger().info(f"ASR device: {info['resolved_device']}, Pipeline loaded: {info['pipeline']}")
                self.get_logger().info("ASR engine is healthy and ready.")
            else:
                self.get_logger().error("ASR engine health check failed!")
                self.destroy_node()
                return
                
        except Exception as e:
            self.get_logger().error(f"Failed to initialize ASR: {e}")
            self.destroy_node()
            return

        # Create ASR result publisher
        self.asr_result_publisher = self.create_publisher(String, '/asr_result', 10)
        self.get_logger().info("Publisher '/asr_result' created.")

        # Initialize PyAudio and start audio processing
        try:
            self.p_audio = pyaudio.PyAudio()
            self.audio_stream = self.p_audio.open(
                format=pyaudio.paInt16,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.vad_chunk_size
            )
            self.get_logger().info("Audio stream opened successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to open audio stream: {e}")
            self.destroy_node()
            return

        # Start the audio processing thread
        self.rms_threshold = int(os.getenv('rms_threshold','0'))
        self._is_active = True
        self._audio_thread_running = True
        self._audio_thread = threading.Thread(target=self._audio_thread_func)
        self._audio_thread.start()
        
        self.get_logger().info("Node fully initialized and started ASR listening thread.")

    def _audio_thread_func(self):
        """Thread function for audio processing loop"""
        self.get_logger().info("Audio processing thread started.")
        
        while self._audio_thread_running and rclpy.ok():
            try:
                # Read audio chunk for VAD and buffering
                raw_data = self.audio_stream.read(self.vad_chunk_size, exception_on_overflow=False)
                audio_chunk_int16 = np.frombuffer(raw_data, dtype=np.int16)
                rms = np.mean(np.square(audio_chunk_int16-np.mean(audio_chunk_int16)))
                if rms < self.rms_threshold:
                    audio_chunk_int16 = np.zeros_like(audio_chunk_int16).tobytes()
                
                # Add current chunk to pre-speech buffer
                self.pre_speech_buffer.append(audio_chunk_int16)

                # Process with VAD
                speech_started, speech_ended = self.process_audio_chunk(audio_chunk_int16)
                
                # Handle state changes
                if speech_started and not self.is_recording:
                    self.start_recording()
                
                # If we're recording, also add this chunk to the recording buffer
                if self.is_recording:
                    float_chunk = audio_chunk_int16.astype(np.float32) / 32768.0
                    self.audio_data.append(float_chunk)
                
                if speech_ended and self.is_recording:
                    # Stop recording and recognize
                    audio_array = self.stop_recording()
                    
                    if audio_array is not None:
                        result = self.recognize_audio(audio_array)
                        if result:
                            self.get_logger().info(f"Recognition Result: {result}")
                            # Publish the result
                            msg = String()
                            msg.data = result
                            if self.asr_result_publisher:
                                self.asr_result_publisher.publish(msg)
                                self.get_logger().info(f"Published ASR result: '{result}'")
                            else:
                                self.get_logger().warn("ASR result publisher is not active.")
                        else:
                            self.get_logger().info("No transcription result")

            except Exception as e:
                self.get_logger().error(f"Error in audio processing loop: {e}")
                # Potentially try to recover or transition to error state if critical

    def destroy_resources(self):
        """
        Clean up all resources before node destruction.
        """
        self.get_logger().info("Cleaning up node resources...")
        
        # Signal the audio thread to stop
        self._audio_thread_running = False
        
        # Wait for the audio thread to finish
        if self._audio_thread is not None:
            self._audio_thread.join(timeout=1.0)
            if self._audio_thread.is_alive():
                self.get_logger().warn("Audio thread did not stop gracefully")
            self._audio_thread = None

        # Stop recording if active
        if self.is_recording:
            self.stop_recording()

        # Close audio stream and PyAudio
        if self.audio_stream:
            self.audio_stream.stop_stream()
            self.audio_stream.close()
            self.audio_stream = None
            self.get_logger().info("Audio stream closed.")
            
        if self.p_audio:
            self.p_audio.terminate()
            self.p_audio = None
            self.get_logger().info("PyAudio system terminated.")

        # Release ASR engine resources
        if self.asr_engine:
            self.asr_engine = None
            self.get_logger().info("ASR engine released.")
        
        # Release VAD instance
        if self.ten_vad_instance:
            self.ten_vad_instance = None
            self.get_logger().info("VAD instance released.")

        self.get_logger().info("All resources cleaned up.")

    def start_recording(self):
        """Start audio recording, including pre-speech buffer."""
        if self.is_recording:
            self.get_logger().warn("Already recording!")
            return
        
        self.is_recording = True
        
        # Initialize audio_data with contents of the pre-speech buffer
        self.audio_data = [chunk.astype(np.float32) / 32768.0 for chunk in self.pre_speech_buffer]
        self.get_logger().info(f"Recording started (speech detected), including {len(self.audio_data)} pre-speech chunks.")
    
    def stop_recording(self) -> Optional[np.ndarray]:
        """Stop audio recording and return recorded audio."""
        if not self.is_recording:
            self.get_logger().warn("Not recording!")
            return None
        
        self.is_recording = False
        
        if not self.audio_data:
            self.get_logger().warn("No audio data recorded!")
            return None
        
        # Concatenate all audio chunks
        audio_array = np.concatenate(self.audio_data, axis=0)
        duration = len(audio_array) / self.sample_rate
        
        self.get_logger().info(f"Recording stopped (silence detected). Duration: {duration:.2f}s")
        
        # Ensure minimum length
        if len(audio_array) < 512: # Minimum samples for ASR
            self.get_logger().warn("Recording too short (minimum 512 samples required), discarding.")
            return None
        
        return audio_array.astype(np.float32)
            
    def recognize_audio(self, audio_array: np.ndarray) -> Optional[str]:
        """Recognize speech from audio array."""
        if self.asr_engine is None:
            self.get_logger().error("ASR engine not initialized!")
            return None
        
        try:
            self.get_logger().debug("Recognizing speech...")
            start_time = time.time()
            
            result = self.asr_engine.recognize(audio_array)
            
            end_time = time.time()
            processing_time = end_time - start_time
            
            self.get_logger().debug(f"Recognition completed in {processing_time:.2f}s")
            return result
            
        except Exception as e:
            self.get_logger().error(f"Recognition failed: {e}")
            return None
    
    def process_audio_chunk(self, audio_data_int16: np.ndarray) -> Tuple[bool, bool]:
        """
        Process an audio chunk with VAD.
        Returns tuple of (speech_started, speech_ended)
        """
        out_probability, out_flag = self.ten_vad_instance.process(audio_data_int16)
        
        if out_flag == 1: # Speech detected
            self.speak_buffer.append(1)
            self.nospeak_buffer.clear()
        else: # Silence detected
            self.nospeak_buffer.append(0)
            self.speak_buffer.clear()
        
        speech_started = False
        speech_ended = False
        
        if self.vad_state == 0 and len(self.speak_buffer) == self.vad_buffer_size:
            self.get_logger().info('Speech started')
            self.vad_state = 1
            speech_started = True
        
        if self.vad_state == 1 and len(self.nospeak_buffer) == self.vad_buffer_size:
            self.get_logger().info('Speech ended')
            self.vad_state = 0
            speech_ended = True
            
        return speech_started, speech_ended

    def destroy_node(self):
        """
        Override destroy_node to ensure proper cleanup.
        """
        self.destroy_resources()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = CombinedVADASRNode()
    
    # Use the default single-threaded executor for a simple node.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt. Shutting down...")
    finally:
        # Clean up and shutdown
        node.get_logger().info("Destroying node and shutting down rclpy.")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    load_dotenv()
    main()
