---
id: whisper-voice-control
title: Voice Intelligence with Whisper
sidebar_label: Chapter 16 Whisper Voice Control
---

# Voice Intelligence with Whisper

## Overview

This chapter covers the integration of OpenAI Whisper for real-time voice-to-text capabilities in the humanoid robot system. We'll explore both noise reduction techniques and compare local vs. cloud-based Speech-to-Text approaches.

## Installing OpenAI Whisper

First, let's install the OpenAI Whisper library and its dependencies:

```bash
pip install openai-whisper
```

You may also need to install additional dependencies for audio processing:

```bash
pip install torch torchaudio
```

For audio recording capabilities, install PyAudio:

```bash
pip install pyaudio
```

## Basic Whisper Integration with ROS 2

Now let's create a ROS 2 node that captures audio from a microphone and processes it with Whisper:

```python
import rclpy
from rclpy.node import Node
import whisper
import pyaudio
import numpy as np
import threading
import queue
from std_msgs.msg import String

class WhisperVoiceNode(Node):
    def __init__(self):
        super().__init__('whisper_voice_node')

        # Create publisher for voice commands
        self.voice_publisher = self.create_publisher(String, 'voice_commands', 10)

        # Load Whisper model
        self.model = whisper.load_model("base")  # You can use "tiny", "base", "small", "medium", "large"

        # Audio parameters
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000  # Whisper works best at 16kHz

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()

        # Create audio stream
        self.stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        # Start audio processing thread
        self.audio_queue = queue.Queue()
        self.processing_thread = threading.Thread(target=self.process_audio)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        self.get_logger().info("Whisper Voice Node initialized")

    def process_audio(self):
        while rclpy.ok():
            # Read audio data
            data = self.stream.read(self.chunk)
            audio_array = np.frombuffer(data, dtype=np.int16)

            # Normalize audio
            audio_array = audio_array.astype(np.float32) / 32768.0

            # Add to queue for processing
            self.audio_queue.put(audio_array)

            # Process accumulated audio if queue has enough data
            if self.audio_queue.qsize() > 10:  # Process every 10 chunks (about 0.6 seconds)
                self.transcribe_audio()

    def transcribe_audio(self):
        # Collect audio data from queue
        audio_data = []
        while not self.audio_queue.empty():
            audio_data.append(self.audio_queue.get())

        if len(audio_data) > 0:
            # Concatenate audio data
            full_audio = np.concatenate(audio_data)

            # Transcribe with Whisper
            result = self.model.transcribe(full_audio)
            text = result["text"].strip()

            if text:  # Only publish if there's actual text
                msg = String()
                msg.data = text
                self.voice_publisher.publish(msg)
                self.get_logger().info(f"Published voice command: {text}")

    def destroy_node(self):
        # Clean up audio resources
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WhisperVoiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Creating the ROS 2 Package

Create a new ROS 2 package for the voice control functionality:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python whisper_voice_control
cd whisper_voice_control
```

Update the `setup.py` file to include the executable:

```python
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'whisper_voice_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Voice control using OpenAI Whisper',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'whisper_node = whisper_voice_control.whisper_node:main',
        ],
    },
)
```

## Bridge Architecture

The voice command bridge works as follows:

1. **Audio Capture**: PyAudio captures real-time audio from the microphone
2. **Audio Processing**: Audio is normalized and buffered for processing
3. **Transcription**: Whisper processes the audio buffer and generates text
4. **ROS 2 Integration**: Transcribed text is published to the `/voice_commands` topic
5. **Command Processing**: Other ROS 2 nodes can subscribe to `/voice_commands` to execute voice-activated actions

## Testing the Voice Command Bridge

You can test the voice command bridge by running the node and listening to the topic:

```bash
# Terminal 1: Start the whisper node
ros2 run whisper_voice_control whisper_node

# Terminal 2: Listen to voice commands
ros2 topic echo /voice_commands std_msgs/msg/String
```

## Performance Considerations

- **Model Selection**: Smaller models (tiny, base) are faster but less accurate; larger models are more accurate but slower
- **Audio Buffering**: Balance between responsiveness and accuracy by adjusting buffer size
- **Noise Reduction**: Consider adding preprocessing steps for noise reduction in noisy environments
- **Real-time Constraints**: For real-time applications, consider using faster models or optimizing the audio processing pipeline

## Performance Considerations

- **Model Selection**: Smaller models (tiny, base) are faster but less accurate; larger models are more accurate but slower
- **Audio Buffering**: Balance between responsiveness and accuracy by adjusting buffer size
- **Noise Reduction**: Consider adding preprocessing steps for noise reduction in noisy environments
- **Real-time Constraints**: For real-time applications, consider using faster models or optimizing the audio processing pipeline

## Safety and Validation

When implementing voice command systems, it's important to include validation and safety checks:

```python
def validate_voice_command(self, command_text):
    """Validate voice commands before processing"""
    # Filter out potentially harmful commands
    forbidden_keywords = ["shutdown", "emergency", "kill", "stop all"]

    for keyword in forbidden_keywords:
        if keyword.lower() in command_text.lower():
            self.get_logger().warning(f"Potentially harmful command blocked: {command_text}")
            return False

    # Validate command length to prevent buffer overflow attacks
    if len(command_text) > 200:  # arbitrary limit
        self.get_logger().warning(f"Command too long: {len(command_text)} chars")
        return False

    # Additional validation can be added here
    return True
```

## Next Steps

In the next chapters, we'll explore how to process these voice commands using LLMs to generate ROS 2 actions and integrate with vision systems for a complete Vision-Language-Action pipeline.