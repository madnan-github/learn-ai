---
sidebar_position: 2
title: 'Voice Command Processing'
---

# Voice Command Processing

This chapter covers voice command processing for humanoid robots, including speech recognition, natural language understanding, and voice-to-action conversion using OpenAI Whisper and other speech processing technologies.

## What You'll Learn

In this chapter, you'll explore:
- Speech recognition systems for robotics
- Voice command interpretation
- Integration with OpenAI Whisper
- Natural language processing for commands
- Voice activity detection
- Multi-language voice command support

## Prerequisites

- Completion of Module 1-3
- Understanding of ROS 2 messaging
- Basic knowledge of audio processing
- Experience with Python and AI libraries
- Audio input hardware (microphone)

## Speech Recognition Systems for Robotics

### Overview of Speech Recognition in Robotics

Speech recognition in robotics involves converting spoken language into text that can be processed by the robot's AI systems. For humanoid robots, this enables natural human-robot interaction through voice commands.

### Key Components

1. **Audio Input**: Microphones or microphone arrays
2. **Voice Activity Detection**: Distinguishing speech from background noise
3. **Speech-to-Text**: Converting audio to text
4. **Natural Language Understanding**: Interpreting the meaning of commands
5. **Action Mapping**: Converting commands to robot actions

### Basic Voice Command Processing Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import AudioData
import speech_recognition as sr
import threading
import queue
import time

class VoiceCommandProcessor(Node):
    def __init__(self):
        super().__init__('voice_command_processor')

        # Publisher for recognized text
        self.text_pub = self.create_publisher(String, '/recognized_text', 10)

        # Publisher for robot commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize speech recognizer
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 4000  # Adjust based on environment
        self.recognizer.dynamic_energy_threshold = True

        # Audio source (microphone)
        try:
            self.microphone = sr.Microphone()
        except OSError:
            self.get_logger().warn('Microphone not found, using simulation mode')
            self.microphone = None

        # Command queue for processing
        self.command_queue = queue.Queue()

        # Start voice recognition thread
        self.recognition_thread = threading.Thread(target=self.voice_recognition_loop, daemon=True)
        self.recognition_thread.start()

        # Timer for periodic processing
        self.timer = self.create_timer(0.1, self.process_commands)

        self.get_logger().info('Voice Command Processor initialized')

    def voice_recognition_loop(self):
        """Main loop for voice recognition"""
        if self.microphone is None:
            self.get_logger().warn('Cannot start voice recognition - no microphone available')
            return

        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)  # Adjust for background noise

        self.get_logger().info('Voice recognition started, listening for commands...')

        while rclpy.ok():
            try:
                with self.microphone as source:
                    # Listen for audio with timeout
                    audio = self.recognizer.listen(source, timeout=1.0, phrase_time_limit=5.0)

                # Process audio in separate thread to avoid blocking
                threading.Thread(
                    target=self.recognize_audio,
                    args=(audio,),
                    daemon=True
                ).start()

            except sr.WaitTimeoutError:
                # No speech detected, continue listening
                continue
            except Exception as e:
                self.get_logger().error(f'Voice recognition error: {str(e)}')
                time.sleep(0.1)  # Brief pause before retrying

    def recognize_audio(self, audio):
        """Recognize audio and add to command queue"""
        try:
            # Use Google Web Speech API (requires internet)
            text = self.recognizer.recognize_google(audio)

            # Add recognized text to queue
            self.command_queue.put(text)

            self.get_logger().info(f'Recognized: "{text}"')

            # Publish recognized text
            text_msg = String()
            text_msg.data = text
            self.text_pub.publish(text_msg)

        except sr.UnknownValueError:
            self.get_logger().info('Could not understand audio')
        except sr.RequestError as e:
            self.get_logger().error(f'Could not request results from speech recognition service; {e}')
        except Exception as e:
            self.get_logger().error(f'Error in audio recognition: {str(e)}')

    def process_commands(self):
        """Process recognized commands"""
        # Process all available commands in queue
        while not self.command_queue.empty():
            try:
                command_text = self.command_queue.get_nowait()
                self.process_command(command_text)
            except queue.Empty:
                break

    def process_command(self, command_text):
        """Process a single voice command"""
        # Convert to lowercase for easier processing
        command_lower = command_text.lower().strip()

        # Define command mappings
        command_mappings = {
            'move forward': self.move_forward,
            'go forward': self.move_forward,
            'move backward': self.move_backward,
            'go backward': self.move_backward,
            'turn left': self.turn_left,
            'turn right': self.turn_right,
            'stop': self.stop_robot,
            'halt': self.stop_robot,
            'spin left': self.spin_left,
            'spin right': self.spin_right
        }

        # Check for command matches
        for command_phrase, command_func in command_mappings.items():
            if command_phrase in command_lower:
                self.get_logger().info(f'Executing command: {command_phrase}')
                command_func()
                return

        # If no exact match, try fuzzy matching or NLP processing
        self.process_fuzzy_command(command_lower)

    def move_forward(self):
        """Move robot forward"""
        cmd = Twist()
        cmd.linear.x = 0.3  # Forward speed
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def move_backward(self):
        """Move robot backward"""
        cmd = Twist()
        cmd.linear.x = -0.3  # Backward speed
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def turn_left(self):
        """Turn robot left"""
        cmd = Twist()
        cmd.linear.x = 0.1  # Small forward movement while turning
        cmd.angular.z = 0.5  # Left turn
        self.cmd_pub.publish(cmd)

    def turn_right(self):
        """Turn robot right"""
        cmd = Twist()
        cmd.linear.x = 0.1  # Small forward movement while turning
        cmd.angular.z = -0.5  # Right turn
        self.cmd_pub.publish(cmd)

    def spin_left(self):
        """Spin robot left in place"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.8  # Fast left spin
        self.cmd_pub.publish(cmd)

    def spin_right(self):
        """Spin robot right in place"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = -0.8  # Fast right spin
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        """Stop robot movement"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def process_fuzzy_command(self, command_lower):
        """Process commands using fuzzy matching or NLP"""
        # Simple keyword-based fuzzy matching
        if 'forward' in command_lower or 'ahead' in command_lower:
            self.move_forward()
        elif 'backward' in command_lower or 'back' in command_lower:
            self.move_backward()
        elif 'left' in command_lower:
            self.turn_left()
        elif 'right' in command_lower:
            self.turn_right()
        elif 'stop' in command_lower or 'halt' in command_lower:
            self.stop_robot()
        else:
            self.get_logger().info(f'Unknown command: "{command_lower}", ignoring')

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down voice command processor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with OpenAI Whisper

### Whisper-based Voice Recognition

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
import numpy as np
import pyaudio
import wave
import threading
import queue
import time
import io

class WhisperVoiceProcessor(Node):
    def __init__(self):
        super().__init__('whisper_voice_processor')

        # Publisher for recognized text
        self.text_pub = self.create_publisher(String, '/whisper_recognized_text', 10)

        # Audio parameters
        self.rate = 16000  # Sample rate
        self.chunk = 1024  # Buffer size
        self.format = pyaudio.paInt16
        self.channels = 1
        self.record_seconds = 5  # Maximum recording length

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()

        # Audio stream
        try:
            self.stream = self.audio.open(
                format=self.format,
                channels=self.channels,
                rate=self.rate,
                input=True,
                frames_per_buffer=self.chunk
            )
            self.microphone_available = True
        except Exception as e:
            self.get_logger().warn(f'Could not initialize audio stream: {e}')
            self.microphone_available = False

        # Command queue
        self.command_queue = queue.Queue()

        # Start audio recording thread
        self.recording_thread = threading.Thread(target=self.audio_recording_loop, daemon=True)
        self.recording_thread.start()

        # Timer for processing
        self.timer = self.create_timer(0.1, self.process_commands)

        # Initialize Whisper model (this will be loaded later)
        self.whisper_model = None
        self.load_whisper_model()

        self.get_logger().info('Whisper Voice Processor initialized')

    def load_whisper_model(self):
        """Load Whisper model for speech recognition"""
        try:
            import whisper
            # Load model (you can choose different sizes: tiny, base, small, medium, large)
            self.whisper_model = whisper.load_model("base")
            self.get_logger().info('Whisper model loaded successfully')
        except ImportError:
            self.get_logger().error('Whisper library not found. Install with: pip install openai-whisper')
            self.whisper_model = None
        except Exception as e:
            self.get_logger().error(f'Error loading Whisper model: {str(e)}')
            self.whisper_model = None

    def audio_recording_loop(self):
        """Main loop for audio recording"""
        if not self.microphone_available:
            self.get_logger().warn('Audio recording not available - no microphone')
            return

        self.get_logger().info('Audio recording started, listening for commands...')

        while rclpy.ok():
            try:
                # Record audio
                frames = []
                silent_chunks = 0
                max_silent_chunks = 20  # About 0.5 seconds of silence to stop

                # Listen for voice activity
                while rclpy.ok():
                    data = self.stream.read(self.chunk, exception_on_overflow=False)
                    frames.append(data)

                    # Simple voice activity detection based on amplitude
                    audio_data = np.frombuffer(data, dtype=np.int16)
                    amplitude = np.sqrt(np.mean(audio_data**2))

                    if amplitude < 500:  # Threshold for silence
                        silent_chunks += 1
                        if silent_chunks > max_silent_chunks:
                            break  # Stop recording after silence
                    else:
                        silent_chunks = 0  # Reset on voice activity

                    # Limit recording length
                    if len(frames) > self.rate * self.record_seconds / self.chunk:
                        break

                # If we recorded something
                if len(frames) > max_silent_chunks:
                    # Convert to WAV format for Whisper
                    wav_buffer = self.create_wav_buffer(frames)

                    # Process with Whisper in separate thread
                    threading.Thread(
                        target=self.transcribe_audio,
                        args=(wav_buffer,),
                        daemon=True
                    ).start()

            except Exception as e:
                self.get_logger().error(f'Audio recording error: {str(e)}')
                time.sleep(0.1)

    def create_wav_buffer(self, frames):
        """Create WAV buffer from audio frames"""
        wav_buffer = io.BytesIO()

        # Create WAV file
        with wave.open(wav_buffer, 'wb') as wav_file:
            wav_file.setnchannels(self.channels)
            wav_file.setsampwidth(self.audio.get_sample_size(self.format))
            wav_file.setframerate(self.rate)
            wav_file.writeframes(b''.join(frames))

        wav_buffer.seek(0)
        return wav_buffer

    def transcribe_audio(self, wav_buffer):
        """Transcribe audio using Whisper"""
        if self.whisper_model is None:
            return

        try:
            # Save buffer to temporary file for Whisper
            import tempfile
            import os

            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                temp_file.write(wav_buffer.getvalue())
                temp_filename = temp_file.name

            # Transcribe using Whisper
            result = self.whisper_model.transcribe(temp_filename)
            text = result['text'].strip()

            # Clean up temporary file
            os.unlink(temp_filename)

            if text:  # If transcription is not empty
                self.get_logger().info(f'Whisper recognized: "{text}"')

                # Add to command queue
                self.command_queue.put(text)

                # Publish recognized text
                text_msg = String()
                text_msg.data = text
                self.text_pub.publish(text_msg)

        except Exception as e:
            self.get_logger().error(f'Whisper transcription error: {str(e)}')

    def process_commands(self):
        """Process recognized commands"""
        # Process all available commands in queue
        while not self.command_queue.empty():
            try:
                command_text = self.command_queue.get_nowait()
                self.process_command(command_text)
            except queue.Empty:
                break

    def process_command(self, command_text):
        """Process a single voice command from Whisper"""
        # This would typically forward to a higher-level command processor
        # For now, just log the command
        self.get_logger().info(f'Processing Whisper command: {command_text}')

    def destroy_node(self):
        """Clean up audio resources"""
        if self.microphone_available:
            self.stream.stop_stream()
            self.stream.close()
        self.audio.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WhisperVoiceProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down Whisper voice processor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Voice Activity Detection

### Advanced Voice Activity Detection Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import AudioData
import numpy as np
import pyaudio
import threading
import time
from collections import deque

class VoiceActivityDetector(Node):
    def __init__(self):
        super().__init__('voice_activity_detector')

        # Publishers
        self.vad_status_pub = self.create_publisher(Bool, '/voice_activity', 10)
        self.energy_pub = self.create_publisher(Float32, '/audio_energy', 10)

        # Audio parameters
        self.rate = 16000
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1

        # VAD parameters
        self.energy_threshold = 1000  # Adjust based on environment
        self.silence_duration_threshold = 0.5  # seconds of silence to end speech
        self.speech_duration_threshold = 0.1   # minimum speech duration

        # Audio analysis
        self.audio = pyaudio.PyAudio()
        self.energy_history = deque(maxlen=100)  # Store recent energy values
        self.speech_active = False
        self.silence_start_time = None
        self.speech_start_time = None

        try:
            self.stream = self.audio.open(
                format=self.format,
                channels=self.channels,
                rate=self.rate,
                input=True,
                frames_per_buffer=self.chunk
            )
            self.microphone_available = True
        except Exception as e:
            self.get_logger().warn(f'Could not initialize audio stream: {e}')
            self.microphone_available = False

        # Start VAD thread
        self.vad_thread = threading.Thread(target=self.vad_loop, daemon=True)
        self.vad_thread.start()

        self.get_logger().info('Voice Activity Detector initialized')

    def vad_loop(self):
        """Main loop for voice activity detection"""
        if not self.microphone_available:
            return

        self.get_logger().info('Voice activity detection started')

        while rclpy.ok():
            try:
                # Read audio data
                data = self.stream.read(self.chunk, exception_on_overflow=False)

                # Convert to numpy array
                audio_data = np.frombuffer(data, dtype=np.int16)

                # Calculate energy (RMS)
                energy = np.sqrt(np.mean(audio_data**2))

                # Update energy history
                self.energy_history.append(energy)

                # Publish current energy
                energy_msg = Float32()
                energy_msg.data = float(energy)
                self.energy_pub.publish(energy_msg)

                # Detect voice activity
                is_speech = self.detect_voice_activity(energy)

                # Update speech state
                if is_speech and not self.speech_active:
                    # Speech started
                    self.speech_active = True
                    self.speech_start_time = time.time()
                    self.publish_vad_status(True)
                    self.get_logger().info('Speech detected')
                elif not is_speech and self.speech_active:
                    # Check if speech has ended (enough silence)
                    if self.silence_start_time is None:
                        self.silence_start_time = time.time()
                    elif time.time() - self.silence_start_time > self.silence_duration_threshold:
                        # Speech ended
                        speech_duration = time.time() - self.speech_start_time
                        if speech_duration >= self.speech_duration_threshold:
                            self.get_logger().info(f'Speech ended (duration: {speech_duration:.2f}s)')

                        self.speech_active = False
                        self.silence_start_time = None
                        self.publish_vad_status(False)

                time.sleep(0.01)  # 100Hz processing rate

            except Exception as e:
                self.get_logger().error(f'VAD loop error: {str(e)}')
                time.sleep(0.1)

    def detect_voice_activity(self, energy):
        """Detect voice activity based on energy threshold"""
        # Use adaptive threshold based on recent history
        if len(self.energy_history) > 10:
            recent_avg = np.mean(list(self.energy_history)[-10:])
            adaptive_threshold = recent_avg * 2.0  # 2x the recent average

            # Use the higher of fixed and adaptive threshold
            threshold = max(self.energy_threshold, adaptive_threshold)
        else:
            threshold = self.energy_threshold

        return energy > threshold

    def publish_vad_status(self, is_speech):
        """Publish voice activity status"""
        status_msg = Bool()
        status_msg.data = is_speech
        self.vad_status_pub.publish(status_msg)

    def destroy_node(self):
        """Clean up audio resources"""
        if self.microphone_available:
            self.stream.stop_stream()
            self.stream.close()
        self.audio.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VoiceActivityDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down voice activity detector...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Natural Language Processing for Commands

### Command Understanding with NLP

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
import spacy
import re
from dataclasses import dataclass
from typing import Optional, List

@dataclass
class CommandIntent:
    """Data class for command intent"""
    action: str
    direction: Optional[str] = None
    distance: Optional[float] = None
    angle: Optional[float] = None
    target: Optional[str] = None

class NLPCommandProcessor(Node):
    def __init__(self):
        super().__init__('nlp_command_processor')

        # Subscribe to recognized text
        self.text_sub = self.create_subscription(
            String,
            '/recognized_text',
            self.text_callback,
            10
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.intent_pub = self.create_publisher(String, '/command_intent', 10)

        # Load spaCy model for NLP
        try:
            self.nlp = spacy.load("en_core_web_sm")
            self.get_logger().info('spaCy model loaded successfully')
        except OSError:
            self.get_logger().error('spaCy model not found. Install with: python -m spacy download en_core_web_sm')
            self.nlp = None

        # Command action mappings
        self.action_keywords = {
            'move': ['move', 'go', 'walk', 'travel', 'navigate'],
            'turn': ['turn', 'rotate', 'spin', 'pivot'],
            'stop': ['stop', 'halt', 'pause', 'stand', 'freeze'],
            'approach': ['approach', 'come to', 'go to', 'reach', 'get to'],
            'find': ['find', 'locate', 'search for', 'look for', 'detect']
        }

        # Direction keywords
        self.direction_keywords = {
            'forward': ['forward', 'ahead', 'straight', 'front'],
            'backward': ['backward', 'back', 'reverse'],
            'left': ['left', 'port'],
            'right': ['right', 'starboard']
        }

        # Distance/angle keywords
        self.distance_units = ['meter', 'meters', 'm', 'centimeter', 'centimeters', 'cm', 'foot', 'feet', 'ft']
        self.angle_units = ['degree', 'degrees', 'deg']

        self.get_logger().info('NLP Command Processor initialized')

    def text_callback(self, msg):
        """Process recognized text for command understanding"""
        if self.nlp is None:
            return

        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f'Processing text: "{text}"')

        # Parse command intent
        intent = self.parse_command_intent(text)

        if intent:
            # Publish intent for monitoring
            intent_msg = String()
            intent_msg.data = f"Action: {intent.action}, Direction: {intent.direction}, Distance: {intent.distance}, Angle: {intent.angle}, Target: {intent.target}"
            self.intent_pub.publish(intent_msg)

            # Execute command
            self.execute_command(intent)

    def parse_command_intent(self, text: str) -> Optional[CommandIntent]:
        """Parse natural language command to intent"""
        doc = self.nlp(text.lower())

        # Extract action
        action = self.extract_action(text)
        if not action:
            return None

        # Extract direction
        direction = self.extract_direction(text)

        # Extract distance
        distance = self.extract_distance(text)

        # Extract angle
        angle = self.extract_angle(text)

        # Extract target/object
        target = self.extract_target(doc)

        return CommandIntent(
            action=action,
            direction=direction,
            distance=distance,
            angle=angle,
            target=target
        )

    def extract_action(self, text: str) -> Optional[str]:
        """Extract the main action from text"""
        text_lower = text.lower()

        for action, keywords in self.action_keywords.items():
            for keyword in keywords:
                if keyword in text_lower:
                    return action

        return None

    def extract_direction(self, text: str) -> Optional[str]:
        """Extract direction from text"""
        text_lower = text.lower()

        for direction, keywords in self.direction_keywords.items():
            for keyword in keywords:
                if keyword in text_lower:
                    return direction

        return None

    def extract_distance(self, text: str) -> Optional[float]:
        """Extract distance from text"""
        # Look for number followed by distance unit
        pattern = r'(\d+(?:\.\d+)?)\s*(meter|meters|m|centimeter|centimeters|cm|foot|feet|ft)'
        match = re.search(pattern, text.lower())

        if match:
            value = float(match.group(1))
            unit = match.group(2)

            # Convert to meters
            if unit in ['centimeter', 'centimeters', 'cm']:
                return value / 100.0
            elif unit in ['foot', 'feet', 'ft']:
                return value * 0.3048
            else:  # meters
                return value

        return None

    def extract_angle(self, text: str) -> Optional[float]:
        """Extract angle from text"""
        # Look for number followed by angle unit
        pattern = r'(\d+(?:\.\d+)?)\s*(degree|degrees|deg)'
        match = re.search(pattern, text.lower())

        if match:
            value = float(match.group(1))
            # Convert to radians for ROS
            return value * 3.14159 / 180.0

        return None

    def extract_target(self, doc) -> Optional[str]:
        """Extract target object from parsed document"""
        # Look for nouns that might be targets
        for token in doc:
            if token.pos_ in ['NOUN', 'PROPN'] and token.dep_ in ['dobj', 'pobj']:
                return token.text

        # Look for named entities
        for ent in doc.ents:
            if ent.label_ in ['OBJECT', 'PERSON', 'FAC', 'GPE']:
                return ent.text

        return None

    def execute_command(self, intent: CommandIntent):
        """Execute the parsed command intent"""
        if intent.action == 'move':
            self.execute_move_command(intent)
        elif intent.action == 'turn':
            self.execute_turn_command(intent)
        elif intent.action == 'stop':
            self.execute_stop_command()
        elif intent.action == 'approach':
            self.execute_approach_command(intent)
        elif intent.action == 'find':
            self.execute_find_command(intent)
        else:
            self.get_logger().info(f'Unknown action: {intent.action}')

    def execute_move_command(self, intent: CommandIntent):
        """Execute move command"""
        cmd = Twist()

        if intent.direction == 'forward':
            cmd.linear.x = 0.3 if intent.distance is None else min(0.5, intent.distance)
        elif intent.direction == 'backward':
            cmd.linear.x = -0.3 if intent.distance is None else -min(0.5, intent.distance)
        elif intent.direction == 'left':
            cmd.angular.z = 0.5 if intent.angle is None else intent.angle
        elif intent.direction == 'right':
            cmd.angular.z = -0.5 if intent.angle is None else -intent.angle

        self.cmd_pub.publish(cmd)

    def execute_turn_command(self, intent: CommandIntent):
        """Execute turn command"""
        cmd = Twist()

        if intent.direction == 'left':
            cmd.angular.z = 0.5 if intent.angle is None else intent.angle
        elif intent.direction == 'right':
            cmd.angular.z = -0.5 if intent.angle is None else -intent.angle

        self.cmd_pub.publish(cmd)

    def execute_stop_command(self):
        """Execute stop command"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def execute_approach_command(self, intent: CommandIntent):
        """Execute approach command"""
        if intent.target:
            self.get_logger().info(f'Approaching target: {intent.target}')
            # In a real system, this would involve navigation to the target
            # For now, just move forward
            cmd = Twist()
            cmd.linear.x = 0.2
            self.cmd_pub.publish(cmd)

    def execute_find_command(self, intent: CommandIntent):
        """Execute find command"""
        if intent.target:
            self.get_logger().info(f'Looking for: {intent.target}')
            # In a real system, this would involve object detection
            # For now, just rotate to scan
            cmd = Twist()
            cmd.angular.z = 0.3  # Slow rotation to look around
            self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = NLPCommandProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down NLP command processor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Multi-Language Voice Commands

### Multi-Language Voice Processing

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int8
import speech_recognition as sr
import threading
import queue
import time

class MultiLanguageVoiceProcessor(Node):
    def __init__(self):
        super().__init__('multi_language_voice_processor')

        # Publishers
        self.text_pub = self.create_publisher(String, '/multilang_recognized_text', 10)
        self.language_pub = self.create_publisher(Int8, '/detected_language', 10)

        # Supported languages
        self.supported_languages = {
            'en': 'English',
            'es': 'Spanish',
            'fr': 'French',
            'de': 'German',
            'it': 'Italian',
            'pt': 'Portuguese',
            'ru': 'Russian',
            'ja': 'Japanese',
            'ko': 'Korean',
            'zh': 'Chinese'
        }

        # Current language (default to English)
        self.current_language = 'en'
        self.language_names = list(self.supported_languages.keys())

        # Initialize speech recognizer
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 4000
        self.recognizer.dynamic_energy_threshold = True

        # Audio source
        try:
            self.microphone = sr.Microphone()
        except OSError:
            self.get_logger().warn('Microphone not found, using simulation mode')
            self.microphone = None

        # Command queue
        self.command_queue = queue.Queue()

        # Start recognition thread
        self.recognition_thread = threading.Thread(target=self.recognition_loop, daemon=True)
        self.recognition_thread.start()

        # Timer for processing
        self.timer = self.create_timer(0.1, self.process_commands)

        self.get_logger().info('Multi-Language Voice Processor initialized')

    def recognition_loop(self):
        """Main loop for multi-language recognition"""
        if self.microphone is None:
            return

        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        self.get_logger().info('Multi-language voice recognition started')

        while rclpy.ok():
            try:
                with self.microphone as source:
                    audio = self.recognizer.listen(source, timeout=1.0, phrase_time_limit=5.0)

                # Try recognition in current language first, then try others if needed
                text = self.recognize_with_language(audio, self.current_language)

                if text:
                    self.get_logger().info(f'Recognized ({self.current_language}): "{text}"')

                    # Add to queue
                    self.command_queue.put((text, self.current_language))

                    # Publish text
                    text_msg = String()
                    text_msg.data = f"[{self.current_language}] {text}"
                    self.text_pub.publish(text_msg)

                    # Publish language
                    lang_msg = Int8()
                    lang_msg.data = list(self.language_names).index(self.current_language)
                    self.language_pub.publish(lang_msg)

            except sr.WaitTimeoutError:
                continue
            except Exception as e:
                self.get_logger().error(f'Multi-lang recognition error: {str(e)}')
                time.sleep(0.1)

    def recognize_with_language(self, audio, language_code):
        """Recognize audio with specific language"""
        try:
            # Use Google Web Speech API with specific language
            text = self.recognizer.recognize_google(audio, language=language_code)
            return text
        except sr.UnknownValueError:
            # Try other languages if current one fails
            for other_lang in self.language_names:
                if other_lang != language_code:
                    try:
                        text = self.recognizer.recognize_google(audio, language=other_lang)
                        # Update current language
                        self.current_language = other_lang
                        self.get_logger().info(f'Switched to language: {other_lang}')
                        return text
                    except sr.UnknownValueError:
                        continue
            return None
        except sr.RequestError as e:
            self.get_logger().error(f'Speech recognition request error: {e}')
            return None

    def process_commands(self):
        """Process recognized commands"""
        while not self.command_queue.empty():
            try:
                text, language = self.command_queue.get_nowait()
                self.process_multilang_command(text, language)
            except queue.Empty:
                break

    def process_multilang_command(self, text, language):
        """Process command in specific language"""
        # Convert to lowercase for processing
        text_lower = text.lower()

        # Language-specific command mappings
        command_mappings = self.get_command_mappings(language)

        # Check for command matches
        for command_phrase, command_func in command_mappings.items():
            if command_phrase.lower() in text_lower:
                self.get_logger().info(f'Executing {language} command: {command_phrase}')
                command_func()
                return

        # If no match, log the unrecognized command
        self.get_logger().info(f'Unrecognized {language} command: "{text}"')

    def get_command_mappings(self, language):
        """Get command mappings for specific language"""
        if language == 'en':  # English
            return {
                'move forward': self.move_forward,
                'go forward': self.move_forward,
                'move backward': self.move_backward,
                'go backward': self.move_backward,
                'turn left': self.turn_left,
                'turn right': self.turn_right,
                'stop': self.stop_robot,
                'halt': self.stop_robot
            }
        elif language == 'es':  # Spanish
            return {
                'mover adelante': self.move_forward,
                'ir adelante': self.move_forward,
                'mover atrás': self.move_backward,
                'girar izquierda': self.turn_left,
                'girar derecha': self.turn_right,
                'parar': self.stop_robot,
                'detenerse': self.stop_robot
            }
        elif language == 'fr':  # French
            return {
                'avancer': self.move_forward,
                'reculer': self.move_backward,
                'tourner à gauche': self.turn_left,
                'tourner à droite': self.turn_right,
                'arrêter': self.stop_robot,
                'stop': self.stop_robot
            }
        elif language == 'de':  # German
            return {
                'vorwärts': self.move_forward,
                'rückwärts': self.move_backward,
                'links': self.turn_left,
                'rechts': self.turn_right,
                'halten': self.stop_robot,
                'stopp': self.stop_robot
            }
        else:  # Default to English for other languages
            return {
                'move forward': self.move_forward,
                'go forward': self.move_forward,
                'move backward': self.move_backward,
                'go backward': self.move_backward,
                'turn left': self.turn_left,
                'turn right': self.turn_right,
                'stop': self.stop_robot,
                'halt': self.stop_robot
            }

    def move_forward(self):
        """Move robot forward"""
        from geometry_msgs.msg import Twist
        cmd = Twist()
        cmd.linear.x = 0.3
        cmd.angular.z = 0.0
        self.create_publisher(Twist, '/cmd_vel', 10).publish(cmd)

    def move_backward(self):
        """Move robot backward"""
        from geometry_msgs.msg import Twist
        cmd = Twist()
        cmd.linear.x = -0.3
        cmd.angular.z = 0.0
        self.create_publisher(Twist, '/cmd_vel', 10).publish(cmd)

    def turn_left(self):
        """Turn robot left"""
        from geometry_msgs.msg import Twist
        cmd = Twist()
        cmd.linear.x = 0.1
        cmd.angular.z = 0.5
        self.create_publisher(Twist, '/cmd_vel', 10).publish(cmd)

    def turn_right(self):
        """Turn robot right"""
        from geometry_msgs.msg import Twist
        cmd = Twist()
        cmd.linear.x = 0.1
        cmd.angular.z = -0.5
        self.create_publisher(Twist, '/cmd_vel', 10).publish(cmd)

    def stop_robot(self):
        """Stop robot movement"""
        from geometry_msgs.msg import Twist
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.create_publisher(Twist, '/cmd_vel', 10).publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MultiLanguageVoiceProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down multi-language voice processor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-on Lab: Complete Voice Command System

In this lab, you'll create a complete voice command system that integrates all components.

### Step 1: Create the Voice Command System Launch File

Create `voice_command_system_launch.py`:

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Voice command system nodes
    voice_processor = Node(
        package='ai_robo_learning',
        executable='voice_command_processor',
        name='voice_command_processor',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    whisper_processor = Node(
        package='ai_robo_learning',
        executable='whisper_voice_processor',
        name='whisper_voice_processor',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    vad_detector = Node(
        package='ai_robo_learning',
        executable='voice_activity_detector',
        name='voice_activity_detector',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    nlp_processor = Node(
        package='ai_robo_learning',
        executable='nlp_command_processor',
        name='nlp_command_processor',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    multilang_processor = Node(
        package='ai_robo_learning',
        executable='multi_language_voice_processor',
        name='multi_language_voice_processor',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Return launch description
    ld = LaunchDescription()

    # Add all nodes
    ld.add_action(voice_processor)
    ld.add_action(whisper_processor)
    ld.add_action(vad_detector)
    ld.add_action(nlp_processor)
    ld.add_action(multilang_processor)

    return ld
```

### Step 2: Create the Complete Voice Command Node

Create `complete_voice_command_system.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import AudioData
import speech_recognition as sr
import threading
import queue
import time
import numpy as np
import pyaudio
from collections import deque

class CompleteVoiceCommandSystem(Node):
    def __init__(self):
        super().__init__('complete_voice_command_system')

        # Publishers
        self.text_pub = self.create_publisher(String, '/complete_recognized_text', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(Bool, '/voice_system_active', 10)

        # Initialize components
        self.setup_speech_recognition()
        self.setup_audio_input()
        self.setup_command_processing()

        # System state
        self.system_active = True
        self.listening_enabled = True
        self.command_history = deque(maxlen=50)

        # Start processing threads
        self.recognition_thread = threading.Thread(target=self.recognition_loop, daemon=True)
        self.recognition_thread.start()

        # Timer for system monitoring
        self.timer = self.create_timer(1.0, self.system_monitor)

        self.get_logger().info('Complete Voice Command System initialized')

    def setup_speech_recognition(self):
        """Setup speech recognition components"""
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 4000
        self.recognizer.dynamic_energy_threshold = True

    def setup_audio_input(self):
        """Setup audio input components"""
        self.rate = 16000
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1

        self.audio = pyaudio.PyAudio()

        try:
            self.microphone = sr.Microphone()
            self.stream = self.audio.open(
                format=self.format,
                channels=self.channels,
                rate=self.rate,
                input=True,
                frames_per_buffer=self.chunk
            )
            self.microphone_available = True
        except Exception as e:
            self.get_logger().warn(f'Could not initialize audio: {e}')
            self.microphone_available = False

    def setup_command_processing(self):
        """Setup command processing components"""
        self.command_queue = queue.Queue()
        self.command_mappings = {
            'move forward': self.move_forward,
            'go forward': self.move_forward,
            'move backward': self.move_backward,
            'go backward': self.move_backward,
            'turn left': self.turn_left,
            'turn right': self.turn_right,
            'spin left': self.spin_left,
            'spin right': self.spin_right,
            'stop': self.stop_robot,
            'halt': self.stop_robot,
            'help': self.show_help
        }

    def recognition_loop(self):
        """Main recognition loop"""
        if not self.microphone_available:
            self.get_logger().warn('Voice recognition not available')
            return

        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        self.get_logger().info('Voice command system listening...')

        while rclpy.ok() and self.system_active:
            try:
                if self.listening_enabled:
                    with self.microphone as source:
                        audio = self.recognizer.listen(source, timeout=1.0, phrase_time_limit=5.0)

                    # Process audio
                    threading.Thread(
                        target=self.process_audio,
                        args=(audio,),
                        daemon=True
                    ).start()

            except sr.WaitTimeoutError:
                continue
            except Exception as e:
                self.get_logger().error(f'Recognition loop error: {str(e)}')
                time.sleep(0.1)

    def process_audio(self, audio):
        """Process audio and recognize speech"""
        try:
            # Try Google Speech Recognition
            text = self.recognizer.recognize_google(audio)
            self.process_recognized_text(text)

        except sr.UnknownValueError:
            self.get_logger().info('Could not understand audio')
        except sr.RequestError as e:
            self.get_logger().error(f'Speech recognition error: {e}')

    def process_recognized_text(self, text):
        """Process recognized text"""
        if not text.strip():
            return

        self.get_logger().info(f'Recognized: "{text}"')

        # Add to history
        self.command_history.append({
            'text': text,
            'timestamp': time.time()
        })

        # Publish recognized text
        text_msg = String()
        text_msg.data = text
        self.text_pub.publish(text_msg)

        # Process command
        self.process_command(text)

    def process_command(self, command_text):
        """Process voice command"""
        command_lower = command_text.lower().strip()

        # Check for exact matches first
        for cmd_phrase, cmd_func in self.command_mappings.items():
            if cmd_phrase in command_lower:
                self.get_logger().info(f'Executing command: {cmd_phrase}')
                cmd_func()
                return

        # Check for fuzzy matches
        self.process_fuzzy_command(command_lower)

    def process_fuzzy_command(self, command_lower):
        """Process commands using fuzzy matching"""
        # Simple keyword matching
        if any(word in command_lower for word in ['forward', 'ahead', 'straight']):
            self.move_forward()
        elif any(word in command_lower for word in ['backward', 'back', 'reverse']):
            self.move_backward()
        elif 'left' in command_lower:
            if 'spin' in command_lower or 'turn' in command_lower:
                self.spin_left()
            else:
                self.turn_left()
        elif 'right' in command_lower:
            if 'spin' in command_lower or 'turn' in command_lower:
                self.spin_right()
            else:
                self.turn_right()
        elif any(word in command_lower for word in ['stop', 'halt', 'pause']):
            self.stop_robot()
        else:
            self.get_logger().info(f'Unknown command: "{command_lower}"')

    def move_forward(self):
        """Move robot forward"""
        cmd = Twist()
        cmd.linear.x = 0.3
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def move_backward(self):
        """Move robot backward"""
        cmd = Twist()
        cmd.linear.x = -0.3
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def turn_left(self):
        """Turn robot left"""
        cmd = Twist()
        cmd.linear.x = 0.1
        cmd.angular.z = 0.5
        self.cmd_pub.publish(cmd)

    def turn_right(self):
        """Turn robot right"""
        cmd = Twist()
        cmd.linear.x = 0.1
        cmd.angular.z = -0.5
        self.cmd_pub.publish(cmd)

    def spin_left(self):
        """Spin robot left"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.8
        self.cmd_pub.publish(cmd)

    def spin_right(self):
        """Spin robot right"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = -0.8
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        """Stop robot"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def show_help(self):
        """Show available commands"""
        help_text = """
        Available voice commands:
        - Move forward/Go forward
        - Move backward/Go backward
        - Turn left/Turn right
        - Spin left/Spin right
        - Stop/Halt
        """
        self.get_logger().info(help_text)

    def system_monitor(self):
        """Monitor system status"""
        status_msg = Bool()
        status_msg.data = self.system_active
        self.status_pub.publish(status_msg)

        # Log system status periodically
        self.get_logger().info(f'Voice system active: {self.system_active}, Commands in history: {len(self.command_history)}')

    def destroy_node(self):
        """Clean up resources"""
        self.system_active = False
        if self.microphone_available:
            self.stream.stop_stream()
            self.stream.close()
        self.audio.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CompleteVoiceCommandSystem()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down complete voice command system...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Test the Voice Command System

1. Make sure you have the required dependencies:
```bash
pip3 install speechrecognition pyaudio numpy
# For Whisper (optional):
pip3 install openai-whisper
# For spaCy (optional):
python -m spacy download en_core_web_sm
```

2. Run the complete voice command system:
```bash
python3 complete_voice_command_system.py
```

3. Speak commands to your microphone such as:
   - "Move forward"
   - "Turn left"
   - "Stop"
   - "Spin right"

## Best Practices

1. **Robust Recognition**: Implement multiple fallback recognition methods
2. **Voice Activity Detection**: Use VAD to reduce processing when not speaking
3. **Noise Reduction**: Apply audio filtering to improve recognition quality
4. **Command Validation**: Verify commands make sense before executing
5. **Error Handling**: Gracefully handle recognition failures
6. **Privacy**: Consider privacy implications of always-listening systems
7. **Multi-language Support**: Design systems to support multiple languages

## Next Steps

After completing this chapter, you'll be ready to learn about cognitive planning with LLMs in Chapter 3, where you'll explore how to use large language models to translate natural language commands into sequences of robot actions.