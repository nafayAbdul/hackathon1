"""
Speech processing utilities for Module 4
Handles speech recognition, natural language processing, and voice command interpretation
"""
import whisper
import torch
import pyaudio
import wave
import threading
import queue
import numpy as np
from transformers import AutoTokenizer, AutoModel
from typing import Dict, List, Optional, Tuple, Any
import logging
import time
import io
from scipy.io import wavfile

logger = logging.getLogger(__name__)

class SpeechProcessingUtilities:
    """
    Utilities for processing speech input in VLA systems
    """
    
    def __init__(self, 
                 model_size: str = "large", 
                 device: str = None,
                 language: str = "en"):
        """
        Initialize speech processing utilities
        
        Args:
            model_size: Size of the Whisper model ('tiny', 'base', 'small', 'medium', 'large')
            device: Device to run the model on ('cuda', 'cpu', or None for auto)
            language: Language for speech recognition
        """
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.language = language
        
        logger.info(f"Initializing speech processing utilities on {self.device}")
        
        # Load Whisper model for speech recognition
        self.speech_model = whisper.load_model(model_size).to(self.device)
        
        # Audio parameters
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        
        # Setup audio stream
        self.audio = pyaudio.PyAudio()
        
        logger.info("Speech processing utilities initialized successfully")
    
    def record_audio(self, duration: float = 5.0, filename: str = "temp_audio.wav") -> str:
        """
        Record audio from microphone for specified duration
        
        Args:
            duration: Recording duration in seconds
            filename: Output filename for the recorded audio
            
        Returns:
            Path to the recorded audio file
        """
        try:
            logger.info(f"Starting audio recording for {duration} seconds...")
            
            stream = self.audio.open(
                format=self.format,
                channels=self.channels,
                rate=self.rate,
                input=True,
                frames_per_buffer=self.chunk
            )
            
            frames = []
            for _ in range(0, int(self.rate / self.chunk * duration)):
                data = stream.read(self.chunk)
                frames.append(data)
            
            stream.stop_stream()
            stream.close()
            
            # Save to WAV file
            wf = wave.open(filename, 'wb')
            wf.setnchannels(self.channels)
            wf.setsampwidth(self.audio.get_sample_size(self.format))
            wf.setframerate(self.rate)
            wf.writeframes(b''.join(frames))
            wf.close()
            
            logger.info(f"Audio recorded successfully to {filename}")
            return filename
            
        except Exception as e:
            logger.error(f"Error recording audio: {e}")
            raise
    
    def transcribe_audio(self, audio_file: str) -> Dict[str, Any]:
        """
        Transcribe audio file to text using Whisper
        
        Args:
            audio_file: Path to the audio file to transcribe
            
        Returns:
            Dictionary containing transcription results
        """
        try:
            logger.info(f"Transcribing audio file: {audio_file}")
            
            # Load audio file
            audio = whisper.load_audio(audio_file)
            audio = whisper.pad_or_trim(audio)
            
            # Make log-Mel spectrogram and move to the same device as the model
            mel = whisper.log_mel_spectrogram(audio).to(self.speech_model.device)
            
            # Detect language
            _, probs = self.speech_model.detect_language(mel)
            detected_language = max(probs, key=probs.get)
            
            # Decode the audio
            options = whisper.DecodingOptions(fp16=self.device == "cuda")
            result = whisper.decode(self.speech_model, mel, options)
            
            transcription_result = {
                'text': result.text,
                'language': detected_language,
                'duration': len(audio) / self.rate,
                'success': True,
                'message': 'Transcription completed successfully'
            }
            
            logger.info(f"Transcription completed: {result.text}")
            return transcription_result
            
        except Exception as e:
            logger.error(f"Error transcribing audio: {e}")
            return {
                'text': '',
                'language': self.language,
                'success': False,
                'message': str(e)
            }
    
    def transcribe_audio_bytes(self, audio_bytes: bytes) -> Dict[str, Any]:
        """
        Transcribe audio from bytes data directly
        
        Args:
            audio_bytes: Audio data as bytes
            
        Returns:
            Dictionary containing transcription results
        """
        try:
            # Save bytes to temporary file
            import tempfile
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                temp_file.write(audio_bytes)
                temp_filename = temp_file.name
            
            # Transcribe the temporary file
            result = self.transcribe_audio(temp_filename)
            
            # Clean up temporary file
            import os
            os.unlink(temp_filename)
            
            return result
        except Exception as e:
            logger.error(f"Error transcribing audio bytes: {e}")
            return {
                'text': '',
                'language': self.language,
                'success': False,
                'message': str(e)
            }
    
    def process_voice_command(self, audio_input) -> Dict[str, Any]:
        """
        Process voice command from audio input (file path or bytes)
        
        Args:
            audio_input: Either path to audio file or audio bytes
            
        Returns:
            Dictionary containing processed command and results
        """
        if isinstance(audio_input, str):
            # If it's a file path
            transcription = self.transcribe_audio(audio_input)
        elif isinstance(audio_input, bytes):
            # If it's audio bytes
            transcription = self.transcribe_audio_bytes(audio_input)
        else:
            return {
                'command': '',
                'success': False,
                'message': 'Invalid audio input type'
            }
        
        if transcription['success']:
            # Here you might add additional NLP processing to extract intent
            processed_command = self.parse_natural_language_command(transcription['text'])
            
            return {
                'command': processed_command,
                'raw_transcription': transcription['text'],
                'language': transcription['language'],
                'success': True,
                'message': 'Command processed successfully'
            }
        else:
            return {
                'command': '',
                'raw_transcription': '',
                'success': False,
                'message': transcription['message']
            }
    
    def parse_natural_language_command(self, text: str) -> Dict[str, Any]:
        """
        Parse natural language command to extract intent and parameters
        
        Args:
            text: Raw transcribed text
            
        Returns:
            Dictionary with parsed command structure
        """
        # For now, return the raw text as the command
        # In a more sophisticated implementation, this would use NLP techniques
        # to extract intent, objects, locations, etc.
        return {
            'raw_command': text,
            'intent': 'generic_action',  # Would be extracted via NLP in real implementation
            'objects': [],  # Would be extracted via NLP in real implementation
            'locations': [],  # Would be extracted via NLP in real implementation
            'actions': []  # Would be extracted via NLP in real implementation
        }
    
    def is_speech_present(self, audio_file: str, threshold: float = 0.01) -> bool:
        """
        Detect if speech is present in the audio file
        
        Args:
            audio_file: Path to audio file
            threshold: Volume threshold to consider as speech
            
        Returns:
            True if speech is detected, False otherwise
        """
        try:
            sample_rate, audio_data = wavfile.read(audio_file)
            
            # Calculate volume (RMS)
            rms = np.sqrt(np.mean(audio_data.astype(float) ** 2))
            
            return rms > threshold
        except Exception as e:
            logger.error(f"Error detecting speech: {e}")
            return False
    
    def __del__(self):
        """
        Clean up audio resources
        """
        try:
            self.audio.terminate()
        except:
            pass  # Audio might already be terminated


class RealTimeSpeechProcessor(SpeechProcessingUtilities):
    """
    Real-time speech processing with continuous listening capability
    """
    
    def __init__(self, 
                 model_size: str = "large", 
                 device: str = None,
                 language: str = "en",
                 wake_word: Optional[str] = "Athena"):
        """
        Initialize real-time speech processor
        
        Args:
            model_size: Size of the Whisper model
            device: Device to run the model on
            language: Language for speech recognition
            wake_word: Wake word to activate processing (None to disable)
        """
        super().__init__(model_size, device, language)
        self.wake_word = wake_word.lower() if wake_word else None
        self.listening = False
        self.audio_queue = queue.Queue()
        self.command_queue = queue.Queue()
        
    def start_listening(self):
        """
        Start continuous listening for voice commands
        """
        self.listening = True
        self.listen_thread = threading.Thread(target=self._continuous_listen)
        self.listen_thread.start()
        logger.info("Started continuous listening for voice commands")
    
    def stop_listening(self):
        """
        Stop continuous listening
        """
        self.listening = False
        if hasattr(self, 'listen_thread'):
            self.listen_thread.join()
        logger.info("Stopped continuous listening")
    
    def _continuous_listen(self):
        """
        Internal method for continuous audio listening
        """
        while self.listening:
            try:
                # Record a short audio segment
                temp_file = self.record_audio(duration=2.0)  # Short segments for real-time
                
                # Check if speech is present
                if self.is_speech_present(temp_file):
                    # Process the audio
                    result = self.transcribe_audio(temp_file)
                    
                    if result['success'] and result['text'].strip():
                        command_text = result['text'].lower().strip()
                        
                        # Check for wake word if enabled
                        if not self.wake_word or self.wake_word in command_text:
                            # Extract command after wake word if present
                            if self.wake_word:
                                command_start = command_text.find(self.wake_word) + len(self.wake_word)
                                command = command_text[command_start:].strip()
                            else:
                                command = command_text
                            
                            # Add command to queue for processing
                            self.command_queue.put({
                                'command': command,
                                'timestamp': time.time(),
                                'success': True
                            })
                            logger.info(f"Detected command: {command}")
                
                # Clean up temp file
                import os
                os.unlink(temp_file)
                
                # Small delay to prevent excessive CPU usage
                time.sleep(0.5)
                
            except Exception as e:
                logger.error(f"Error in continuous listening: {e}")
                time.sleep(1)  # Wait before retrying
    
    def get_command(self, timeout: Optional[float] = None) -> Optional[Dict[str, Any]]:
        """
        Get the next command from the queue
        
        Args:
            timeout: Maximum time to wait for a command (None for no timeout)
            
        Returns:
            Command dictionary or None if timeout
        """
        try:
            return self.command_queue.get(timeout=timeout)
        except queue.Empty:
            return None


# Standalone function for easy usage
def initialize_speech_processor(model_size: str = "large", device: str = None) -> SpeechProcessingUtilities:
    """
    Initialize and return speech processing utilities
    
    Args:
        model_size: Size of the Whisper model to use
        device: Device to run the model on ('cuda', 'cpu', or None for auto)
        
    Returns:
        Initialized SpeechProcessingUtilities instance
    """
    return SpeechProcessingUtilities(model_size, device)


if __name__ == "__main__":
    # Example usage
    print("Testing Speech Processing Utilities...")
    
    # Initialize speech processor
    speech_utils = initialize_speech_processor()
    print("Speech Processing Utilities initialized successfully!")
    
    print("Speech Processing Utilities test completed.")