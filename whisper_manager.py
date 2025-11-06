# whisper_manager.py

import whisper
import sounddevice as sd
from scipy.io.wavfile import write
import numpy as np
import rospy

class WhisperManager:
    def __init__(self, model_size="tiny"):
        """
        Initializes the WhisperManager.
        Args:
            model_size (str): The size of the Whisper model to use 
                              (e.g., "tiny", "base", "small", "medium", "large").
                              Smaller models are faster but less accurate.
        """
        rospy.loginfo(f"Loading Whisper model '{model_size}'...")
        try:
            self.model = whisper.load_model(model_size)
            rospy.loginfo("Whisper model loaded successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to load Whisper model: {e}")
            rospy.logerr("Please ensure you have a stable internet connection for the first download, and that you have enough memory.")
            self.model = None

        self.samplerate = 16000  # 16kHz is standard for Whisper
        self.channels = 1
        self.temp_filename = "temp_recording.wav"

    def record_audio(self, duration=5):
        """
        Records audio from the microphone for a fixed duration.
        """
        rospy.loginfo(f"Recording for {duration} seconds...")
        recording = sd.rec(int(duration * self.samplerate), samplerate=self.samplerate, channels=self.channels, dtype='int16')
        sd.wait()  # Wait until recording is finished
        rospy.loginfo("Recording finished.")
        return recording

    def save_recording(self, recording_data):
        """
        Saves the recorded audio data to a WAV file.
        """
        write(self.temp_filename, self.samplerate, recording_data)
        rospy.loginfo(f"Recording saved to {self.temp_filename}")

    def transcribe_audio(self):
        """
        Transcribes the saved audio file using the Whisper model.
        """
        if not self.model:
            rospy.logerr("Cannot transcribe, Whisper model not loaded.")
            return ""
            
        rospy.loginfo("Transcribing audio...")
        try:
            result = self.model.transcribe(self.temp_filename, fp16=False) # fp16=False for CPU
            transcribed_text = result['text']
            rospy.loginfo(f"Transcription result: '{transcribed_text}'")
            return transcribed_text
        except Exception as e:
            rospy.logerr(f"An error occurred during transcription: {e}")
            return ""

    def listen_for_command(self, duration=5):
        """
        A simple high-level function to record and transcribe a user command.
        
        MODIFIED: This function no longer prompts the user.
        It immediately records audio.
        """
        if not self.model:
            # This part can probably be simplified as the check is also in the main controller
            rospy.logerr("Whisper model is not available.")
            return "Whisper model is not available."

        # THE INPUT PROMPT IS REMOVED FROM HERE
        # input(">> Press Enter to start speaking, you will have 5 seconds...")
        
        recording = self.record_audio(duration)
        self.save_recording(recording)
        command = self.transcribe_audio()
        return command.strip()