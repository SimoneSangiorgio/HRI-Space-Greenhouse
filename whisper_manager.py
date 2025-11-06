# whisper_manager.py
# FINAL, ROBUST VERSION using the proven callback-based audio stream logic.

import rospy
import sounddevice as sd
import numpy as np
from faster_whisper import WhisperModel
import threading

class WhisperManager:
    def __init__(self, model_size="base"):
        """
        Initializes the WhisperManager using faster-whisper and a robust,
        callback-based audio stream to prevent hardware lock issues.
        """
        rospy.loginfo("Initializing WhisperManager (Robust Callback Edition)...")
        rospy.loginfo(f"Loading faster-whisper model '{model_size}'...")
        try:
            # "base" is a great balance of speed and accuracy.
            self.model = WhisperModel(model_size, device="cpu", compute_type="int8")
            rospy.loginfo("Faster-whisper model loaded successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to load faster-whisper model: {e}")
            self.model = None

        self.samplerate = 16000
        self.channels = 1
        self.is_recording = False
        self.recorded_frames = []
        self.stream = None
        self.lock = threading.Lock() # To prevent race conditions

    def _audio_callback(self, indata, frames, time, status):
        """This function is called by a separate thread for each new audio chunk."""
        if status:
            rospy.logwarn(f'Sounddevice status: {status}')
        if self.is_recording:
            self.recorded_frames.append(indata.copy())

    def listen_for_command(self, duration=5):
        """
        Orchestrates the recording and transcription using a reliable callback stream.
        This function blocks for the duration of the recording.
        """
        if not self.model:
            rospy.logerr("Cannot listen, whisper model not loaded.")
            return "Whisper model is not available."

        with self.lock:
            if self.is_recording:
                rospy.logwarn("Already recording. Request ignored.")
                return ""

            self.recorded_frames = []
            self.is_recording = True
        
        rospy.loginfo(f"Recording for {duration} seconds...")

        try:
            # The stream is created and started here
            self.stream = sd.InputStream(
                samplerate=self.samplerate,
                channels=self.channels,
                callback=self._audio_callback,
                dtype='float32',
                device=0 # IMPORTANT: Ensure this is your correct microphone index!
            )
            self.stream.start()
            
            # Wait for the specified duration
            rospy.sleep(duration)

        finally:
            # This block guarantees that the stream is stopped and closed
            if self.stream:
                self.stream.stop()
                self.stream.close()
                self.stream = None
            self.is_recording = False
            rospy.loginfo("Recording finished and stream closed.")

        # --- Process and Transcribe the collected audio frames ---
        with self.lock:
            if not self.recorded_frames:
                rospy.logwarn("No audio frames were recorded.")
                return ""
            recording_data = np.concatenate(self.recorded_frames, axis=0)

        rospy.loginfo("Transcribing audio from memory...")
        try:
            segments, info = self.model.transcribe(recording_data.flatten(), beam_size=5, language="en")
            transcribed_text = "".join(segment.text for segment in segments).strip()
            rospy.loginfo(f"Transcription result: '{transcribed_text}'")
            return transcribed_text
        except Exception as e:
            rospy.logerr(f"An error occurred during transcription: {e}")
            return ""

    def shutdown(self):
        """A cleanup method to ensure the stream is stopped if the app closes unexpectedly."""
        rospy.loginfo("WhisperManager shutting down.")
        if self.stream:
            self.stream.stop()
            self.stream.close()