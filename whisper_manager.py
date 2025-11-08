# whisper_manager.py
# DEFINITIVE VERSION with forced audio subsystem re-initialization.
# This is designed to combat stubborn environmental hardware locks.

import rospy
import sounddevice as sd
import numpy as np
from faster_whisper import WhisperModel
import threading

class WhisperManager:
    def __init__(self, model_size="base"):
        rospy.loginfo("Initializing WhisperManager (Forced Reset Edition)...")
        rospy.loginfo(f"Loading faster-whisper model '{model_size}'...")
        try:
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
        self.lock = threading.Lock()

    def _audio_callback(self, indata, frames, time, status):
        if status:
            rospy.logwarn(f'Sounddevice status: {status}')
        if self.is_recording:
            self.recorded_frames.append(indata.copy())

    def start_listening(self):
        with self.lock:
            if self.is_recording:
                rospy.logwarn("Start listening called when already recording.")
                return False
            
            # --- THE DEFINITIVE FIX: FORCE AUDIO SUBSYSTEM RESET ---
            try:
                rospy.loginfo("Forcing audio subsystem reset...")
                sd._terminate() # Completely release all audio devices.
                sd._initialize() # Re-scan and re-initialize all audio devices.
                rospy.loginfo("Audio subsystem has been reset.")
            except Exception as e:
                rospy.logerr(f"Failed during audio subsystem reset: {e}")
                # This is a critical failure, we cannot proceed.
                return False
            # --- END OF FIX ---

            rospy.loginfo("Starting audio recording stream...")
            self.recorded_frames = []
            self.is_recording = True
            try:
                self.stream = sd.InputStream(
                    samplerate=self.samplerate,
                    channels=self.channels,
                    callback=self._audio_callback,
                    dtype='float32',
                    device=0 # IMPORTANT: Ensure this is your correct microphone index!
                )
                self.stream.start()
                rospy.loginfo("Stream started. Now listening...")
                return True
            except Exception as e:
                rospy.logerr(f"Failed to start audio stream after reset: {e}")
                self.is_recording = False
                return False

    def stop_listening_and_transcribe(self):
        with self.lock:
            if not self.is_recording:
                # This case is now normal if the start failed, so we don't need a warning.
                return ""
            
            rospy.loginfo("Stopping audio recording stream...")
            if self.stream:
                self.stream.stop()
                self.stream.close()
                self.stream = None
            self.is_recording = False
            rospy.loginfo("Stream stopped and closed.")

            if not self.recorded_frames:
                rospy.logwarn("No audio frames were recorded.")
                return ""
            
            recording_data = np.concatenate(self.recorded_frames, axis=0)

        if np.max(np.abs(recording_data)) < 0.01:
            rospy.logwarn("Audio signal appears to be silent. Skipping transcription.")
            return ""

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
        rospy.loginfo("WhisperManager shutting down.")
        if self.stream:
            self.stream.stop()
            self.stream.close()
        # Ensure the audio subsystem is terminated on final shutdown
        sd._terminate()