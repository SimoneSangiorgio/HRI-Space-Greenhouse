#!/usr/bin/env python

import rospy
try:
    from sound_play.libsoundplay import SoundClient
except ImportError:
    from sound_play.sound_client import SoundClient

class SpeechManager:
    """A simpler wrapper class to make TIAGo speak using sound_play."""
    def __init__(self):
        rospy.loginfo("Initializing Speech Manager (using sound_play)...")
        
        # Create a sound client instance
        # The 'blocking=True' argument makes it wait until the soundplay_node is ready.
        self.sound_client = SoundClient(blocking=True)
        
        # Wait a moment for publishers to connect
        rospy.sleep(1)
        
        rospy.loginfo("Connected to sound_play server.")

    def say(self, text):
        """
        Sends a string of text to the sound_play node to be spoken.
        :param text: The string to be spoken.
        """
        # The voice parameter can be changed to get different accents
        self.sound_client.say(text, voice='voice_kal_diphone')

    def shutdown(self):
        """Cleanly unregisters from the sound_play node."""
        self.sound_client.stopAll()
        rospy.loginfo("Speech manager shut down.")