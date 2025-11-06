# check_audio.py
import sounddevice as sd

try:
    print("Querying audio devices...")
    print(sd.query_devices())
    print("\nTest successful. At least one audio device was found.")
except Exception as e:
    print(f"\nAn error occurred: {e}")
    print("This likely means no audio devices are accessible inside the container.")