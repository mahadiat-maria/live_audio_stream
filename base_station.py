import rospy
from std_msgs.msg import String
import pyaudio
import base64
import threading
import time

class BaseStationAudio:
    def __init__(self):
        # Audio Configuration
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 44100

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()

        # Audio streams
        self.input_stream = None
        self.output_stream = None

        # ROS Publisher & Subscriber
        self.audio_pub = rospy.Publisher('/base_station/audio', String, queue_size=10)
        rospy.Subscriber('/rover/audio', String, self.handle_rover_audio)

        # ROS setup
        rospy.init_node('base_station_audio', anonymous=True)

    def start_audio_streams(self):
        try:
            self.input_stream = self.audio.open(format=self.FORMAT, channels=self.CHANNELS, rate=self.RATE,
                                                 input=True, frames_per_buffer=self.CHUNK)
            self.output_stream = self.audio.open(format=self.FORMAT, channels=self.CHANNELS, rate=self.RATE,
                                                  output=True, frames_per_buffer=self.CHUNK)
        except Exception as e:
            print(f"Error initializing audio streams: {e}")
            return False
        return True

    def audio_capture_thread(self):
        """Capture audio and publish to ROS topic"""
        while not rospy.is_shutdown():
            try:
                data = self.input_stream.read(self.CHUNK, exception_on_overflow=False)
                audio_b64 = base64.b64encode(data).decode('utf-8')
                self.audio_pub.publish(audio_b64)
                time.sleep(0.1)
            except Exception as e:
                print(f"Error capturing audio: {e}")

    def handle_rover_audio(self, data):
        """Handle audio from rover"""
        try:
            audio_data = base64.b64decode(data.data)
            if self.output_stream:
                self.output_stream.write(audio_data)
        except Exception as e:
            print(f"Error playing rover audio: {e}")

    def start(self):
        """Start the base station audio system"""
        print("Starting Base Station Audio System...")
        if not self.start_audio_streams():
            print("Failed to start audio streams")
            return

        # Start audio capture thread
        capture_thread = threading.Thread(target=self.audio_capture_thread)
        capture_thread.daemon = True
        capture_thread.start()

        rospy.spin()  # Keep the node running

if __name__ == "__main__":
    base_station = BaseStationAudio()
    base_station.start()
