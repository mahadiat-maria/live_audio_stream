import rospy
from std_msgs.msg import String
import pyaudio
import base64
import threading

class RoverAudio:
    def __init__(self):

        # ROS setup: Initialize the ROS node FIRST
        rospy.init_node('rover_audio', anonymous=True)
        rospy.loginfo("Rover Audio Node Initialized.")

        # Audio configuration (same as base station)
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
        self.audio_pub = rospy.Publisher('/rover/audio', String, queue_size=10)
        rospy.Subscriber('/base_station/audio', String, self.handle_base_station_audio)


    #Initializes and opens PyAudio input and output streams
    def start_audio_streams(self):
        try:
            self.input_stream = self.audio.open(format=self.FORMAT, channels=self.CHANNELS, rate=self.RATE,
                                                 input=True, frames_per_buffer=self.CHUNK)
            rospy.loginfo("Input audio stream opened successfully.")
            self.output_stream = self.audio.open(format=self.FORMAT, channels=self.CHANNELS, rate=self.RATE,
                                                  output=True, frames_per_buffer=self.CHUNK)
            rospy.loginfo("Output audio stream opened successfully.")
            return True
        except Exception as e:
            rospy.logerr(f"Error initializing audio streams: {e}")
            return False

    #Capture audio and publish to ROS topic
    def audio_capture_thread(self):
        rospy.loginfo("Audio capture thread started.")
        while not rospy.is_shutdown():
            try:
                if self.input_stream:
                    data = self.input_stream.read(self.CHUNK, exception_on_overflow=False)
                    audio_b64 = base64.b64encode(data).decode('utf-8')
                    self.audio_pub.publish(audio_b64)
            except Exception as e:
                rospy.logerr(f"Error capturing audio: {e}")
                break

    #Handle audio from base station
    def handle_base_station_audio(self, data):
        try:
            audio_data = base64.b64decode(data.data)
            if self.output_stream:
                self.output_stream.write(audio_data)
        except Exception as e:
            rospy.logerr(f"Error playing base station audio: {e}")

    #Start the rover audio system
    def start(self):
        rospy.loginfo("Starting Rover Audio System...")
        if not self.start_audio_streams():
            rospy.logerr("Failed to start audio streams. Exiting.")
            return

        # Start audio capture thread
        capture_thread = threading.Thread(target=self.audio_capture_thread)
        capture_thread.daemon = True
        capture_thread.start()
        rospy.loginfo("Audio capture thread launched.")

        rospy.on_shutdown(self.shutdown_hook) # Set up shutdown hook

        rospy.spin() # Keep the node running
    

    #Cleanup when shutting down
    def shutdown_hook(self):

        rospy.loginfo("Shutting down Rover Audio System...")
        if self.input_stream:
            self.input_stream.stop_stream()
            self.input_stream.close()
        if self.output_stream:
            self.output_stream.stop_stream()
            self.output_stream.close()
        if self.audio:
            self.audio.terminate()
        rospy.loginfo("Rover Audio System shutdown complete.")


if __name__ == "__main__":
    try:
        rover = RoverAudio()
        rover.start()
    except rospy.ROSInterruptException:
        rospy.loginfo("Rover Audio Node interrupted.")
    except Exception as e:
        rospy.logerr(f"Unhandled exception: {e}")