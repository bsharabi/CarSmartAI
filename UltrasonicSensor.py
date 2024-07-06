import RPi.GPIO as GPIO
import time
import settings
import threading

class UltrasonicSensor(threading.Thread):
    """
    A singleton class to manage the Ultrasonic Sensor using GPIO pins.
    This class extends threading.Thread to allow asynchronous distance measurement.
    """
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(UltrasonicSensor, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if not hasattr(self, 'initialized'):  # To prevent reinitialization
            super(UltrasonicSensor, self).__init__()
            self.trigger_pin = settings.ULTRASONIC_TR_PIN
            self.echo_pin = settings.ULTRASONIC_EC_PIN

            # Setup GPIO
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.trigger_pin, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.echo_pin, GPIO.IN)

            self.distance = None
            self.__flag = threading.Event()
            self.__terminate = threading.Event()
            self.initialized = True

    def _send_pulse(self):
        """
        Send a short pulse to trigger the ultrasonic sensor.
        """
        GPIO.output(self.trigger_pin, GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(self.trigger_pin, GPIO.LOW)

    def _measure_distance(self):
        """
        Measure the distance by timing the echo response.

        :return: Distance in meters.
        """
        # Wait for the echo response
        while not GPIO.input(self.echo_pin):
            pass
        start_time = time.time()
        while GPIO.input(self.echo_pin):
            if (time.time() - start_time) * 340 / 2 >= 2:
                break
        end_time = time.time()

        # Calculate distance based on the time taken for the echo to return
        distance = (end_time - start_time) * 340 / 2
        return distance

    def get_distance(self):
        """
        Get the measured distance, retrying if necessary to handle invalid readings.

        :return: Distance in meters, or None if the measurement is invalid.
        """
        for _ in range(5):  # Retry up to 5 times to handle invalid readings
            self._send_pulse()
            distance = self._measure_distance()
            if distance < 9:  # Assuming valid readings are less than 9 meters
                self.distance = round(distance, 2) * 100  # Convert to cm
                return self.distance
        self.distance = None
        return None

    def run(self):
        """
        Run the thread to continuously measure distance.
        """
        while not self.__terminate.is_set():
            self.__flag.wait()
            self.get_distance()
            time.sleep(1)  # Adjust the sleep time as needed

    def start_measuring(self):
        """
        Start the distance measurement.
        """
        self.__flag.set()

    def stop_measuring(self):
        """
        Stop the distance measurement.
        """
        self.__flag.clear()

    def terminate(self):
        """
        Terminate the thread safely.
        """
        self.__terminate.set()
        self.__flag.set()  # Ensure the thread exits any wait state
        self.cleanup()

    def cleanup(self):
        """
        Clean up the GPIO settings.
        """
        GPIO.cleanup()

if __name__ == '__main__':
    sensor = UltrasonicSensor()
    sensor.start()  # Start the thread

    try:
        sensor.start_measuring()
        while True:
            distance = sensor.get_distance()
            if distance is not None:
                print(f"{distance:.2f} cm")
            else:
                print("Invalid measurement")
            time.sleep(1)
    except KeyboardInterrupt:
        print("Measurement stopped by user")
    finally:
        sensor.terminate()
        sensor.join()  # Ensure the thread is properly terminated
