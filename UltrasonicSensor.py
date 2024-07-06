import RPi.GPIO as GPIO
import time

class UltrasonicSensor:
    def __init__(self, trigger_pin, echo_pin):
        """
        Initialize the Ultrasonic Sensor with the specified GPIO pins.

        :param trigger_pin: GPIO pin connected to the trigger pin of the ultrasonic sensor.
        :param echo_pin: GPIO pin connected to the echo pin of the ultrasonic sensor.
        """
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin

        # Setup GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.echo_pin, GPIO.IN)

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
                return round(distance, 2)
        return None

    def cleanup(self):
        """
        Clean up the GPIO settings.
        """
        GPIO.cleanup()

if __name__ == '__main__':
    sensor = UltrasonicSensor(trigger_pin=11, echo_pin=8)
    
    try:
        while True:
            distance = sensor.get_distance()*100
            if distance is not None:
                print(f"{distance:.2f} cm")
            else:
                print("Invalid measurement")
            time.sleep(1)
    except KeyboardInterrupt:
        print("Measurement stopped by user")
    finally:
        sensor.cleanup()
