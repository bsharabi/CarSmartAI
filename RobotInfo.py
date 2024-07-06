import psutil
import os
import settings

class RobotInfo:
    def __init__(self):
        """
        Initialize the RobotInfo class.
        """
        self.cpu_temp_path = settings.CPU_TEMP_PATH
        self.gpu_temp_command = settings.GPU_TEMP_COMMAND

    def get_cpu_temp(self):
        """
        Get the CPU temperature.

        :return: CPU temperature in degrees Celsius as a string.
        """
        try:
            with open(self.cpu_temp_path, 'r') as file:
                temp = float(file.read()) / 1000
                return f"{temp:.1f}"
        except FileNotFoundError:
            return "N/A"

    def get_gpu_temp(self):
        """
        Get the GPU temperature.

        :return: GPU temperature in degrees Celsius as a string.
        """
        try:
            temp = os.popen(self.gpu_temp_command).readline().replace("temp=", "").strip()
            return temp
        except Exception:
            return "N/A"

    def get_cpu_usage(self):
        """
        Get the CPU usage percentage.

        :return: CPU usage percentage as a string.
        """
        return f"{psutil.cpu_percent():.1f}"

    def get_ram_usage(self):
        """
        Get the RAM usage percentage.

        :return: RAM usage percentage as a string.
        """
        return f"{psutil.virtual_memory().percent:.1f}"

    def get_swap_usage(self):
        """
        Get the swap memory usage percentage.

        :return: Swap memory usage percentage as a string.
        """
        return f"{psutil.swap_memory().percent:.1f}"

def main():
    robot_info = RobotInfo()

    print("CPU Temperature:", robot_info.get_cpu_temp(), "°C")
    print("GPU Temperature:", robot_info.get_gpu_temp(), "°C")
    print("CPU Usage:", robot_info.get_cpu_usage(), "%")
    print("RAM Usage:", robot_info.get_ram_usage(), "%")
    print("Swap Usage:", robot_info.get_swap_usage(), "%")

if __name__ == '__main__':
    main()
