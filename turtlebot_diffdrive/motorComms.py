import serial
import time
import struct
from serial.tools import list_ports
from turtlebot_diffdrive.configuration import *

class MotorMessage:
    def __init__(self):
        self.left_wheel_encoder_count = 0
        self.right_wheel_encoder_count = 0
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0

class MotorController:
    # def __init__(self):
    #     self.port

    def initHandshake(self,port=""):
        try:
            if port=="":
                self.port=self.scan_devices()[0]
            self.serial = serial.Serial(self.port, BAUD_RATE, timeout=TIMEOUT)
            return True
        except Exception as e:
            print("Error opening serial port: ", e)
            return False
        
    @staticmethod
    def scan_devices():
        """Scan for available ports."""
        ports = list_ports.comports()
        return [port.device for port in ports]
        
    def shutdown(self):
        if self.serial.is_open:
            self.send_velocity_command(0, 0)
            self.serial.close()
        else:
            print("[motorComms] Error closing serial port!")

    def calculate_receive_checksum(self,data):
        """Calculate checksum for received data."""
        return sum(data[2:50]) % 256  

    def calculate_transmit_checksum(self,data):
        """Calculate checksum for data to be transmitted."""
        return sum(data[3:11]) % 256

    def send_velocity_command(self,left_wheel_vel, right_wheel_vel):
        """Send velocity command to turtlebot."""
        data = bytearray(SIZE_OF_RX_DATA)
        data[0:2] = HEADER, HEADER
        data[2] = VELOCITY_MODE

        # Packing velocity into 4 bytes
        data[3:7] = struct.pack('f', left_wheel_vel)  # velocity x
        data[7:11] = right_wheel_vel # velocity y (not neccesary for diff-drive)
        checksum = self.calculate_transmit_checksum(data)
        data[SIZE_OF_RX_DATA - 2] = checksum
        data[SIZE_OF_RX_DATA - 1] = TAIL
        self.serial.write(data)
        self.serial.flush() #flush data stream

    def read_serial_data(self,motorMessage):
        """Read data from serial and process it."""
        if self.serial.in_waiting > 0:
            received_data = self.serial.read(self.serial.in_waiting)
            if len(received_data) >= SIZE_OF_TX_DATA and received_data[0:2] == bytes([HEADER, HEADER]):
                if self.calculate_receive_checksum(received_data) == received_data[SIZE_OF_TX_DATA-2]:
                    unpacked_data = struct.unpack('f'*6, received_data[2:SIZE_OF_TX_DATA-2])
                    #print(unpacked_data)
                    motorMessage.left_wheel_encoder_count = unpacked_data[0]
                    motorMessage.right_wheel_encoder_count = unpacked_data[1]
                    motorMessage.left_wheel_vel = unpacked_data[2]
                    motorMessage.right_wheel_vel = unpacked_data[3]
                else:
                    print("[motorComms] Checksum mismatch")
            else:
                print("[motorComms] Header mismatch")

def main():
    """Main function to control omni-wheel car."""
    esp = MotorController()
    motorMessage = MotorMessage()
    connect_bool = esp.initHandshake()
    if not connect_bool: 
        print("[motorComms] Error opening serial port!")
    else: 
        print("[motorComms] Serial port opened successfully!")
    print(f"[motorComms] Using port: {esp.port}")
    try:
        while True:
            speed = float(input("Enter speed: "))
            esp.send_velocity_command(speed, 0)
            esp.read_serial_data(motorMessage)
            print(f"[motorComms] Linear Vel: {motorMessage.velocity_x}")
            print(f"[motorComms] Angular Vel: {motorMessage.velocity_angular}\n")
            # for i in range(10):
            #     start_time = time.time()
            #     while time.time() - start_time < 10:
            #         esp.send_velocity_command(i * 0.05, 0)
            #         esp.read_serial_data(motorMessage)
            #         print(f"Linear Vel: {motorMessage.velocity_x}")
            #         print(f"Angular Vel: {motorMessage.velocity_angular}\n")
            #         time.sleep(0.1)
    except KeyboardInterrupt:
        esp.shutdown()
        print("[motorComms] Program stopped by user")
    finally:
        esp.shutdown()

if __name__ == "__main__":
    main()

