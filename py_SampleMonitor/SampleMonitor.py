import serial
import time
import argparse


def send_command(port: str, baudrate: int, command: str, timeout: float = 1.0):
    try:
        with serial.Serial(port, baudrate, timeout=timeout) as ser:
            print(f"Sending command: {command}")
            ser.write((command + '\n').encode())
            time.sleep(0.1)

            response = ser.read_until().decode().strip()
            print("Received response:\n")
            print(response + "\n")
            return 0
    except serial.SerialException as e:
        print(f"Error: {e}")
        return None


def main():
    parser = argparse.ArgumentParser(
        description="Send command over serial port and read response.")
    parser.add_argument(
        "port", type=str, help="COM port (e.g., COM3 or /dev/ttyUSB0)")
    parser.add_argument("baudrate", type=int, nargs="?", default=115200,
                        help="Baud rate (default: 115200)")
    parser.add_argument("command", type=str, nargs="?", default="GET_IMU",
                        help="Command to send over serial port")

    args = parser.parse_args()
    send_command(args.port, args.baudrate, args.command)


if __name__ == "__main__":
    main()
