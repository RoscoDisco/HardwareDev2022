import serial
import time


def write_read(x, arduino):
    arduino.write(bytes(x, "utf-8"))
    time.sleep(0.05)
    msg = arduino.readline().decode("utf-8")
    print(msg)


def main():
    serial_port = "/dev/cu.usbmodem1411301"
    b_rate = 9600
    arduino_timeout = 0.1
    arduino = serial.Serial(
        port=serial_port, baudrate=b_rate, timeout=arduino_timeout)

    while True:
        num = input("Enter a number: ")  # Taking input from user
        value = write_read(num, arduino)
        print(value)  # printing the value


if __name__ == '__main__':
    main()
