import tkinter as tk
import os
import sys
import argparse
from KeyboardControlGUI import KeyboardControlFrame


def main():
    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--arduino_port", "-p",
                        help="USB port of the arduino.",
                        default="/dev/cu.usbmodem1411301")
    parser.add_argument(
        "--debug", "-d", help="Flag used for testing and debugging", default=0)

    args = parser.parse_args()
    arduino_port = args.arduino_port
    debug = args.debug

   # Setting up test window and frame
    window = tk.Tk()
    window.title("Keyboard demo")

    # Check the os
    current_platform = sys.platform
    print("Current OS: " + current_platform)

    # Calling the function
    frame = KeyboardControlFrame(
        window,
        os=current_platform,
        port=arduino_port,
        debug=debug)

    frame.grid()
    window.mainloop()


if __name__ == '__main__':
    main()
