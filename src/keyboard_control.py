import tkinter as tk
from tkinter import ttk
import os
import time
import sys
import serial


class Crosshair():
    """docstring for Crosshair."""

    def __init__(self, frame, w, h):
        super(Crosshair, self).__init__()
        self.p_frame = frame
        self.w = w
        self.h = h
        self.ch_thickness =2
        self._create_widget()

    def calc_pos(self):
        # Plot horizontal line
        x1_1 = self.p_frame.x - self.w/2
        x1_2 = self.p_frame.x + self.w/2
        y1_1 = self.p_frame.y - self.ch_thickness
        y1_2 = self.p_frame.y + self.ch_thickness

        # Plot vertical line
        x2_1 = self.p_frame.x - self.ch_thickness
        x2_2 = self.p_frame.x + self.ch_thickness
        y2_1 = self.p_frame.y - self.h/2
        y2_2 = self.p_frame.y + self.h/2


        return x1_1, y1_1, x1_2, y1_2, x2_1, y2_1, x2_2, y2_2

    def _create_widget(self):
        """
        USAGE: Function for the creation of the crosshair object that is used
        for targeting the robot
        """
        coords = self.calc_pos()
        self.rect1 = self.p_frame.canv.create_rectangle(
            coords[0], coords[1], coords[2], coords[3], fill="red", outline="red")
        self.rect2 = self.p_frame.canv.create_rectangle(
            coords[4], coords[5], coords[6], coords[7], fill="red", outline="red")

    def move(self):
        """
        USAGE: Function for the movement of the rect
        """
        '''
        coords = self.calc_pos()

        self.rect = self.p_frame.canv.create_rectangle(
            coords[0], coords[1], coords[2], coords[3], fill="red")
        '''
        self.p_frame.canv.move(
            self.rect1, self.p_frame.vel_x, self.p_frame.vel_y)
        self.p_frame.canv.move(
            self.rect2, self.p_frame.vel_x, self.p_frame.vel_y)


class KeyPressClass(object):
    """docstring for KeyPressClass."""

    def __init__(self, root, key):
        super(KeyPressClass, self).__init__()

        # Initlise variables
        self.root = root
        self.key = key
        self.pressed = False
        self.time_since_release = 0

    def _key_pressed(self):
        self.pressed = True

    def check_state(self):
        """
        USAGE: This funciton is for checking the state of the key. It is required becasue of the bug with OSX.
        """

        self.root.calc_vel(self.key, self.pressed)
        self.pressed = False


class KeyboardControlFrame(tk.Frame):
    """docstring for KeyboardControlFrame."""

    def __init__(self, root):
        super(KeyboardControlFrame, self).__init__()

        # Initilise the main frame
        self.root = root

        # canvas variables
        self.canv_w = 400
        self.canv_h = 400

        # Crosshair variables
        self.ch_w = 50
        self.ch_h = 50

        # Cross hair position
        self.x = 50
        self.y = 50
        self.z = 0

        # crosshair velocity
        self.base_vel = 3
        self.vel_x = 0
        self.vel_y = 0
        self.vel_z = 0


        self._create_widget()
        self._bind_keys()

        self._open_serial_port()
        self.root.after(10, self._move_pos)



        # OS key repeat turned off. This is needed to prevent keypress bug

    def _create_widget(self):
        # Generate the position lable
        self.pos_lb_text = tk.StringVar()
        self.pos_lb_text.set("Position:  x: " + str(self.x)
                             + ", y: " + str(self.y) + ", z: " + str(self.z))

        self.pos_lb = tk.Label(self, textvariable=self.pos_lb_text)

        # Genreate the canvas and objects
        self.canv = tk.Canvas(self.root, width=self.canv_w, height=self.canv_h, bg="white")
        self.crosshair = Crosshair(self, self.ch_w, self.ch_h)

        self.canv.pack(pady=10, padx=10)
        self.pos_lb.pack()

    def _bind_keys(self):
        """
        USAGE: Function for the binding of the keypresses
        """

        self.root.bind("<KeyPress>", self._key_press)

        # Check the os
        current_platform = sys.platform
        if current_platform == "linux":
            self.keys = {
                "w": KeyPressClass(self, "Up"),
                "s": KeyPressClass(self, "Down"),
                "a": KeyPressClass(self, "Left"),
                "d": KeyPressClass(self, "Right"),
                "r": KeyPressClass(self, "In"),
                "f": KeyPressClass(self, "Out"),
            }
        else:
            self.keys = {
                "\uf700": KeyPressClass(self, "Up"),
                "\uf701": KeyPressClass(self, "Down"),
                "\uf702": KeyPressClass(self, "Left"),
                "\uf703": KeyPressClass(self, "Right"),
                "r": KeyPressClass(self, "In"),
                "f": KeyPressClass(self, "Out"),
            }

        # self.root.bind("<KeyRelease>", self._key_release)

    def _key_press(self, event):
        """
        USAGE: Function for the decoding the keypress event
        """
        self.keys[event.char]._key_pressed()

    def _open_serial_port(self):
        """
        USAGE: Function checking if the serial port is not open
        """
        # Initilise variables
        self.port_open = False

        try:
            # begin arduino
            serial_port ="/dev/ttyACM0"
            b_rate = 9600
            arduino_timeout = 0.001
            self.arduino = serial.Serial(port=serial_port, baudrate=b_rate, timeout=arduino_timeout)

            self.port_open = True
            self.debug = 0
            self.send_timer = 0
            print("Serial Port is open")
        except IOError:
            print("Serial port not open.")
            print("Check if serial montior is open or if Arduino is plugged in.")


    def _write_to_serial(self):
        """
        USAGE: Function for the serial writing of the poition
        """
        tx_string = str(self.x) + "," + str(self.y) + "\n"
        self.arduino.write(bytes(tx_string, "utf-8"))
        print(tx_string)
        if self.debug:

            data = self.arduino.readline().decode()
            if not data:
                return
            print(data)



    def _move_pos(self):
        """
        USAGE: Function for the movement of the canvas object based on the
        direction traveled
        """

        # Reset velocities
        self.vel_x = 0
        self.vel_y = 0
        self.vel_z = 0

        # Update the setup
        for key in self.keys:
            self.keys[key].check_state()

        # Up date the position
        self.x += self.vel_x
        self.y += self.vel_y
        self.z += self.vel_z

        if self.x < self.ch_w/2:
            self.x = self.ch_w/2
            self.vel_x = 0
        elif self.x > self.canv_w - self.ch_w/2:
            self.x = self.canv_w - self.ch_w/2
            self.vel_x = 0

        if self.y < self.ch_h/2:
            self.y = self.ch_h/2
            self.vel_y = 0
        elif self.y > self.canv_h - self.ch_h/2:
            self.y = self.canv_h - self.ch_h/2
            self.vel_y = 0

        # Update the canv obj
        self.crosshair.move()

        # Update the label
        self.pos_lb_text.set("Position:  x: " + str(self.x)
                             + ", y: " + str(self.y) + ", z: " + str(self.z))


        # Send position to arduino
        if self.port_open:
            if self.send_timer >1000:
                self._write_to_serial()
                self.send_timer = 0
            self.send_timer +=1

        self.root.after(10, self._move_pos)



    def calc_vel(self, dir, state):

        if dir == "Up":
            self.vel_y = -self.base_vel * state
        elif dir == "Down" and self.vel_y == 0:
            # Ensuring that up and down arent being pressed
            self.vel_y = self.base_vel * state

        if dir == "Left":
            self.vel_x = -self.base_vel * state
        elif dir == "Right" and self.vel_x == 0:
            # Ensuring that Left and Right arent being pressed
            self.vel_x = self.base_vel * state
        else:
            pass


def main():
   # Setting up test window and frame
    window = tk.Tk()
    window.title("Keyboard demo")
    # This is required for preventing the keypress bug
    # os.system('xset r off')

    # Calling the function
    frame = KeyboardControlFrame(window)

    frame.pack()
    window.mainloop()


if __name__ == '__main__':
    main()
