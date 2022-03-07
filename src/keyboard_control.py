import tkinter as tk
from tkinter import ttk
import os
import time


class Crosshair():
    """docstring for Crosshair."""

    def __init__(self, frame):
        super(Crosshair, self).__init__()
        self.p_frame = frame
        self.rect_w = 100
        self.rect_h = 100
        self._create_widget()

    def calc_pos(self):
        x1_1 = self.p_frame.x
        x2_1 = x1_1 + self.rect_w
        y1_1 = self.p_frame.y
        y2_1 = y1_1 + self.rect_h

        return x1_1, y1_1, x2_1, y2_1

    def _create_widget(self):
        """
        USAGE: Function for the creation of the crosshair object that is used
        for targeting the robot
        """
        coords = self.calc_pos()
        self.rect = self.p_frame.canv.create_rectangle(
            coords[0], coords[1], coords[2], coords[3], fill="red")

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
            self.rect, self.p_frame.vel_x, self.p_frame.vel_y)


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

        self.keys = {
            "\uf700": KeyPressClass(self, "Up"),
            "\uf701": KeyPressClass(self, "Down"),
            "\uf702": KeyPressClass(self, "Left"),
            "\uf703": KeyPressClass(self, "Right"),
            "r": KeyPressClass(self, "In"),
            "f": KeyPressClass(self, "Out"),

        }
        self.root.after(10, self._move_pos)

        # OS key repeat turned off. This is needed to prevent keypress bug

    def _create_widget(self):
        # Generate the position lable
        self.pos_lb_text = tk.StringVar()
        self.pos_lb_text.set("Position:  x: " + str(self.x)
                             + ", y: " + str(self.y) + ", z: " + str(self.z))

        self.pos_lb = tk.Label(self, textvariable=self.pos_lb_text)

        # Genreate the canvas and objects
        self.canv = tk.Canvas(self.root, width=self.canv_w, height=self.canv_h)
        self.crosshair = Crosshair(self)

        self.canv.pack()
        self.pos_lb.pack()

    def _bind_keys(self):
        """
        USAGE: Function for the binding of the keypresses
        """

        self.root.bind("<KeyPress>", self._key_press)

        # self.root.bind("<KeyRelease>", self._key_release)

    def _key_press(self, event):
        """
        USAGE: Function for the decoding the keypress event
        """
        self.keys[event.char]._key_pressed()

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

        if self.x < 0:
            self.x = 0
            self.vel_x = 0
        elif self.x > self.canv_w - 100:
            self.x = self.canv_w - 100
            self.vel_x = 0

        if self.y < 0:
            self.y = 0
            self.vel_y = 0
        elif self.y > self.canv_h - 100:
            self.y = self.canv_h - 100
            self.vel_y = 0

        # Update the canv obj
        self.crosshair.move()

        # Update the label
        self.pos_lb_text.set("Position:  x: " + str(self.x)
                             + ", y: " + str(self.y) + ", z: " + str(self.z))
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
