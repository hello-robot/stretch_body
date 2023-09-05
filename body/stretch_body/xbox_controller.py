#!/usr/bin/env python3
from __future__ import print_function
from inputs import get_gamepad
import threading
import time


class Stick():
    def __init__(self):
        # joystick pushed
        #   all the way down: y = -1.0
        #   all the way up: y ~= 1.0
        #   all the way left: x = -1.0
        #   all the way right: x ~= 1.0
        self.x = 0.0
        self.y = 0.0
        # normalized signed 16 bit integers to be in the range [-1.0, 1.0]
        self.norm = float(pow(2, 15))

    def update_x(self, abs_x):
        self.x = int(abs_x) / self.norm

    def update_y(self, abs_y):
        self.y = -int(abs_y) / self.norm

    def print_string(self):
        return 'x: {0:4.2f}, y:{1:4.2f}'.format(x, y)


class Button():
    def __init__(self):
        self.pressed = False

    def update(self, state):
        if state == 0:
            self.pressed = False
        elif state == 1:
            self.pressed = True

    def print_string(self):
        return str(self.pressed)


class Trigger():
    def __init__(self, xbox_one=False):
        # Xbox One trigger
        #   not pulled = 0
        #   max pulled = 1023
        # normalize unsigned 10 bit integer to be in the range [0.0, 1.0]

        # Xbox 360 trigger
        #   not pulled = 0
        #   max pulled = 255
        # normalize unsigned 8 bit integer to be in the range [0.0, 1.0]
        if xbox_one:
            # xbox one
            num_bits = 10
        else:
            # xbox 360
            num_bits = 8
        self.norm = float(pow(2, num_bits) - 1)
        self.pulled = 0.0

    def update(self, state):
        self.pulled = int(state) / self.norm
        # Ensure that the pulled value is not greater than 1.0, which
        # will can happen with the use of an Xbox One controller, if
        # the option was not properly set.
        if self.pulled > 1.0:
            self.pulled = 1.0

    def print_string(self):
        return '{0:4.2f}'.format(self.pulled)


class XboxController():
    '''Successfully tested with the following controllers:
            + Xbox One Controller connected using a USB cable (change xbox_one parameter to True for full 10 bit trigger information)
            + EasySMX wireless controller set to appropriate mode (Xbox 360 mode with upper half of ring LED illuminated - top two LED quarter circle arcs)
            + JAMSWALL Xbox 360 Wireless Controller (Sometimes issues would occur after inactivity that would seem to require unplugging and replugging the USB dongle.)

       Unsuccessful tests:
            - Xbox One Controller connected via Bluetooth
            - Xbox 360 Controller connected with an Insten Wireless Controller USB Charging Cable
            +/- VOYEE Wired Xbox 360 Controller mostly worked, but it had various issues including false middle LED button presses, phantom shoulder button presses, and low joystick sensitivity that made small motions more difficult to execute.
    '''

    def __init__(self, print_events=False):
        self.print_events = print_events

        self.left_stick = Stick()
        self.right_stick = Stick()

        self.left_stick_button = Button()
        self.right_stick_button = Button()

        self.middle_led_ring_button = Button()

        self.bottom_button = Button()
        self.top_button = Button()
        self.left_button = Button()
        self.right_button = Button()

        self.right_shoulder_button = Button()
        self.left_shoulder_button = Button()

        self.select_button = Button()
        self.start_button = Button()

        self.left_trigger = Trigger(xbox_one=False)
        self.right_trigger = Trigger(xbox_one=False)

        self.left_pad = Button()
        self.right_pad = Button()
        self.top_pad = Button()
        self.bottom_pad = Button()

        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.update)
        self.thread.daemon = True

    def start(self):
        self.thread.start()

    def stop(self):
        pass

    def update(self):
        while True:
            events = get_gamepad()
            with self.lock:
                for event in events:
                    if event.code == 'ABS_X':
                        self.left_stick.update_x(event.state)
                    if event.code == 'ABS_Y':
                        self.left_stick.update_y(event.state)
                    if event.code == 'ABS_RX':
                        self.right_stick.update_x(event.state)
                    if event.code == 'ABS_RY':
                        self.right_stick.update_y(event.state)

                    # This is the glowing X button on an authentic Xbox controller
                    if event.code == 'BTN_MODE':
                        self.middle_led_ring_button.update(event.state)

                    if event.code == 'BTN_SOUTH':  # green A, bottom button
                        self.bottom_button.update(event.state)
                    if event.code == 'BTN_WEST':  # yellow Y, ***top button*** WEIRD!
                        self.top_button.update(event.state)
                    if event.code == 'BTN_NORTH':  # blue X, ***left button*** WEIRD!
                        self.left_button.update(event.state)
                    if event.code == 'BTN_EAST':  # red B, right button
                        self.right_button.update(event.state)

                    if event.code == 'BTN_TL':  # left shoulder button
                        self.left_shoulder_button.update(event.state)
                    if event.code == 'BTN_TR':  # right shoulder button
                        self.right_shoulder_button.update(event.state)

                    if event.code == 'ABS_Z':  # left trigger 0-1023
                        self.left_trigger.update(event.state)
                    if event.code == 'ABS_RZ':  # right trigger 0-1023
                        self.right_trigger.update(event.state)

                    if event.code == 'BTN_SELECT':  # 1/0
                        self.select_button.update(event.state)
                    if event.code == 'BTN_START':  # 1/0
                        self.start_button.update(event.state)

                    if event.code == 'BTN_THUMBL':  # 1/0
                        self.left_stick_button.update(event.state)
                    if event.code == 'BTN_THUMBR':  # 1/0
                        self.right_stick_button.update(event.state)

                    # 4-way pad
                    if event.code == 'ABS_HAT0Y':  # -1 up / 1 down
                        if event.state == 0:
                            self.top_pad.update(0)
                            self.bottom_pad.update(0)
                        elif event.state == 1:
                            self.top_pad.update(0)
                            self.bottom_pad.update(1)
                        elif event.state == -1:
                            self.bottom_pad.update(0)
                            self.top_pad.update(1)

                    if event.code == 'ABS_HAT0X':  # -1 left / 1 right
                        if event.state == 0:
                            self.left_pad.update(0)
                            self.right_pad.update(0)
                        elif event.state == 1:
                            self.left_pad.update(0)
                            self.right_pad.update(1)
                        elif event.state == -1:
                            self.right_pad.update(0)
                            self.left_pad.update(1)

                    if self.print_events:
                        print(event.ev_type, event.code, event.state)
                #time.sleep(0.01)



    def get_state(self):
        with self.lock:
            state = {'middle_led_ring_button_pressed': self.middle_led_ring_button.pressed,
                     'left_stick_x': self.left_stick.x,
                     'left_stick_y': self.left_stick.y,
                     'right_stick_x': self.right_stick.x,
                     'right_stick_y': self.right_stick.y,
                     'left_stick_button_pressed': self.left_stick_button.pressed,
                     'right_stick_button_pressed': self.right_stick_button.pressed,
                     'bottom_button_pressed': self.bottom_button.pressed,
                     'top_button_pressed': self.top_button.pressed,
                     'left_button_pressed': self.left_button.pressed,
                     'right_button_pressed': self.right_button.pressed,
                     'left_shoulder_button_pressed': self.left_shoulder_button.pressed,
                     'right_shoulder_button_pressed': self.right_shoulder_button.pressed,
                     'select_button_pressed': self.select_button.pressed,
                     'start_button_pressed': self.start_button.pressed,
                     'left_trigger_pulled': self.left_trigger.pulled,
                     'right_trigger_pulled': self.right_trigger.pulled,
                     'bottom_pad_pressed': self.bottom_pad.pressed,
                     'top_pad_pressed': self.top_pad.pressed,
                     'left_pad_pressed': self.left_pad.pressed,
                     'right_pad_pressed': self.right_pad.pressed}
        return state


def main():
    xbox_controller = XboxController(print_events=True)
    xbox_controller.start()
    try:
        while True:
            state = xbox_controller.get_state()
            print('------------------------------')
            print('XBOX CONTROLLER STATE')
            for k in state.keys():
                print(k, ' : ', state[k])
            print('------------------------------')
            time.sleep(1.0)
    except (KeyboardInterrupt, SystemExit):
        pass


if __name__ == "__main__":
    main()
